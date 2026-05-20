/**
 * @file crystal_slicer.cpp
 * @brief Step-1 implementation of Crystal.
 *
 * For now produces only the planar base layers (first N Z-slices) by
 * delegating to PlanarSlicer and truncating the result. The remaining
 * pipeline stages (scalar field, iso-extraction, branch tracking, ...)
 * are filled in by subsequent steps.
 */

#include "crystal_slicer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "alternating_scheduler.h"
#include "branch_tracker.h"
#include "crystal_infill.h"
#include "common/infill_generator.h"
#include "iso_extractor.h"
#include "logger.h"
#include "mesh_refiner.h"
#include "planar/planar_slicer.h"
#include "triangle_mesh.h"

namespace slicing::crystal
{
    CrystalSlicer::CrystalSlicer(CrystalParams params)
        : mParams(std::move(params))
    {
        if (!mParams.scheduler)
        {
            mParams.scheduler = std::make_shared<AlternatingScheduler>();
        }

        const float effective_dh =
            (mParams.iso_layer_thickness > 0.0f) ? mParams.iso_layer_thickness
                                                 : mParams.layer_thickness;

        MEDUSA_INFO("Crystal: instantiated (planar_base={}, iso_dh={:.3f} mm, walls={}, "
                    "infill_spacing={:.3f} mm, scheduler='{}')",
                    mParams.planar_base_layers, effective_dh, mParams.wall_count,
                    mParams.infill_spacing, mParams.scheduler->name());
    }

    std::vector<Layer> CrystalSlicer::slice(const geometry::TriangleMesh& mesh)
    {
        mToolpath = Toolpath{};
        mScalarField = ScalarField{};
        mRefinedMesh = {};
        mCoverage = 0.0f;

        if (!mesh.isValid())
        {
            MEDUSA_WARN("Crystal::slice: mesh is empty - skipping");
            return {};
        }

        MEDUSA_INFO("Crystal::slice: mesh has {} vertices, {} faces - "
                    "steps 1-2 (planar base + harmonic Phi; iso-layers TODO)",
                    mesh.numVertices(), mesh.numFaces());

        // ---- Step 1: planar base layers via the existing PlanarSlicer ----
        // PlanarSlicer slices the full height; truncate to the first N layers
        // for bed adhesion. Iso-layers will be appended in later steps.
        SlicerParams planar_params;
        planar_params.layer_thickness = mParams.layer_thickness;
        planar_params.up_axis         = mParams.up_axis;

        PlanarSlicer planar(planar_params);
        std::vector<Layer> all_layers = planar.slice(mesh);

        const int requested = std::max(0, mParams.planar_base_layers);
        const auto take = std::min<std::size_t>(static_cast<std::size_t>(requested),
                                                all_layers.size());
        std::vector<Layer> base(all_layers.begin(), all_layers.begin() + take);

        // Force layer.index / branch_id to Crystal convention.
        for (std::uint32_t i = 0; i < base.size(); ++i)
        {
            base[i].index     = i;
            base[i].branch_id = 0;          // base trunk
        }

        // Coverage: fraction of base layers we managed to extract from the
        // requested count. 1.0 once iso-layers are wired in (later steps).
        mCoverage = (requested > 0)
            ? static_cast<float>(base.size()) / static_cast<float>(requested)
            : 0.0f;

        MEDUSA_INFO("Crystal::slice: emitted {}/{} planar base layers (coverage={:.1f}%)",
                    base.size(), requested, mCoverage * 100.0f);

        // ---- GAP DIAGNOSTICS: log y-centers of all planar base layers ----
        const int axis = std::clamp(mParams.up_axis, 0, 2);
        std::vector<float> planar_y_centers;
        planar_y_centers.reserve(base.size());
        for (const auto& layer : base)
        {
            if (layer.contour_points.empty()) continue;
            double y_sum = 0.0;
            for (const auto& pt : layer.contour_points)
                y_sum += static_cast<double>(pt[axis]);
            const float mean_y = static_cast<float>(y_sum / layer.contour_points.size());
            planar_y_centers.push_back(mean_y);
        }
        if (!planar_y_centers.empty())
        {
            std::string y_list;
            for (std::size_t i = 0; i < planar_y_centers.size(); ++i)
            {
                if (i > 0) y_list += ", ";
                y_list += std::to_string(static_cast<int>(std::round(planar_y_centers[i] * 100.0f))) + "e-2";
            }
            MEDUSA_INFO("Crystal GAP-CHECK: planar_base y-centers=[{}]", y_list);
        }

        // ---- Step 1.5: refine a working copy of the mesh ----
        // Phi-solve, gradient and iso-extraction all run on this refined
        // copy; the original mesh is left untouched (visualisation, other
        // slicers, etc.). Skip if the user disabled refinement
        // (refine_max_edge_length <= 0).
        const geometry::TriangleMesh* phi_mesh = &mesh;
        geometry::TriangleMesh refined;
        if (mParams.refine_max_edge_length > 0.0f)
        {
            refined = mesh;
            MeshRefinerConfig rcfg;
            rcfg.min_edge_length = mParams.refine_max_edge_length;
            if (refineMesh(refined, rcfg))
            {
                phi_mesh = &refined;
            }
            else
            {
                MEDUSA_WARN("Crystal::slice: mesh refinement failed - "
                            "falling back to original mesh for Phi-solve");
            }
        }
        // Cache the (refined or original) Phi-mesh so visualisers can index
        // sf.phi against the same vertex set the solve used.
        mRefinedMesh = *phi_mesh;

        // ---- Step 2: harmonic scalar field Phi over the refined mesh ----
        // Step 3 will replace the bottom Dirichlet set with the actual top
        // boundary of the planar base layers; for now the bottom band of the
        // bounding box serves as a reasonable proxy that already produces a
        // monotone field on cube / T / hourglass test geometries.
        ScalarFieldConfig sfg;
        sfg.up_axis = mParams.up_axis;
        mScalarField = solveHarmonicField(*phi_mesh, sfg);
        if (!mScalarField.valid)
        {
            MEDUSA_WARN("Crystal::slice: harmonic field solve failed - "
                        "iso-layer stages will be skipped");
        }

        // ---- Step 3: iso-surface extraction ---------------------------------
        // Convert the world-space top of the planar base layers to a phi-value
        // by linear interpolation along the up-axis. For monotone fields (the
        // common case once the harmonic boundary conditions are sane) this
        // closely matches the actual phi at that height; non-monotone meshes
        // get a slight overlap with the base, which is harmless and goes away
        // once Step 3' anchors phi=0 to the actual base-top boundary.
        if (mScalarField.valid)
        {
            const int axis = std::clamp(mParams.up_axis, 0, 2);
            const float h_min = phi_mesh->bounds.min[axis];
            const float h_max = phi_mesh->bounds.max[axis];
            const float h_range = std::max(h_max - h_min, 1e-6f);

            // phi_start must correspond to the POSITION of the last planar
            // layer (its slice height), not the volume-top of the base region.
            // PlanarSlicer places layers at yMin + (i+0.5)*h, so the last one
            // sits at yMin + (N-0.5)*h. The iso-extractor then adds its own
            // half-step offset, resulting in center-to-center = layer_thickness.
            const float base_top_world =
                h_min + (static_cast<float>(base.size()) - 0.5f) * mParams.layer_thickness;
            const float t_base = std::clamp((base_top_world - h_min) / h_range, 0.0f, 1.0f);
            const float phi_start =
                mScalarField.phi_min + t_base * (mScalarField.phi_max - mScalarField.phi_min);

            // Target height for the first iso-layer center.
            const float first_iso_target_y = base_top_world + mParams.layer_thickness;

            MEDUSA_INFO("Crystal: planar base layers: N={} at y-centers=[{:.2f}, ..., {:.2f}]",
                        base.size(),
                        h_min + 0.5f * mParams.layer_thickness,
                        base_top_world);
            MEDUSA_INFO("Crystal: phi_start={:.6f} (linear t={:.4f}) -> "
                        "first iso-layer TARGETED at y~{:.2f} (offset={:.2f} mm from last base)",
                        phi_start, t_base, first_iso_target_y,
                        first_iso_target_y - base_top_world);

            IsoExtractorConfig icfg;
            icfg.up_axis         = axis;
            icfg.layer_thickness = (mParams.iso_layer_thickness > 0.0f)
                                   ? mParams.iso_layer_thickness
                                   : mParams.layer_thickness;
            icfg.phi_start       = phi_start;
            icfg.phi_end         = mScalarField.phi_max;

            auto iso_layers = extractIsoLayers(*phi_mesh, mScalarField, icfg,
                                               static_cast<std::uint32_t>(base.size()));

            // ---- GAP DIAGNOSTICS: compute all three gap values ----
            if (!planar_y_centers.empty() && !iso_layers.empty())
            {
                // Extract y-center of first iso-layer.
                float first_iso_y = -1.0f;
                if (!iso_layers.front().contour_points.empty())
                {
                    double y_sum = 0.0;
                    for (const auto& pt : iso_layers.front().contour_points)
                        y_sum += static_cast<double>(pt[axis]);
                    first_iso_y = static_cast<float>(y_sum / iso_layers.front().contour_points.size());
                }

                // Compute gaps.
                const float gap_planar = (planar_y_centers.size() >= 2)
                    ? planar_y_centers.back() - planar_y_centers[planar_y_centers.size() - 2]
                    : -1.0f;
                const float gap_to_iso = (first_iso_y > 0.0f)
                    ? first_iso_y - planar_y_centers.back()
                    : -1.0f;
                const float target = mParams.layer_thickness;

                if (gap_planar > 0.0f && gap_to_iso > 0.0f)
                {
                    const float delta = gap_to_iso - target;
                    MEDUSA_INFO("Crystal GAP-CHECK: planar_spacing={:.2f}, transition_spacing={:.2f}, "
                                "target={:.2f} (delta_transition={:+.2f} mm)",
                                gap_planar, gap_to_iso, target, delta);
                }
            }

            // Append iso-layers to the planar base.
            base.reserve(base.size() + iso_layers.size());
            for (auto& l : iso_layers) base.push_back(std::move(l));

            mCoverage = 1.0f;

            // ---- Step 4: branch tracking + scheduling ----
            // Tag each iso-layer with a stable branch_id by tracking
            // components across iso-steps, then reorder layers via the
            // configured strategy (default: alternating round-robin).
            BranchTrackerConfig btcfg;
            btcfg.first_iso_layer_index = static_cast<std::uint32_t>(base.size() - iso_layers.size());
            (void)assignBranchIds(base, btcfg);

            // ---- Step 5: infill generation -----------------------------------
            // Planar base layers go through the existing axis-aligned scanline
            // generator; iso-layers use the tangent-plane projection variant
            // because their points do not share a constant up-axis coordinate.
            const std::size_t planar_count = base.size() - iso_layers.size();
            if (mParams.infill_spacing > 0.0f && planar_count > 0)
            {
                std::vector<Layer> planar_subset(base.begin(),
                                                 base.begin() + static_cast<std::ptrdiff_t>(planar_count));
                InfillGenerator::generate(planar_subset, mParams.infill_spacing);
                std::move(planar_subset.begin(), planar_subset.end(), base.begin());
            }
            if (mParams.infill_spacing > 0.0f && iso_layers.size() > 0)
            {
                std::vector<Layer> iso_subset(base.begin() + static_cast<std::ptrdiff_t>(planar_count),
                                              base.end());
                generateNonPlanarInfill(iso_subset, mParams.infill_spacing, *phi_mesh);
                std::move(iso_subset.begin(), iso_subset.end(),
                          base.begin() + static_cast<std::ptrdiff_t>(planar_count));
            }

            scheduleLayers(*mParams.scheduler, base);
            MEDUSA_INFO("Crystal::slice: scheduler '{}' produced {} layer ops",
                        mParams.scheduler->name(), base.size());

            // Build the 6-DOF toolpath ourselves so branch_id is preserved
            // end-to-end and travel moves on branch switches use the right
            // motion type. SlicingPipeline detects a non-empty toolpath and
            // skips its generic InfillGenerator/PathPlanner stages.
            mToolpath = buildToolpath(base);
        }

        // mToolpath is intentionally left empty: until step 6 produces real
        // 6-DOF segments, the pipeline falls through to InfillGenerator +
        // PathPlanner so the user sees the contour rings in the viewer.

        // TODO (Step 4): BranchTracker -> connected components / branch_id
        // TODO (Step 5): PathGenerator + PoseBuilder -> 6-DOF segments
        // TODO (Step 6): IBranchScheduler::schedule -> ordered Toolpath

        return base;
    }

    Toolpath CrystalSlicer::buildToolpath(const std::vector<Layer>& layers) const
    {
        Toolpath tp;
        if (layers.empty()) return tp;

        if (!layers.front().contour_points.empty())
        {
            tp.origin = layers.front().contour_points.front();
        }

        // Travel-move detection: if the previous segment's end and the next
        // segment's start are further apart than this threshold (mm), we
        // insert a Travel segment between them. Branch switches always
        // trigger a travel by definition.
        constexpr float kTravelGapMm = 1e-3f;

        std::uint32_t prev_branch = std::numeric_limits<std::uint32_t>::max();
        glm::vec3 cursor(0.0f);
        bool      have_cursor = false;

        std::uint32_t travel_count = 0;
        std::uint32_t print_count  = 0;

        for (const auto& layer : layers)
        {
            const auto& pts = layer.contour_points;
            const auto& nrm = layer.contour_normals;
            if (pts.size() < 2) continue;

            const std::size_t contour_end =
                (layer.contour_count > 0) ? layer.contour_count : pts.size();

            for (std::size_t i = 0; i + 1 < pts.size(); i += 2)
            {
                const glm::vec3& a = pts[i];
                const glm::vec3& b = pts[i + 1];

                // Insert travel if branch switched or if there is a position
                // gap between the previous segment's end and this segment's
                // start.
                if (have_cursor)
                {
                    const bool branch_switch = (layer.branch_id != prev_branch);
                    const float gap = glm::length(a - cursor);
                    if (branch_switch || gap > kTravelGapMm)
                    {
                        Segment t;
                        t.start       = cursor;
                        t.end         = a;
                        t.orientation = (i + 1 < nrm.size())
                                        ? nrm[i] : glm::vec3(0, 1, 0);
                        t.branch_id   = layer.branch_id;
                        t.growth_step = layer.index;
                        t.is_infill   = false;
                        t.is_travel   = true;
                        tp.segments.push_back(t);
                        ++travel_count;
                    }
                }

                Segment s;
                s.start       = a;
                s.end         = b;
                if (i < nrm.size() && i + 1 < nrm.size())
                {
                    glm::vec3 n = nrm[i] + nrm[i + 1];
                    const float l = glm::length(n);
                    s.orientation = (l > 1e-8f) ? (n / l) : glm::vec3(0, 1, 0);
                }
                s.branch_id   = layer.branch_id;
                s.growth_step = layer.index;
                s.is_infill   = (i >= contour_end);
                s.is_travel   = false;
                tp.segments.push_back(s);
                ++print_count;

                cursor      = b;
                have_cursor = true;
                prev_branch = layer.branch_id;
            }
        }

        MEDUSA_INFO("Crystal::buildToolpath: {} segments ({} print, {} travel)",
                    tp.segments.size(), print_count, travel_count);
        return tp;
    }
} // namespace slicing::crystal
