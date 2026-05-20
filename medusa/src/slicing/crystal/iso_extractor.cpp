/**
 * @file iso_extractor.cpp
 * @brief Implementation of Crystal Step 3 — iso-surface contour extraction.
 *
 * Strategy mirrors PlanarSlicer::intersectPlane / chain step but parametrised
 * over an arbitrary scalar field instead of a single coordinate axis. Each
 * triangle is treated as a linearly-interpolated function phi(barycentric);
 * for an iso-value phi* the intersection with the triangle is a single line
 * segment whose endpoints lie on two of the triangle's edges.
 *
 * Greedy nearest-endpoint chaining (O(n^2) per layer) groups segments into
 * closed rings. This matches the existing PlanarSlicer behaviour so downstream
 * stages (InfillGenerator / PathPlanner) consume identical structures.
 */

#include "iso_extractor.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <string>

#include "logger.h"
#include "triangle_mesh.h"

namespace slicing::crystal
{
    namespace
    {
        struct RawEdge { glm::vec3 a, b; std::uint32_t face_id; };

        /// Output of chainRings: a ring as a list of (point, face_id) entries.
        /// face_id is the mesh face that produced the segment which contributed
        /// this point (used downstream by PoseBuilder to look up -grad Phi).
        struct RingPoint { glm::vec3 p; std::uint32_t face_id; };
        using Ring = std::vector<RingPoint>;

        /// Segment-vs-iso intersection for a single triangle.
        /// Writes 0 or 2 points into @p out and returns the count.
        int triangleIso(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                        float p0, float p1, float p2, float iso, glm::vec3 out[2])
        {
            const float d0 = p0 - iso;
            const float d1 = p1 - iso;
            const float d2 = p2 - iso;

            int n = 0;
            // Mirrors PlanarSlicer::tryEdge: skip the 'b'-on-plane case to
            // avoid double-counting at shared vertices.
            auto tryEdge = [&](const glm::vec3& a, const glm::vec3& b, float da, float db)
            {
                if (n >= 2) return;
                if ((da > 0.0f) != (db > 0.0f))
                {
                    if (std::abs(db) >= 1e-6f)
                    {
                        const float t = da / (da - db);
                        out[n++] = a + t * (b - a);
                    }
                }
                else if (std::abs(da) < 1e-6f)
                {
                    out[n++] = a;
                }
            };

            tryEdge(v0, v1, d0, d1);
            tryEdge(v1, v2, d1, d2);
            tryEdge(v2, v0, d2, d0);
            return (n == 2) ? 2 : 0;
        }

        /// Greedy ring chaining. Returns one Ring per closed contour,
        /// each entry tagged with the face_id of its source segment so the
        /// caller can look up per-face -grad Phi for tool orientation.
        std::vector<Ring> chainRings(const std::vector<RawEdge>& raw,
                                     std::size_t* dropped_out = nullptr)
        {
            std::vector<Ring> rings;
            if (raw.empty()) { if (dropped_out) *dropped_out = 0; return rings; }

            static constexpr float kSnapTol = 1e-4f;          // 0.1 µm
            static constexpr float kSnapSq  = kSnapTol * kSnapTol;

            auto distSq = [](const glm::vec3& p, const glm::vec3& q)
            {
                const glm::vec3 d = p - q;
                return d.x * d.x + d.y * d.y + d.z * d.z;
            };

            std::vector<bool> used(raw.size(), false);
            std::size_t consumed = 0;

            for (std::size_t seedIdx = 0; seedIdx < raw.size(); ++seedIdx)
            {
                if (used[seedIdx]) continue;

                Ring ring;
                ring.reserve(64);

                used[seedIdx] = true;
                ++consumed;
                ring.push_back({raw[seedIdx].a, raw[seedIdx].face_id});
                ring.push_back({raw[seedIdx].b, raw[seedIdx].face_id});

                glm::vec3 cursor = raw[seedIdx].b;
                while (true)
                {
                    std::size_t bestJ = SIZE_MAX;
                    float bestD       = std::numeric_limits<float>::max();
                    bool bestFlip     = false;

                    for (std::size_t j = 0; j < raw.size(); ++j)
                    {
                        if (used[j]) continue;
                        const float dA = distSq(cursor, raw[j].a);
                        const float dB = distSq(cursor, raw[j].b);
                        if (dA < bestD) { bestD = dA; bestJ = j; bestFlip = false; }
                        if (dB < bestD) { bestD = dB; bestJ = j; bestFlip = true;  }
                    }

                    if (bestJ == SIZE_MAX || bestD > kSnapSq) break;

                    used[bestJ] = true;
                    ++consumed;
                    cursor = bestFlip ? raw[bestJ].a : raw[bestJ].b;
                    ring.push_back({cursor, raw[bestJ].face_id});
                }

                if (ring.size() >= 3) rings.push_back(std::move(ring));
                else { /* degenerate seed — counted as consumed but dropped */ }
            }

            if (dropped_out) *dropped_out = raw.size() - consumed;
            return rings;
        }
    } // namespace

    std::vector<Layer> extractIsoLayers(const geometry::TriangleMesh& mesh,
                                        const ScalarField& field,
                                        const IsoExtractorConfig& cfg,
                                        std::uint32_t starting_layer_index)
    {
        std::vector<Layer> out;

        if (!mesh.isValid())
        {
            MEDUSA_WARN("IsoExtractor: invalid mesh");
            return out;
        }
        if (!field.valid || field.phi.size() != mesh.numVertices())
        {
            MEDUSA_WARN("IsoExtractor: scalar field invalid or size mismatch ({} vs {})",
                        field.phi.size(), mesh.numVertices());
            return out;
        }

        const int axis = std::clamp(cfg.up_axis, 0, 2);
        const float h_min = mesh.bounds.min[axis];
        const float h_max = mesh.bounds.max[axis];
        const float h_range = h_max - h_min;
        if (h_range < 1e-6f)
        {
            MEDUSA_WARN("IsoExtractor: zero mesh height along axis {}", axis);
            return out;
        }

        // ---- iso-value sampling --------------------------------------------------
        const float phi_lo = std::min(cfg.phi_start, cfg.phi_end);
        const float phi_hi = std::max(cfg.phi_start, cfg.phi_end);
        const float phi_span = phi_hi - phi_lo;
        if (phi_span < 1e-6f || cfg.layer_thickness <= 0.0f)
        {
            MEDUSA_WARN("IsoExtractor: invalid range/thickness "
                        "(phi=[{:.3f},{:.3f}], dh={:.3f})",
                        phi_lo, phi_hi, cfg.layer_thickness);
            return out;
        }

        // Global / mean dphi step (used as a reference for the adaptive
        // bounds and as a fallback when |grad Phi| is unavailable).
        const float dphi_uniform = cfg.layer_thickness / h_range
                                 * (field.phi_max - field.phi_min);

        // Adaptive bounds: clamp the per-step dphi between sane multiples of
        // the uniform reference. Without this a single near-zero gradient
        // sample (e.g. at a saddle point) can collapse the step to ~0 and
        // pin the loop to one phi for thousands of iterations.
        const float dphi_min = dphi_uniform * cfg.dphi_min_factor;
        const float dphi_max = dphi_uniform * cfg.dphi_max_factor;

        const bool use_adaptive = cfg.adaptive
                               && field.grad_norm_face.size() == mesh.numFaces()
                               && field.grad_mean > 1e-8f;

        if (cfg.adaptive && !use_adaptive)
        {
            MEDUSA_WARN("IsoExtractor: adaptive requested but |grad Phi| "
                        "missing/zero — falling back to uniform dphi={:.4f}",
                        dphi_uniform);
        }

        MEDUSA_INFO("IsoExtractor: walking phi=({:.3f},{:.3f}], "
                    "{} mode, target dh={:.3f} mm, dphi_uniform={:.4f}, "
                    "dphi_bounds=[{:.4f}, {:.4f}]",
                    phi_lo, phi_hi,
                    use_adaptive ? "ADAPTIVE" : "uniform",
                    cfg.layer_thickness, dphi_uniform, dphi_min, dphi_max);

        out.reserve(256);

        // Pre-extract face vertex indices once.
        const auto& V = mesh.vertices;
        const auto& Fs = mesh.faces;
        const auto& Phi = field.phi;
        const auto& Gnorm = field.grad_norm_face;

        // ---- perturbation lookup -------------------------------------------------
        // Sort vertex Phi values once so we can binary-search the nearest
        // value to a candidate iso. When the iso falls within
        //   eps_perturb_phi = 1e-5 * phi_span
        // of any vertex Phi the marching cubes case is degenerate
        // (iso-surface passes through a vertex -> rays of fragmented sub-rings).
        // We then bump the iso by `delta_perturb_phi` away from the offender.
        std::vector<float> phi_sorted(Phi.begin(), Phi.end());
        std::sort(phi_sorted.begin(), phi_sorted.end());
        const float eps_perturb_phi   = 1e-5f * phi_span;
        const float delta_perturb_phi = 1e-4f * phi_span;

        auto nearestVertexPhi = [&](float v) -> float
        {
            auto it = std::lower_bound(phi_sorted.begin(), phi_sorted.end(), v);
            float best = std::numeric_limits<float>::infinity();
            float best_signed = 0.0f;     // (v - vertex)
            auto consider = [&](std::vector<float>::const_iterator k)
            {
                const float d = v - *k;
                if (std::abs(d) < std::abs(best))
                {
                    best        = std::abs(d);
                    best_signed = d;
                }
            };
            if (it != phi_sorted.end())   consider(it);
            if (it != phi_sorted.begin()) consider(std::prev(it));
            (void)best;
            return best_signed; // signed distance v - phi_v_nearest
        };

        // ---- merge distance for fragmented-ring fixup ----------------------------
        // Components whose centroids are closer than this OR whose bboxes
        // overlap (expanded by this amount) are fused into a single logical
        // component. Catches degenerate slicing cases where a single ring
        // splits into N adjacent perimeter pieces (Cube example: 27 pieces
        // along y=34.375). Conservative enough to leave true topological
        // branches (e.g. double-pyramid pinch point) untouched.
        const float merge_distance = std::max(2.0f * cfg.layer_thickness,
                                              0.5f * cfg.layer_thickness);
        // (Both terms collapse to layer_thickness; kept as a max() for clarity
        // and so a future pass can swap one term for actual mesh edge length.)

        const glm::vec3 fallback_normal = (axis == 0) ? glm::vec3(1, 0, 0)
                                       : (axis == 1)  ? glm::vec3(0, 1, 0)
                                                      : glm::vec3(0, 0, 1);

        std::size_t empty_layers = 0;
        std::size_t total_raw     = 0;
        std::size_t total_rings   = 0;
        std::size_t total_dropped = 0;

        // Iso-step counter (one per phi-value visited that produced layers).
        // All ring-Layers of the same iso-step share Layer.index.
        int iso_step_count = 0;
        std::vector<int> comps_per_iso;
        comps_per_iso.reserve(256);

        // GAP DIAGNOSTICS: track y-centers of the first two iso-layers.
        float first_iso_y  = -1.0f;
        float second_iso_y = -1.0f;

        // Adaptive walk: start half a step above phi_lo, advance by
        // dphi_local = layer_thickness * |grad Phi|_local until phi_hi.
        // First step uses the LOCAL gradient at phi_start (computed by
        // sampling all faces that phi_start intersects) instead of the
        // global mean. This prevents overshooting when the field gradient
        // varies across the domain (e.g., compressed Phi near the base).
        float phi_cur = phi_lo;
        float g_seed  = use_adaptive ? field.grad_mean : 0.0f;

        // Sample local gradient at phi_start to compute a more accurate
        // initial step size.
        if (use_adaptive)
        {
            double g_sum_local = 0.0;
            int g_count_local = 0;
            for (std::size_t fi = 0; fi < Fs.size(); ++fi)
            {
                const auto& f = Fs[fi];
                const float p0 = Phi[f.x];
                const float p1 = Phi[f.y];
                const float p2 = Phi[f.z];
                const float pmin = std::min({p0, p1, p2});
                const float pmax = std::max({p0, p1, p2});
                // Check if phi_lo falls within this face's phi-range.
                if (phi_lo >= pmin && phi_lo <= pmax)
                {
                    g_sum_local += static_cast<double>(Gnorm[fi]);
                    ++g_count_local;
                }
            }
            if (g_count_local > 0)
            {
                g_seed = static_cast<float>(g_sum_local / g_count_local);
                MEDUSA_INFO("IsoExtractor: sampled LOCAL gradient at phi_start={:.6f}: "
                            "g_local={:.4f} (from {} faces), global g_mean={:.4f}",
                            phi_lo, g_seed, g_count_local, field.grad_mean);
            }
            else
            {
                MEDUSA_WARN("IsoExtractor: no faces found at phi_start={:.6f}, "
                            "falling back to global g_mean={:.4f}",
                            phi_lo, field.grad_mean);
            }
        }

        // Seed offset: advance by ONE full layer thickness from phi_start
        // so the first iso-layer sits exactly layer_thickness above the
        // last planar base layer.
        const float initial_dphi = use_adaptive
            ? std::clamp(cfg.layer_thickness * g_seed, dphi_min, dphi_max)
            : dphi_uniform;
        phi_cur += initial_dphi;

        MEDUSA_INFO("IsoExtractor: first iso at phi_cur={:.6f} "
                    "(phi_start={:.6f} + initial_dphi={:.6f}, g_seed={:.4f})",
                    phi_cur, phi_lo, initial_dphi, g_seed);

        int iter = 0;
        while (phi_cur < phi_hi && iso_step_count < cfg.max_layers)
        {
            ++iter;
            float iso = phi_cur;

            // ---- Fix A: iso-vs-vertex perturbation ----------------------------
            // If iso is too close to any vertex Phi, nudge it away by
            // delta_perturb_phi. Direction = sign of (iso - nearest vertex Phi)
            // so we push *away* from the offender. If iso sits exactly on a
            // vertex (signed_d == 0) we bias upward.
            const float signed_d   = nearestVertexPhi(iso);
            const float abs_d      = std::abs(signed_d);
            bool perturbed = false;
            float iso_before_perturb = iso;
            if (abs_d < eps_perturb_phi)
            {
                const float dir = (signed_d >= 0.0f) ? 1.0f : -1.0f;
                iso += dir * delta_perturb_phi;
                perturbed = true;
            }

            std::vector<RawEdge> raw;
            raw.reserve(Fs.size() / 8);

            // Accumulator for adaptive step: mean |grad Phi| across all
            // triangles that the iso passes through.
            double g_sum    = 0.0;
            int    g_count  = 0;

            glm::vec3 isect[2];
            for (std::size_t fi = 0; fi < Fs.size(); ++fi)
            {
                const auto& f = Fs[fi];
                const float p0 = Phi[f.x];
                const float p1 = Phi[f.y];
                const float p2 = Phi[f.z];

                const float pmin = std::min({p0, p1, p2});
                const float pmax = std::max({p0, p1, p2});
                if (iso < pmin || iso > pmax) continue;

                if (triangleIso(V[f.x], V[f.y], V[f.z], p0, p1, p2, iso, isect) == 2)
                {
                    raw.push_back({isect[0], isect[1], static_cast<std::uint32_t>(fi)});
                    if (use_adaptive)
                    {
                        g_sum += static_cast<double>(Gnorm[fi]);
                        ++g_count;
                    }
                }
            }

            std::size_t dropped_fragments = 0;
            auto rings = chainRings(raw, &dropped_fragments);

            // ---- Fix B: distance-based component merging -----------------------
            // Greedy union-find: components whose centroids are within
            // `merge_distance` OR whose bboxes overlap (expanded by a small
            // tolerance) are fused. Catches degenerate fragmentation where
            // an iso slice through a vertex shatters one logical ring into
            // many adjacent pieces. Conservative enough that genuinely
            // separated branches (centroid distance >> merge_distance) stay
            // intact.
            const std::size_t rings_before_merge = rings.size();
            if (rings.size() > 1)
            {
                struct RingInfo { glm::vec3 cmin, cmax; glm::dvec3 csum; std::size_t n; };
                std::vector<RingInfo> info(rings.size());
                for (std::size_t i = 0; i < rings.size(); ++i)
                {
                    glm::vec3 cmin( std::numeric_limits<float>::max());
                    glm::vec3 cmax(-std::numeric_limits<float>::max());
                    glm::dvec3 csum(0.0);
                    for (const auto& rp : rings[i])
                    {
                        cmin = glm::min(cmin, rp.p);
                        cmax = glm::max(cmax, rp.p);
                        csum += glm::dvec3(rp.p);
                    }
                    info[i] = { cmin, cmax, csum, rings[i].size() };
                }

                // Union-Find
                std::vector<std::size_t> parent(rings.size());
                std::iota(parent.begin(), parent.end(), std::size_t{0});
                std::function<std::size_t(std::size_t)> find_ =
                    [&](std::size_t x) -> std::size_t
                {
                    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
                    return x;
                };
                auto unite = [&](std::size_t a, std::size_t b)
                {
                    a = find_(a); b = find_(b);
                    if (a != b) parent[b] = a;
                };

                const float md  = merge_distance;
                const float md2 = md * md;
                const glm::vec3 exp(md);

                for (std::size_t i = 0; i < rings.size(); ++i)
                {
                    const glm::vec3 ci_centroid =
                        glm::vec3(info[i].csum / static_cast<double>(info[i].n));
                    for (std::size_t j = i + 1; j < rings.size(); ++j)
                    {
                        const glm::vec3 cj_centroid =
                            glm::vec3(info[j].csum / static_cast<double>(info[j].n));
                        const glm::vec3 d = ci_centroid - cj_centroid;
                        const float dd = d.x*d.x + d.y*d.y + d.z*d.z;

                        // bbox overlap test (expanded)
                        const glm::vec3 aimin = info[i].cmin - exp;
                        const glm::vec3 aimax = info[i].cmax + exp;
                        const glm::vec3 ajmin = info[j].cmin - exp;
                        const glm::vec3 ajmax = info[j].cmax + exp;
                        const bool bbox_overlap =
                            aimin.x <= ajmax.x && aimax.x >= ajmin.x &&
                            aimin.y <= ajmax.y && aimax.y >= ajmin.y &&
                            aimin.z <= ajmax.z && aimax.z >= ajmin.z;

                        if (dd < md2 || bbox_overlap) unite(i, j);
                    }
                }

                // Concatenate group members.
                std::vector<Ring> grouped;
                std::vector<std::size_t> group_of(rings.size(), SIZE_MAX);
                for (std::size_t i = 0; i < rings.size(); ++i)
                {
                    const std::size_t r = find_(i);
                    if (group_of[r] == SIZE_MAX)
                    {
                        group_of[r] = grouped.size();
                        grouped.emplace_back();
                    }
                    auto& dst = grouped[group_of[r]];
                    dst.insert(dst.end(), rings[i].begin(), rings[i].end());
                }
                rings = std::move(grouped);

                if (rings.size() != rings_before_merge)
                {
                    MEDUSA_INFO("IsoExtractor: iso[{:4d}] merged {} components -> {} "
                                "(merge_dist={:.3f} mm)",
                                iter, rings_before_merge, rings.size(), merge_distance);
                }
            }

            if (perturbed)
            {
                MEDUSA_INFO("IsoExtractor: iso[{:4d}] phi={:.6f} -> {:.6f} "
                            "(perturbed, was {:.2e} from vertex)",
                            iter, iso_before_perturb, iso, abs_d);
            }

            // Per-iso diagnostic at DEBUG: phi, raw segs, rings, fragments,
            // local gradient (NaN if no faces contributed).
            const float g_local = (g_count > 0)
                ? static_cast<float>(g_sum / g_count)
                : std::numeric_limits<float>::quiet_NaN();
            MEDUSA_DEBUG("IsoExtractor: iso[{:4d}] phi={:.4f} raw={} rings={} "
                         "drop={} g_local={:.4f}",
                         iter, iso, raw.size(), rings.size(),
                         dropped_fragments, g_local);

            // -- Anomaly diagnostics ------------------------------------------------
            // When a single iso-value produces multiple connected components,
            // dump per-component geometry so we can tell apart legitimate
            // branching from degenerate/numerical fragmentation. We also
            // measure how close iso is to the nearest vertex-Phi value: a
            // very small distance hints at iso-through-vertex degeneracy.
            if (rings.size() > 1)
            {
                float min_dphi_vertex = std::numeric_limits<float>::max();
                for (std::size_t vi = 0; vi < Phi.size(); ++vi)
                {
                    const float d = std::abs(Phi[vi] - iso);
                    if (d < min_dphi_vertex) min_dphi_vertex = d;
                }
                MEDUSA_INFO("IsoExtractor: ANOMALY iso[{:4d}] phi={:.6f} "
                            "components={} min|phi_v - iso|={:.2e} g_local={:.4f}",
                            iter, iso, rings.size(), min_dphi_vertex, g_local);

                for (std::size_t ri = 0; ri < rings.size(); ++ri)
                {
                    const auto& r = rings[ri];
                    glm::vec3 cmin( std::numeric_limits<float>::max());
                    glm::vec3 cmax(-std::numeric_limits<float>::max());
                    glm::dvec3 csum(0.0);
                    for (const auto& rp : r)
                    {
                        cmin = glm::min(cmin, rp.p);
                        cmax = glm::max(cmax, rp.p);
                        csum += glm::dvec3(rp.p);
                    }
                    const glm::dvec3 centroid = csum / static_cast<double>(r.size());
                    const glm::vec3  bbox     = cmax - cmin;
                    MEDUSA_INFO("    comp[{:2d}]: verts={:4d} centroid=({:7.3f},{:7.3f},{:7.3f}) "
                                "bbox=({:6.3f},{:6.3f},{:6.3f})",
                                ri, r.size(),
                                centroid.x, centroid.y, centroid.z,
                                bbox.x, bbox.y, bbox.z);
                }
            }

            total_raw      += raw.size();
            total_rings    += rings.size();
            total_dropped  += dropped_fragments;

            if (rings.empty())
            {
                ++empty_layers;
                // Even on empty layers we still need to advance, otherwise an
                // iso outside the mesh (numerical edge case near phi_hi) loops
                // forever. Use the previous step.
            }
            else
            {
                // Emit ONE Layer per ring (= one connected component of the
                // iso-surface). All Layers belonging to the same iso-step
                // share the same Layer.index so the BranchTracker can group
                // them. Branch-id is filled in later by the BranchTracker.
                const std::uint32_t iso_step = static_cast<std::uint32_t>(iso_step_count);
                ++iso_step_count;

                std::size_t comps_this_iso = 0;
                const bool have_grad_vec =
                    (field.grad_face.size() == mesh.numFaces());

                auto orientFromFace = [&](std::uint32_t face_id) -> glm::vec3
                {
                    if (!have_grad_vec || face_id >= field.grad_face.size())
                        return fallback_normal;
                    const glm::vec3 g = field.grad_face[face_id];
                    const float l = glm::length(g);
                    if (l < 1e-8f) return fallback_normal;
                    // z_tool = +normalize(grad Phi): tool z-axis is the approach
                    // vector, pointing AWAY from the part in the direction the
                    // robot tool extends. This matches the planar slicer convention
                    // (+up_axis) and the standard MoveIt/ROS2 tool-frame definition.
                    return g / l;
                };

                // GAP DIAGNOSTICS: track first two iso-layers.
                const bool is_first_iso  = (iso_step_count == 1);
                const bool is_second_iso = (iso_step_count == 2);

                for (const auto& r : rings)
                {
                    if (r.size() < 2) continue;

                    Layer layer;
                    layer.index     = starting_layer_index + iso_step;
                    layer.thickness = cfg.layer_thickness;
                    layer.up_axis   = axis;
                    layer.branch_id = 0; // BranchTracker will overwrite.

                    layer.contour_points.reserve((r.size() - 1) * 2);
                    layer.contour_normals.reserve((r.size() - 1) * 2);
                    layer.contour_face_ids.reserve((r.size() - 1) * 2);

                    double y_sum = 0.0;
                    std::size_t y_count = 0;

                    for (std::size_t i = 0; i + 1 < r.size(); ++i)
                    {
                        // PlanarSlicer-compatible edge-pair format
                        // (see iso_extractor diagnostics for the rationale).
                        layer.contour_points.push_back(r[i].p);
                        layer.contour_points.push_back(r[i + 1].p);
                        // Per-point tool orientation = -normalize(grad Phi)
                        // sampled at the source face. Falls back to the
                        // up-axis if the gradient is degenerate (planar
                        // patches, vanishing |grad Phi|).
                        layer.contour_normals.push_back(orientFromFace(r[i].face_id));
                        layer.contour_normals.push_back(orientFromFace(r[i + 1].face_id));
                        // Propagate mesh face IDs for infill ray-cast filtering.
                        layer.contour_face_ids.push_back(r[i].face_id);
                        layer.contour_face_ids.push_back(r[i + 1].face_id);

                        // Accumulate y-coords for gap diagnostics.
                        if (is_first_iso || is_second_iso)
                        {
                            y_sum += static_cast<double>(r[i].p[axis]);
                            y_sum += static_cast<double>(r[i + 1].p[axis]);
                            y_count += 2;
                        }
                    }
                    layer.contour_count = layer.contour_points.size();

                    // Store mean_y for the first two iso-layers.
                    if (y_count > 0)
                    {
                        const float mean_y = static_cast<float>(y_sum / y_count);
                        if (is_first_iso && first_iso_y < 0.0f)
                        {
                            first_iso_y = mean_y;
                            MEDUSA_INFO("IsoExtractor GAP-CHECK: first_iso phi={:.6f}, mean_y={:.3f}",
                                        iso, mean_y);
                        }
                        else if (is_second_iso && second_iso_y < 0.0f)
                        {
                            second_iso_y = mean_y;
                            MEDUSA_INFO("IsoExtractor GAP-CHECK: second_iso phi={:.6f}, mean_y={:.3f}",
                                        iso, mean_y);
                        }
                    }

                    out.push_back(std::move(layer));
                    ++comps_this_iso;
                }
                comps_per_iso.push_back(static_cast<int>(comps_this_iso));
            }

            // Compute next dphi. If no triangles contributed (iso completely
            // outside the field), reuse the previous seed; clamp to bounds.
            float dphi_next = dphi_uniform;
            if (use_adaptive)
            {
                const float g_use = (g_count > 0) ? static_cast<float>(g_sum / g_count) : g_seed;
                dphi_next = std::clamp(cfg.layer_thickness * g_use, dphi_min, dphi_max);
                g_seed = g_use; // remember for the next iteration
            }
            phi_cur += dphi_next;
        }

        if (empty_layers > 0)
        {
            MEDUSA_WARN("IsoExtractor: {} of {} iso-iterations produced empty contours",
                        empty_layers, iter);
        }

        MEDUSA_INFO("IsoExtractor: emitted {} ring-layers across {} iso-steps "
                    "in {} iterations (total raw_segments={}, rings={}, dropped_fragments={})",
                    out.size(), iso_step_count, iter, total_raw, total_rings, total_dropped);

        // Component-count distribution: how many iso-steps had 1, 2, 3, ...
        // connected components. Splits/merges show up here as steps with > 1.
        // The BranchTracker will then label them across iso-steps.
        if (!comps_per_iso.empty())
        {
            int max_comps = 0;
            for (int c : comps_per_iso) max_comps = std::max(max_comps, c);
            std::vector<int> hist(static_cast<std::size_t>(max_comps + 1), 0);
            for (int c : comps_per_iso) ++hist[static_cast<std::size_t>(c)];
            std::string row;
            for (int n = 1; n <= max_comps; ++n)
            {
                row += std::to_string(n) + "comp=" + std::to_string(hist[n]);
                if (n < max_comps) row += " ";
            }
            MEDUSA_INFO("IsoExtractor: component-count distribution per iso-step: {}", row);
        }

        // ---- GAP DIAGNOSTICS: report iso-layer spacing ----
        if (first_iso_y > 0.0f && second_iso_y > 0.0f)
        {
            const float gap_iso = second_iso_y - first_iso_y;
            MEDUSA_INFO("IsoExtractor GAP-CHECK: iso_spacing={:.2f} mm (target={:.2f}, delta={:+.2f})",
                        gap_iso, cfg.layer_thickness, gap_iso - cfg.layer_thickness);
        }

        return out;
    }
} // namespace slicing::crystal
