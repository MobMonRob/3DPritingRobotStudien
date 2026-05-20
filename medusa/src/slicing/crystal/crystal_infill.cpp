/**
 * @file crystal_infill.cpp
 * @brief Implementation of curvilinear rectilinear infill (PCA basis +
 *        mesh ray-cast lift + connected zigzag).
 */

#include "crystal_infill.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <glm/geometric.hpp>

#include "logger.h"
#include "triangle_mesh.h"

namespace slicing::crystal
{
    namespace
    {
        /// Robust tangent basis from a unit normal: returns (u_hat, v_hat)
        /// spanning the plane with normal n_hat.
        void seedTangentBasis(const glm::vec3& n_hat,
                              glm::vec3& u_hat, glm::vec3& v_hat)
        {
            const glm::vec3 ax = glm::abs(n_hat);
            glm::vec3 seed;
            if (ax.x <= ax.y && ax.x <= ax.z)        seed = glm::vec3(1, 0, 0);
            else if (ax.y <= ax.x && ax.y <= ax.z)   seed = glm::vec3(0, 1, 0);
            else                                     seed = glm::vec3(0, 0, 1);

            u_hat = seed - glm::dot(seed, n_hat) * n_hat;
            const float lu = glm::length(u_hat);
            u_hat = (lu > 1e-8f) ? (u_hat / lu) : glm::vec3(1, 0, 0);
            v_hat = glm::normalize(glm::cross(n_hat, u_hat));
        }

        /// Rotate an in-plane basis (u_hat, v_hat) by `angle` radians around
        /// n_hat (right-hand rule). Used after PCA to align u with the
        /// principal direction of the projected contour.
        void rotateBasis(float angle, glm::vec3& u_hat, glm::vec3& v_hat)
        {
            const float c = std::cos(angle);
            const float s = std::sin(angle);
            const glm::vec3 u_new =  c * u_hat + s * v_hat;
            const glm::vec3 v_new = -s * u_hat + c * v_hat;
            u_hat = u_new;
            v_hat = v_new;
        }

        enum class LiftDrop { none, no_hit, out_of_distance };

        struct LiftResult
        {
            std::optional<glm::vec3> point;
            LiftDrop drop{LiftDrop::none};
        };
    } // namespace

    void generateNonPlanarInfill(std::vector<Layer>& layers,
                                 float spacing,
                                 const geometry::TriangleMesh& mesh)
    {
        if (spacing <= 0.0f)
        {
            MEDUSA_WARN("CrystalInfill: invalid spacing {:.3f} mm - skipping", spacing);
            return;
        }
        if (!mesh.isValid())
        {
            MEDUSA_WARN("CrystalInfill: mesh invalid - skipping");
            return;
        }

        std::size_t total_segments = 0;
        std::size_t total_dropped  = 0;
        std::size_t layers_with_infill = 0;
        std::size_t layers_skipped = 0;

        for (auto& layer : layers)
        {
            const std::size_t contour_size = layer.contour_points.size();
            if (contour_size < 2 || layer.contour_normals.size() != contour_size)
            {
                ++layers_skipped;
                continue;
            }

            // Boundary edges from existing (paired) contour points.
            struct Edge { glm::vec3 a, b; };
            std::vector<Edge> edges;
            edges.reserve(contour_size / 2);
            for (std::size_t i = 0; i + 1 < contour_size; i += 2)
            {
                edges.push_back({layer.contour_points[i], layer.contour_points[i + 1]});
            }
            if (edges.empty()) { ++layers_skipped; continue; }

            // ---- Centroid + average orientation ----
            glm::dvec3 csum(0.0);
            glm::dvec3 nsum(0.0);
            for (std::size_t i = 0; i < contour_size; ++i)
            {
                csum += glm::dvec3(layer.contour_points[i]);
                nsum += glm::dvec3(layer.contour_normals[i]);
            }
            const glm::vec3 centroid = glm::vec3(csum / static_cast<double>(contour_size));
            glm::vec3 n_hat = glm::vec3(nsum / static_cast<double>(contour_size));
            const float nl = glm::length(n_hat);
            n_hat = (nl > 1e-6f) ? (n_hat / nl) : glm::vec3(0, 1, 0);

            // ---- Seed tangent basis ----
            glm::vec3 u_hat, v_hat;
            seedTangentBasis(n_hat, u_hat, v_hat);

            // ---- PCA on projected contour to align u with principal axis ----
            // 2x2 covariance in the seed (u, v) basis. Closed-form eigen.
            double cuu = 0.0, cuv = 0.0, cvv = 0.0;
            // Also track max |d·n_hat| to quantify out-of-plane spread.
            double max_n_component = 0.0;
            double sum_n_sq = 0.0;
            for (const auto& e : edges)
            {
                for (const auto& p : { e.a, e.b })
                {
                    const glm::vec3 d = p - centroid;
                    const double pu = glm::dot(d, u_hat);
                    const double pv = glm::dot(d, v_hat);
                    const double pn = glm::dot(d, n_hat);
                    cuu += pu * pu;
                    cuv += pu * pv;
                    cvv += pv * pv;
                    sum_n_sq += pn * pn;
                    max_n_component = std::max(max_n_component, std::abs(pn));
                }
            }
            const std::size_t pca_n_pts = edges.size() * 2;
            const double tr   = cuu + cvv;
            const double det  = cuu * cvv - cuv * cuv;
            const double disc = std::max(0.0, tr * tr * 0.25 - det);
            const double lam1 = tr * 0.5 + std::sqrt(disc); // larger eigenvalue

            // -- PCA diagnostic: log raw data for suspicious components --
            {
                const double lam2_raw = tr * 0.5 - std::sqrt(disc);
                const double ratio_raw = (lam1 > 1e-12) ? (lam2_raw / lam1) : 0.0;
                const double var_n = (pca_n_pts > 0) ? (sum_n_sq / pca_n_pts) : 0.0;

                MEDUSA_DEBUG("CrystalInfill PCA layer {}: n_hat=({:.3f},{:.3f},{:.3f}), "
                             "n_len_pre_norm={:.4f}, seed_u=({:.3f},{:.3f},{:.3f}), "
                             "seed_v=({:.3f},{:.3f},{:.3f})",
                             layer.index,
                             n_hat.x, n_hat.y, n_hat.z, nl,
                             u_hat.x, u_hat.y, u_hat.z,
                             v_hat.x, v_hat.y, v_hat.z);
                MEDUSA_DEBUG("  cov=[{:.1f},{:.1f};{:.1f},{:.1f}], pca_pts={}, "
                             "lam=[{:.1f},{:.1f}], ratio={:.4f}, "
                             "max_n_comp={:.2f}mm, rms_n={:.2f}mm",
                             cuu, cuv, cuv, cvv, pca_n_pts,
                             lam1, lam2_raw, ratio_raw,
                             max_n_component, std::sqrt(var_n));

                // Sample first 3 edge-midpoints for quick visual check.
                for (std::size_t si = 0; si < std::min(edges.size(), std::size_t(3)); ++si)
                {
                    const glm::vec3 mid = (edges[si].a + edges[si].b) * 0.5f;
                    const glm::vec3 dd = mid - centroid;
                    MEDUSA_DEBUG("  sample[{}]: 3D=({:.2f},{:.2f},{:.2f}), "
                                 "uv=({:.2f},{:.2f}), n_comp={:.2f}",
                                 si, mid.x, mid.y, mid.z,
                                 glm::dot(dd, u_hat), glm::dot(dd, v_hat),
                                 glm::dot(dd, n_hat));
                }
            }
            double ex = lam1 - cvv;
            double ey = cuv;
            if (std::abs(ex) + std::abs(ey) < 1e-12)
            {
                ex = cuv;
                ey = lam1 - cuu;
            }

            const double lam2_val = tr * 0.5 - std::sqrt(disc);
            const double pca_ratio_val = (lam1 > 1e-12) ? (lam2_val / lam1) : 0.0;

            if (pca_ratio_val > 0.7)
            {
                // Truly isotropic contour (e.g. square cross-section).
                // PCA principal axis is unstable → fall back to world-X
                // projected into the tangent plane for a stable, axis-
                // aligned basis.
                const glm::vec3 world_x(1.0f, 0.0f, 0.0f);
                glm::vec3 u_proj = world_x - glm::dot(world_x, n_hat) * n_hat;
                float lu = glm::length(u_proj);
                if (lu < 1e-6f)
                {
                    // n_hat ≈ ±X → use world-Z instead.
                    const glm::vec3 world_z(0.0f, 0.0f, 1.0f);
                    u_proj = world_z - glm::dot(world_z, n_hat) * n_hat;
                    lu = glm::length(u_proj);
                }
                u_hat = u_proj / lu;
                v_hat = glm::normalize(glm::cross(n_hat, u_hat));

                MEDUSA_DEBUG("CrystalInfill: layer {} PCA isotropic fallback: "
                             "using world-aligned axes (ratio={:.3f})",
                             layer.index, pca_ratio_val);
            }
            else
            {
                float pca_angle = 0.0f;
                if (std::abs(ex) + std::abs(ey) > 1e-12)
                {
                    pca_angle = static_cast<float>(std::atan2(ey, ex));
                }
                rotateBasis(pca_angle, u_hat, v_hat);
            }

            // Per-layer 90° cross-hatch: only for flat layers (Stem).
            // Curved layers use PCA MINOR axis as u (sweep along short
            // direction → many lines stacked along the long direction).
            constexpr float kFlatCrosshatch = 0.5f;
            const bool is_curved = (max_n_component >= kFlatCrosshatch);
            if (is_curved)
            {
                // Rotate 90° so u becomes PCA minor axis.
                rotateBasis(static_cast<float>(M_PI) * 0.5f, u_hat, v_hat);
            }
            else if ((layer.index & 1u) != 0u)
            {
                // Flat: alternate cross-hatch.
                rotateBasis(static_cast<float>(M_PI) * 0.5f, u_hat, v_hat);
            }

            // ---- Project contour to a continuous 2D ring polygon ----
            // The contour is stored as edge-pairs [p0,p1, p2,p3, ...] where
            // consecutive endpoints are shared (p1≈p2, p3≈p4). Build one
            // continuous polygon by taking the first point of each pair.
            std::vector<glm::vec2> ring2;
            ring2.reserve(edges.size() + 1);

            auto project = [&](const glm::vec3& p) -> glm::vec2
            {
                const glm::vec3 d = p - centroid;
                return { glm::dot(d, u_hat), glm::dot(d, v_hat) };
            };

            float u_min =  std::numeric_limits<float>::max();
            float u_max = -std::numeric_limits<float>::max();
            float v_min =  std::numeric_limits<float>::max();
            float v_max = -std::numeric_limits<float>::max();

            for (const auto& e : edges)
            {
                glm::vec2 pa = project(e.a);
                ring2.push_back(pa);
                u_min = std::min(u_min, pa.x);
                u_max = std::max(u_max, pa.x);
                v_min = std::min(v_min, pa.y);
                v_max = std::max(v_max, pa.y);
            }
            // Close the ring: add the endpoint of the last edge.
            if (!edges.empty())
            {
                glm::vec2 last = project(edges.back().b);
                ring2.push_back(last);
                u_min = std::min(u_min, last.x);
                u_max = std::max(u_max, last.x);
                v_min = std::min(v_min, last.y);
                v_max = std::max(v_max, last.y);
            }

            // Mark perimeter / infill boundary BEFORE appending infill.
            layer.contour_count = layer.contour_points.size();

            if ((v_max - v_min) < 1e-4f || (u_max - u_min) < 1e-4f)
            {
                ++layers_skipped;
                continue;
            }

            // ---- IDW lift: project ring vertices to uv, then use Inverse-
            //      Distance-Weighting to reconstruct 3D positions from the
            //      known iso-line vertices. Works for all cases (flat, curved,
            //      shell) without ray-casting. ----
            constexpr int kIDW_k = 6; // number of nearest neighbors
            constexpr float kIDW_eps = 0.001f; // avoid div-by-zero

            // Build list of unique ring vertices with their uv projections.
            // contour_points is stored as edge-pairs [a0,b0, a1,b1, ...].
            // Take first point of each pair (= unique ring vertex chain).
            struct RingVtx { glm::vec2 uv; glm::vec3 pos; };
            std::vector<RingVtx> ring_vtx;
            ring_vtx.reserve(contour_size / 2 + 1);
            for (std::size_t i = 0; i < contour_size; i += 2)
            {
                const glm::vec3& p = layer.contour_points[i];
                const glm::vec3 d = p - centroid;
                ring_vtx.push_back({
                    glm::vec2(glm::dot(d, u_hat), glm::dot(d, v_hat)),
                    p
                });
            }
            // Also add last edge endpoint to close ring.
            if (contour_size >= 2)
            {
                const glm::vec3& p = layer.contour_points[contour_size - 1];
                const glm::vec3 d = p - centroid;
                ring_vtx.push_back({
                    glm::vec2(glm::dot(d, u_hat), glm::dot(d, v_hat)),
                    p
                });
            }

            if (ring_vtx.empty())
            {
                ++layers_skipped;
                continue;
            }

            const int k_actual = std::min(kIDW_k, static_cast<int>(ring_vtx.size()));

            MEDUSA_DEBUG("CrystalInfill: layer {} IDW-lift (k={}, ring_verts={})",
                         layer.index, k_actual, ring_vtx.size());

            auto liftPoint = [&](float u, float v) -> LiftResult
            {
                const glm::vec2 pt(u, v);

                // Find k nearest ring vertices by 2D distance.
                // For moderate ring sizes (<500), partial sort is fast enough.
                // Store (dist2, index) pairs.
                thread_local std::vector<std::pair<float, std::size_t>> dists;
                dists.clear();
                dists.reserve(ring_vtx.size());
                for (std::size_t i = 0; i < ring_vtx.size(); ++i)
                {
                    const glm::vec2 diff = pt - ring_vtx[i].uv;
                    dists.push_back({ glm::dot(diff, diff), i });
                }
                std::partial_sort(dists.begin(),
                                  dists.begin() + k_actual,
                                  dists.end(),
                                  [](const auto& a, const auto& b) { return a.first < b.first; });

                // Inverse-distance-weighted interpolation.
                glm::dvec3 weighted_pos(0.0);
                double total_weight = 0.0;
                for (int ki = 0; ki < k_actual; ++ki)
                {
                    const float d = std::sqrt(dists[ki].first) + kIDW_eps;
                    const double w = 1.0 / (static_cast<double>(d) * d);
                    weighted_pos += w * glm::dvec3(ring_vtx[dists[ki].second].pos);
                    total_weight += w;
                }
                const glm::vec3 p3d = glm::vec3(weighted_pos / total_weight);
                return { p3d, LiftDrop::none };
            };

            // ---- Scanline sweep against continuous ring polygon ---------------
            // Even-odd intersection: for each scanline at fixed v, find all
            // crossings with the ring polygon edges, sort, pair as in/out.
            const std::size_t ring_n = ring2.size();

            std::size_t segs_this_layer = 0;
            std::size_t dropped_this_layer = 0;
            std::size_t drops_no_hit = 0;
            std::size_t drops_out_of_dist = 0;
            std::size_t points_total = 0;

            struct DroppedInfo { float u; float v; LiftDrop reason; };
            std::vector<DroppedInfo> first_drops; // keep first 3

            struct LiftedPair { glm::vec3 a, b; };
            std::vector<std::vector<LiftedPair>> per_scan;
            std::size_t infill_lines_generated = 0;
            per_scan.reserve(static_cast<std::size_t>((v_max - v_min) / spacing) + 2);

            for (float v = v_min + spacing * 0.5f; v < v_max; v += spacing)
            {
                // Intersect scanline y=v with every edge of the ring polygon.
                std::vector<float> hits;
                hits.reserve(8);
                for (std::size_t ri = 0; ri + 1 < ring_n; ++ri)
                {
                    const float va = ring2[ri].y;
                    const float vb = ring2[ri + 1].y;
                    if ((va < v) != (vb < v))
                    {
                        const float t = (v - va) / (vb - va);
                        hits.push_back(ring2[ri].x + t * (ring2[ri + 1].x - ring2[ri].x));
                    }
                }
                if (hits.size() < 2) continue;
                std::sort(hits.begin(), hits.end());

                std::vector<LiftedPair> scan;
                for (std::size_t i = 0; i + 1 < hits.size(); i += 2)
                {
                    ++infill_lines_generated;
                    points_total += 2;
                    auto ra = liftPoint(hits[i],     v);
                    auto rb = liftPoint(hits[i + 1], v);
                    if (ra.point && rb.point)
                    {
                        scan.push_back({*ra.point, *rb.point});
                    }
                    else
                    {
                        ++dropped_this_layer;
                        auto trackDrop = [&](float du, float dv, LiftDrop reason)
                        {
                            if (reason == LiftDrop::no_hit) ++drops_no_hit;
                            else if (reason == LiftDrop::out_of_distance) ++drops_out_of_dist;
                            if (first_drops.size() < 3)
                                first_drops.push_back({du, dv, reason});
                        };
                        if (!ra.point) trackDrop(hits[i],     v, ra.drop);
                        if (!rb.point) trackDrop(hits[i + 1], v, rb.drop);
                    }
                }
                if (!scan.empty()) per_scan.push_back(std::move(scan));
            }

            // ---- Emit zigzag (snake) ------------------------------------------
            // Reverse every other scanline so consecutive endpoints can be
            // chained by a print-only connector segment, eliminating the
            // travel-move avalanche between scanlines (Bug 3 fix).
            bool flip = false;
            std::optional<glm::vec3> prev_end;
            for (auto& scan : per_scan)
            {
                if (flip) std::reverse(scan.begin(), scan.end());
                for (auto& seg : scan)
                {
                    const glm::vec3 a = flip ? seg.b : seg.a;
                    const glm::vec3 b = flip ? seg.a : seg.b;
                    if (prev_end)
                    {
                        // Connector (still classified as infill).
                        layer.contour_points.push_back(*prev_end);
                        layer.contour_points.push_back(a);
                        layer.contour_normals.push_back(n_hat);
                        layer.contour_normals.push_back(n_hat);
                        ++segs_this_layer;
                    }
                    layer.contour_points.push_back(a);
                    layer.contour_points.push_back(b);
                    layer.contour_normals.push_back(n_hat);
                    layer.contour_normals.push_back(n_hat);
                    ++segs_this_layer;
                    prev_end = b;
                }
                flip = !flip;
            }

            total_segments += segs_this_layer;
            total_dropped  += dropped_this_layer;
            if (segs_this_layer > 0) ++layers_with_infill;

            // ---- Comprehensive per-component diagnostics ----
            const float bbox_w = u_max - u_min;
            const float bbox_h = v_max - v_min;

            MEDUSA_DEBUG("CrystalInfill: layer {} comp {}: ring_verts={}, "
                         "pca_ev=[{:.2f},{:.2f}], pca_ratio={:.3f}, "
                         "u_dir=({:.3f},{:.3f},{:.3f}), lines={}, pts={}, "
                         "dropped={} (no_hit:{}, ood:{}), bbox_uv=({:.1f},{:.1f})mm",
                         layer.index, layer.branch_id,
                         contour_size,
                         lam1, lam2_val, pca_ratio_val,
                         u_hat.x, u_hat.y, u_hat.z,
                         infill_lines_generated,
                         points_total,
                         dropped_this_layer, drops_no_hit, drops_out_of_dist,
                         bbox_w, bbox_h);

            // Suspicious component warnings
            const bool high_drop_rate = (points_total > 0) &&
                (dropped_this_layer * 100 / std::max(points_total, std::size_t(1)) > 30);
            const bool no_lines = (infill_lines_generated == 0 && contour_size >= 4);
            const bool pca_degen = (pca_ratio_val > 0.85);

            if (high_drop_rate || no_lines || pca_degen)
            {
                std::string reasons;
                if (high_drop_rate)
                    reasons += fmt::format("drop_rate={}% ", dropped_this_layer * 100 / points_total);
                if (no_lines)
                    reasons += "zero_infill_lines ";
                if (pca_degen)
                    reasons += fmt::format("pca_nearly_isotropic(ratio={:.3f}) ", pca_ratio_val);

                MEDUSA_WARN("CrystalInfill: layer {} comp {} WARNING: {}",
                            layer.index, layer.branch_id, reasons);

                for (std::size_t di = 0; di < first_drops.size(); ++di)
                {
                    const auto& dd = first_drops[di];
                    const char* reason_str = (dd.reason == LiftDrop::no_hit)
                        ? "no_hit" : "out_of_distance";
                    MEDUSA_WARN("  dropped[{}]: uv=({:.2f},{:.2f}) reason={}",
                                di, dd.u, dd.v, reason_str);
                }
            }
        }

        MEDUSA_INFO("CrystalInfill: {} segments across {} layers "
                    "({} skipped, {} points dropped, spacing={:.3f} mm)",
                    total_segments, layers_with_infill, layers_skipped,
                    total_dropped, spacing);
    }
} // namespace slicing::crystal
