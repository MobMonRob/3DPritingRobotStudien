/**
 * @file branch_tracker.cpp
 * @brief Centroid-based branch tracking across iso-steps.
 */

#include "branch_tracker.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include <glm/glm.hpp>

#include "logger.h"

namespace slicing::crystal
{
    namespace
    {
        struct CompInfo
        {
            std::size_t   layer_idx;     ///< Index into the layers vector.
            glm::vec3     centroid;
            std::uint32_t branch_id{0};
        };

        /// Centroid of a Layer. Layers store contour points as edge pairs
        /// (p0,p1,p1,p2,...) so we average over all points; duplicates do
        /// not bias the centroid since each interior point appears exactly
        /// twice and endpoints once each.
        glm::vec3 centroidOf(const Layer& l)
        {
            if (l.contour_points.empty()) return glm::vec3(0.0f);
            glm::dvec3 acc(0.0);
            for (const auto& p : l.contour_points)
            {
                acc += glm::dvec3(p);
            }
            return glm::vec3(acc / static_cast<double>(l.contour_points.size()));
        }
    } // namespace

    BranchTrackingStats assignBranchIds(std::vector<Layer>& layers,
                                        const BranchTrackerConfig& cfg)
    {
        BranchTrackingStats stats;
        if (layers.empty()) return stats;

        // Group iso-step layers by Layer.index. Planar base layers (index <
        // first_iso_layer_index) keep branch_id = 0 and are skipped here.
        std::vector<std::vector<std::size_t>> groups;     // per iso-step: layer indices
        std::uint32_t cur_index = std::numeric_limits<std::uint32_t>::max();
        for (std::size_t i = 0; i < layers.size(); ++i)
        {
            if (layers[i].index < cfg.first_iso_layer_index)
            {
                layers[i].branch_id = 0; // planar base
                continue;
            }
            if (layers[i].index != cur_index)
            {
                groups.emplace_back();
                cur_index = layers[i].index;
            }
            groups.back().push_back(i);
        }

        if (groups.empty())
        {
            MEDUSA_INFO("BranchTracker: no iso-layers to track");
            return stats;
        }

        // Per-iso layer thickness (assumed uniform per iso-step). Use the
        // first iso layer's thickness as the reference.
        const float dh = layers[groups.front().front()].thickness;
        const float drift_thresh = std::max(cfg.min_drift_mm,
                                            cfg.max_drift_factor * dh);

        MEDUSA_INFO("BranchTracker: tracking {} iso-steps, drift threshold = {:.3f} mm "
                    "({}x layer thickness {:.3f})",
                    groups.size(), drift_thresh, cfg.max_drift_factor, dh);

        // ---- iso-step 0: every component is a birth ----
        std::vector<CompInfo> prev;
        std::uint32_t next_branch = 1; // 0 reserved for planar base
        for (std::size_t li : groups.front())
        {
            CompInfo c;
            c.layer_idx = li;
            c.centroid  = centroidOf(layers[li]);
            c.branch_id = next_branch++;
            layers[li].branch_id = c.branch_id;
            prev.push_back(c);
            ++stats.births;
        }
        stats.total_components += static_cast<std::uint32_t>(prev.size());
        MEDUSA_DEBUG("BranchTracker: iso[0] components={} branches={}",
                     prev.size(), prev.size());

        // ---- iso-step k > 0: greedy nearest-centroid matching ----
        // For every descendant we find the nearest ancestor within the drift
        // threshold; descendants that share an ancestor count as a SPLIT,
        // unmatched descendants are BIRTHS, unmatched ancestors are DEATHS.
        for (std::size_t gi = 1; gi < groups.size(); ++gi)
        {
            const auto& group = groups[gi];
            std::vector<CompInfo> cur;
            cur.reserve(group.size());

            // Collect descendant centroids
            for (std::size_t li : group)
            {
                CompInfo c;
                c.layer_idx = li;
                c.centroid  = centroidOf(layers[li]);
                cur.push_back(c);
            }

            // For each descendant, find nearest ancestor within threshold
            std::vector<int> assigned_ancestor(cur.size(), -1);
            std::vector<int> ancestor_use_count(prev.size(), 0);

            for (std::size_t di = 0; di < cur.size(); ++di)
            {
                int   best_a = -1;
                float best_d = drift_thresh;
                for (std::size_t ai = 0; ai < prev.size(); ++ai)
                {
                    const float d = glm::length(cur[di].centroid - prev[ai].centroid);
                    if (d < best_d) { best_d = d; best_a = static_cast<int>(ai); }
                }
                assigned_ancestor[di] = best_a;
                if (best_a >= 0) ++ancestor_use_count[static_cast<std::size_t>(best_a)];
            }

            // Phase 1: descendants with a unique ancestor inherit its branch_id.
            // Phase 2: descendants sharing an ancestor (split): one keeps the
            //          branch_id, the others get a fresh one and we log SPLIT.
            // Phase 3: unmatched descendants are BIRTHS.
            // Phase 4: unmatched ancestors are DEATHS (just count, nothing to do).

            // Track which descendant index keeps the inherited id for each
            // multiply-assigned ancestor (the first one).
            std::vector<bool> ancestor_inherited(prev.size(), false);

            for (std::size_t di = 0; di < cur.size(); ++di)
            {
                const int a = assigned_ancestor[di];
                if (a < 0)
                {
                    cur[di].branch_id = next_branch++;
                    layers[cur[di].layer_idx].branch_id = cur[di].branch_id;
                    ++stats.births;
                    MEDUSA_INFO("BranchTracker: BIRTH at iso-step {}, branch {} "
                                "(no ancestor within {:.2f} mm)",
                                gi, cur[di].branch_id, drift_thresh);
                    continue;
                }

                if (!ancestor_inherited[static_cast<std::size_t>(a)])
                {
                    ancestor_inherited[static_cast<std::size_t>(a)] = true;
                    cur[di].branch_id = prev[static_cast<std::size_t>(a)].branch_id;
                }
                else
                {
                    // Split — same ancestor, additional descendant.
                    const std::uint32_t new_id = next_branch++;
                    cur[di].branch_id = new_id;
                    ++stats.splits;
                    MEDUSA_INFO("BranchTracker: SPLIT at iso-step {}, ancestor branch {} "
                                "-> child branch {} (and original kept)",
                                gi, prev[static_cast<std::size_t>(a)].branch_id, new_id);
                }
                layers[cur[di].layer_idx].branch_id = cur[di].branch_id;
            }

            // Merge detection: any case where two distinct ancestors picked the
            // same descendant. With the above 1:N assignment that doesn't happen
            // directly, so we approximate: count ancestors that didn't survive
            // AND whose nearest descendant inherited from a different ancestor.
            for (std::size_t ai = 0; ai < prev.size(); ++ai)
            {
                if (ancestor_use_count[ai] == 0) continue; // death below

                // Check if this ancestor's nearest descendant exists and has
                // a different (already-inherited) ancestor than this one.
                int best_d = -1;
                float best_dist = drift_thresh;
                for (std::size_t di = 0; di < cur.size(); ++di)
                {
                    const float d = glm::length(prev[ai].centroid - cur[di].centroid);
                    if (d < best_dist) { best_dist = d; best_d = static_cast<int>(di); }
                }
                if (best_d >= 0
                    && assigned_ancestor[static_cast<std::size_t>(best_d)] != static_cast<int>(ai)
                    && ancestor_inherited[ai] == false)
                {
                    ++stats.merges;
                    MEDUSA_INFO("BranchTracker: MERGE at iso-step {}, branch {} merged "
                                "into branch {}",
                                gi, prev[ai].branch_id,
                                cur[static_cast<std::size_t>(best_d)].branch_id);
                }
            }

            // Deaths: ancestors with zero descendants assigned to them.
            for (std::size_t ai = 0; ai < prev.size(); ++ai)
            {
                if (ancestor_use_count[ai] == 0)
                {
                    ++stats.deaths;
                    MEDUSA_DEBUG("BranchTracker: DEATH at iso-step {}, branch {} "
                                 "(no descendant within {:.2f} mm)",
                                 gi, prev[ai].branch_id, drift_thresh);
                }
            }

            stats.total_components += static_cast<std::uint32_t>(cur.size());
            MEDUSA_DEBUG("BranchTracker: iso[{}] components={} active_branches={}",
                         gi, cur.size(),
                         std::count_if(cur.begin(), cur.end(),
                                       [](const CompInfo& c) { return c.branch_id != 0; }));

            prev = std::move(cur);
        }

        stats.total_branches = next_branch - 1;

        MEDUSA_INFO("BranchTracker: tracked {} branches over {} iso-steps "
                    "({} births, {} deaths, {} splits, {} merges, {} components total)",
                    stats.total_branches, groups.size(), stats.births, stats.deaths,
                    stats.splits, stats.merges, stats.total_components);
        return stats;
    }
} // namespace slicing::crystal
