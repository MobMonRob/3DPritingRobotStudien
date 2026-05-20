/**
 * @file branch_tracker.h
 * @brief Component-tracking across iso-levels (Crystal Step 4).
 *
 * The IsoExtractor emits one Layer per connected component (= ring) of each
 * iso-surface; all Layers belonging to the same iso-step share the same
 * Layer.index. The BranchTracker walks those iso-steps in order, assigns a
 * stable branch_id to every component, and detects topology events:
 *
 *   - Birth   : a component at iso k has no nearby ancestor in iso k-1
 *               (= a new branch starts, e.g. a cantilever lifts off the trunk).
 *   - Split   : one component at iso k-1 has multiple descendants at iso k
 *               (= the trunk forks; T-crossbar tips eventually separate).
 *   - Merge   : multiple components at iso k-1 share a single descendant at
 *               iso k (= a hole closes; rare for monotone Phi).
 *   - Death   : an ancestor has no descendant (the branch tops out).
 *
 * The matching is centroid-based with a distance threshold derived from the
 * layer thickness (a component cannot move further between two layers than
 * a sane multiple of the layer height). Splits and merges are logged
 * explicitly with branch ids and iso-step indices.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_BRANCH_TRACKER_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_BRANCH_TRACKER_H_

#include <cstdint>
#include <vector>

#include "common/layer.h"

namespace slicing::crystal
{
    /// Aggregate stats from one BranchTracker pass — useful for diagnostics
    /// and for the CrystalSlicer's INFO summary line.
    struct BranchTrackingStats
    {
        std::uint32_t total_branches{0};      ///< Distinct branch_ids assigned.
        std::uint32_t total_components{0};    ///< Sum over all iso-steps.
        std::uint32_t splits{0};
        std::uint32_t merges{0};
        std::uint32_t births{0};
        std::uint32_t deaths{0};
    };

    /// Tracker configuration.
    struct BranchTrackerConfig
    {
        /// Maximum centroid distance (in mm) between an ancestor and a
        /// descendant component, expressed as a multiple of the layer
        /// thickness. Components whose centroids are further apart are
        /// treated as a birth/death pair instead of a continuation.
        ///
        /// Default 4.0 is generous: at 0.4 mm layers a component can drift
        /// up to 1.6 mm horizontally between iso-steps before it counts as
        /// a new branch. Tighten for noisy gradients, loosen for steeply
        /// curved iso-surfaces.
        float max_drift_factor{4.0f};

        /// Hard floor on the drift threshold (mm). Prevents the threshold
        /// from collapsing to zero on extremely thin layers.
        float min_drift_mm{0.5f};

        /// Iso-step index of the first iso-layer (= Layer.index value of
        /// the very first iso-component). Layers with Layer.index < this
        /// are considered planar base layers and get branch_id 0.
        std::uint32_t first_iso_layer_index{0};
    };

    /**
     * @brief Walks @p layers in order, mutates Layer.branch_id in place.
     *
     * Assumes @p layers are already sorted by Layer.index ascending and
     * that Layers belonging to the same iso-step share Layer.index. Order
     * within one iso-step is preserved.
     *
     * @param layers Mutable layer vector — branch_ids are overwritten.
     * @param cfg    Tracker configuration.
     * @return Aggregate statistics for diagnostic logging.
     */
    BranchTrackingStats assignBranchIds(std::vector<Layer>& layers,
                                        const BranchTrackerConfig& cfg);
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_BRANCH_TRACKER_H_
