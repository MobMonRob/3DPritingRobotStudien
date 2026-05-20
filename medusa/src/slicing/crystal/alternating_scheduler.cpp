/**
 * @file alternating_scheduler.cpp
 * @brief Default round-robin branch scheduler.
 *
 * Within one iso-step (= group of Layers sharing Layer.index) the scheduler
 * orders components by branch_id ascending — this gives a deterministic
 * round-robin pattern: when iso-step k contains branches {1, 2, 3}, the
 * scheduler prints branch 1 first, then 2, then 3, then advances to k+1
 * and repeats. Across iso-steps the order is monotone (k -> k+1) so the
 * Z-axis always grows.
 *
 * Travel moves between branch switches are inserted later by the toolpath
 * builder in CrystalSlicer (the scheduler only decides the order, not
 * the move type).
 */

#include "alternating_scheduler.h"

#include <algorithm>
#include <map>
#include <utility>
#include <vector>

#include "i_branch_scheduler.h"

namespace slicing::crystal
{
    std::vector<PrintOp> AlternatingScheduler::schedule(const std::vector<PrintOp>& ops) const
    {
        // Group ops by layer in their original order, then sort branches
        // ascending within each layer. Layers themselves keep ascending order.
        std::map<std::uint32_t, std::vector<PrintOp>> by_layer;
        for (const auto& op : ops)
        {
            by_layer[op.layer_index].push_back(op);
        }

        std::vector<PrintOp> ordered;
        ordered.reserve(ops.size());
        for (auto& [layer, group] : by_layer)
        {
            // Stable sort by branch_id so round-robin order is deterministic
            // and ties (rare; same branch_id in one layer would mean a
            // BranchTracker bug) preserve their original relative order.
            std::stable_sort(group.begin(), group.end(),
                             [](const PrintOp& a, const PrintOp& b) {
                                 return a.branch_id < b.branch_id;
                             });
            ordered.insert(ordered.end(), group.begin(), group.end());
        }
        return ordered;
    }

    void scheduleLayers(const IBranchScheduler& scheduler,
                        std::vector<Layer>& layers)
    {
        if (layers.empty()) return;

        // 1. Build PrintOps from the current Layer order.
        //    component_index = position of this Layer within its iso-step
        //    group (just the index in 'layers' suffices for permutation).
        std::vector<PrintOp> ops;
        ops.reserve(layers.size());
        for (std::size_t i = 0; i < layers.size(); ++i)
        {
            ops.push_back({layers[i].index,
                           layers[i].branch_id,
                           static_cast<std::uint32_t>(i)});
        }

        // 2. Run the strategy.
        const auto ordered = scheduler.schedule(ops);

        // 3. Apply the permutation. We use component_index as the original
        //    position into the input layer vector.
        std::vector<Layer> reordered;
        reordered.reserve(layers.size());
        for (const auto& op : ordered)
        {
            reordered.push_back(std::move(layers[op.component_index]));
        }
        layers = std::move(reordered);
    }
} // namespace slicing::crystal
