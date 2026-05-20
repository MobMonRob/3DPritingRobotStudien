/**
 * @file i_branch_scheduler.h
 * @brief Strategy interface for ordering print operations across branches.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_I_BRANCH_SCHEDULER_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_I_BRANCH_SCHEDULER_H_

#include <cstdint>
#include <vector>

#include "common/layer.h"

namespace slicing::crystal
{
    /**
     * @brief Identifies one print operation: a layer index plus its branch.
     *
     * The scheduler reorders these entries to determine the print sequence.
     */
    struct PrintOp
    {
        std::uint32_t layer_index{0};   ///< Iso-layer index (0 = bottom).
        std::uint32_t branch_id{0};     ///< Branch identifier within the layer.
        std::uint32_t component_index{0}; ///< Connected-component index inside the layer.
    };

    /**
     * @brief Abstract scheduling strategy.
     *
     * Implementations decide in which order branch-components are printed across
     * layers in order to balance material distribution and avoid tipping the part.
     * Default: AlternatingScheduler (round-robin per layer).
     */
    class IBranchScheduler
    {
    public:
        virtual ~IBranchScheduler() = default;

        /**
         * @brief Reorders the given operations into the desired print sequence.
         * @param ops Operations grouped by layer (caller convention: stable per-layer order).
         * @return Operations in the order they should be executed.
         */
        [[nodiscard]] virtual std::vector<PrintOp> schedule(const std::vector<PrintOp>& ops) const = 0;

        /// Display name (for logging / UI).
        [[nodiscard]] virtual const char* name() const noexcept = 0;
    };

    /**
     * @brief Convenience wrapper: reorders a Layer vector in place using the
     *        given scheduling strategy.
     *
     * Builds a PrintOp per Layer (component_index = position within iso-step),
     * delegates to @c scheduler.schedule, then reorders @p layers to match.
     * Layers with the same Layer.index are considered one iso-step group.
     */
    void scheduleLayers(const IBranchScheduler& scheduler,
                        std::vector<Layer>& layers);
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_I_BRANCH_SCHEDULER_H_
