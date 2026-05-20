/**
 * @file alternating_scheduler.h
 * @brief Default branch scheduler: round-robin across branches per layer.
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_ALTERNATING_SCHEDULER_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_ALTERNATING_SCHEDULER_H_

#include "i_branch_scheduler.h"

namespace slicing::crystal
{
    /**
     * @brief Alternating round-robin scheduler.
     *
     * For every layer index, prints all branches once before advancing to the
     * next layer. Avoids printing one side fully before another and keeps the
     * centre of mass balanced.
     *
     * Stub implementation returns @p ops unchanged for now; the real ordering
     * is provided in a later step together with branch detection.
     */
    class AlternatingScheduler final : public IBranchScheduler
    {
    public:
        [[nodiscard]] std::vector<PrintOp> schedule(const std::vector<PrintOp>& ops) const override;
        [[nodiscard]] const char* name() const noexcept override { return "Alternating"; }
    };
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_ALTERNATING_SCHEDULER_H_
