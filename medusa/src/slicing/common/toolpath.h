/**
 * @file toolpath.h
 * @brief Complete toolpath definition with segment collection and query methods.
 */

#ifndef MEDUSA_SRC_SLICING_TOOLPATH_H_
#define MEDUSA_SRC_SLICING_TOOLPATH_H_

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "segment.h"

namespace slicing
{
    /**
     * @brief Complete toolpath consisting of chronologically ordered segments.
     *
     * Lives exclusively in memory — no serialization or export.
     */
    struct Toolpath
    {
        /// All segments in chronological order.
        std::vector<Segment> segments;

        /// Growth origin point.
        glm::vec3 origin{0.0f};

        /** @brief Total path length (sum of all segment lengths). */
        [[nodiscard]] float total_length() const;

        /** @brief Returns sorted, unique branch IDs present in the toolpath. */
        [[nodiscard]] std::vector<uint32_t> branch_ids() const;

        /** @brief Number of distinct branches. */
        [[nodiscard]] uint32_t num_branches() const;

        /** @brief Number of segments. */
        [[nodiscard]] uint32_t num_segments() const;

        /** @brief Returns all segments belonging to a specific branch. */
        [[nodiscard]] std::vector<Segment> get_branch(uint32_t branch_id) const;
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_TOOLPATH_H_
