/**
 * @file toolpath.cpp
 * @brief Implementation of Toolpath query methods.
 */

#include "toolpath.h"

#include <algorithm>
#include <set>

#include <glm/geometric.hpp>

namespace slicing
{
    float Toolpath::total_length() const
    {
        float len = 0.0f;
        for (const auto& seg : segments)
        {
            len += glm::length(seg.end - seg.start);
        }
        return len;
    }

    std::vector<uint32_t> Toolpath::branch_ids() const
    {
        std::set<uint32_t> ids;
        for (const auto& seg : segments)
        {
            ids.insert(seg.branch_id);
        }
        return {ids.begin(), ids.end()};
    }

    uint32_t Toolpath::num_branches() const
    {
        return static_cast<uint32_t>(branch_ids().size());
    }

    uint32_t Toolpath::num_segments() const
    {
        return static_cast<uint32_t>(segments.size());
    }

    std::vector<Segment> Toolpath::get_branch(uint32_t branch_id) const
    {
        std::vector<Segment> result;
        for (const auto& seg : segments)
        {
            if (seg.branch_id == branch_id)
            {
                result.push_back(seg);
            }
        }
        return result;
    }
} // namespace slicing
