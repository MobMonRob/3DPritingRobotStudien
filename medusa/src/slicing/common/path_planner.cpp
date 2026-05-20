/**
 * @file path_planner.cpp
 * @brief Implementation of the layer-to-toolpath conversion.
 */

#include "path_planner.h"
#include "logger.h"

#include <glm/geometric.hpp>
#include <cmath>

namespace slicing
{
    Toolpath PathPlanner::plan(const std::vector<Layer>& layers)
    {
        Toolpath toolpath;

        if (layers.empty())
        {
            MEDUSA_WARN("PathPlanner: no layers to plan");
            return toolpath;
        }

        // Set origin from the first layer's first point
        if (!layers.front().contour_points.empty())
        {
            toolpath.origin = layers.front().contour_points.front();
        }

        uint32_t branchId = 0;
        glm::vec3 prevDirection{0.0f};
        uint32_t skippedLayers = 0;

        // Collect all print segments first, then insert travel moves between them.
        std::vector<Segment> printSegments;

        for (const auto& layer : layers)
        {
            const auto& pts = layer.contour_points;
            const auto& nrm = layer.contour_normals;

            if (pts.size() < 2)
            {
                MEDUSA_DEBUG("PathPlanner: layer {} skipped (only {} contour point(s))",
                             layer.index, pts.size());
                ++skippedLayers;
                continue;
            }

            branchId = layer.branch_id;

            const size_t contourEnd = (layer.contour_count > 0) ? layer.contour_count : pts.size();

            // Contour intersection pairs: (pts[0], pts[1]), (pts[2], pts[3]), ...
            for (size_t i = 0; i + 1 < pts.size(); i += 2)
            {
                Segment seg;
                seg.start = pts[i];
                seg.end = pts[i + 1];
                seg.branch_id = branchId;
                seg.growth_step = layer.index;
                seg.is_infill = (i >= contourEnd);

                // Orientation: average of the two endpoint normals
                if (i < nrm.size() && i + 1 < nrm.size())
                {
                    glm::vec3 avgNormal = nrm[i] + nrm[i + 1];
                    float len = glm::length(avgNormal);
                    seg.orientation = (len > 1e-8f) ? avgNormal / len : glm::vec3(0.0f, 1.0f, 0.0f);
                }

                // Direction delta
                glm::vec3 dir = seg.end - seg.start;
                float dirLen = glm::length(dir);
                if (dirLen > 1e-8f)
                {
                    dir /= dirLen;
                    float prevLen = glm::length(prevDirection);
                    if (prevLen > 1e-8f)
                    {
                        float dot = glm::clamp(glm::dot(dir, prevDirection), -1.0f, 1.0f);
                        seg.direction_delta = std::acos(dot);
                    }
                    prevDirection = dir;
                }

                printSegments.push_back(seg);
            }
        }

        // Interleave travel moves between non-contiguous print segments.
        static constexpr float kTravelThreshold = 1e-4f;
        uint32_t travelCount = 0;

        for (size_t i = 0; i < printSegments.size(); ++i)
        {
            // Insert travel move if the previous segment's end doesn't match this segment's start.
            if (i > 0)
            {
                const glm::vec3& prevEnd = printSegments[i - 1].end;
                const glm::vec3& curStart = printSegments[i].start;
                float gap = glm::length(curStart - prevEnd);

                if (gap > kTravelThreshold)
                {
                    Segment travel;
                    travel.start = prevEnd;
                    travel.end = curStart;
                    travel.branch_id = printSegments[i].branch_id;
                    travel.growth_step = printSegments[i].growth_step;
                    travel.orientation = printSegments[i].orientation;
                    travel.is_travel = true;
                    toolpath.segments.push_back(travel);
                    ++travelCount;
                }
            }

            toolpath.segments.push_back(printSegments[i]);
        }

        MEDUSA_INFO("PathPlanner: generated {} print + {} travel segments across {} layers ({} skipped)",
                    printSegments.size(), travelCount, layers.size(), skippedLayers);
        return toolpath;
    }
} // namespace slicing
