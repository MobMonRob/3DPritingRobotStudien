/**
 * @file path_planner.h
 * @brief Converts filled layers into a Toolpath with segments.
 */

#ifndef MEDUSA_SRC_SLICING_PATH_PLANNER_H_
#define MEDUSA_SRC_SLICING_PATH_PLANNER_H_

#include <vector>
#include "layer.h"
#include "toolpath.h"

namespace slicing
{
    /**
     * @brief Converts a sequence of layers into a complete toolpath.
     *
     * Each pair of consecutive contour points in a layer becomes a Segment.
     * The orientation field is set from the interpolated surface normal.
     */
    class PathPlanner
    {
    public:
        /**
         * @brief Plans a toolpath from the given layers.
         * @param layers Filled layers (with contour points and normals).
         * @return Complete toolpath with all segments.
         */
        static Toolpath plan(const std::vector<Layer>& layers);
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_PATH_PLANNER_H_
