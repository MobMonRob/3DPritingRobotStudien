/**
 * @file segment.h
 * @brief Toolpath segment connecting two 3D points with metadata.
 */

#ifndef MEDUSA_SRC_SLICING_SEGMENT_H_
#define MEDUSA_SRC_SLICING_SEGMENT_H_

#include <cstdint>
#include <glm/glm.hpp>

namespace slicing
{
    /**
     * @brief A single toolpath segment between two points.
     *
     * Carries metadata for 6-axis robot path planning:
     * - orientation: TCP direction (normal to local layer surface)
     * - branch_id: which growth branch this segment belongs to
     * - growth_step: iteration when this segment was created
     * - direction_delta: angular change from predecessor (radians)
     */
    struct Segment
    {
        /// Start point in 3D space.
        glm::vec3 start{0.0f};

        /// End point in 3D space.
        glm::vec3 end{0.0f};

        /// Tool orientation (TCP normal vector at this segment).
        glm::vec3 orientation{0.0f, 0.0f, 1.0f};

        /// Branch identifier for multi-branch growth.
        uint32_t branch_id{0};

        /// Growth iteration when this segment was generated.
        uint32_t growth_step{0};

        /// Angular change from the preceding segment (radians).
        float direction_delta{0.0f};

        /// True if this segment is infill (not a contour/perimeter segment).
        bool is_infill{false};

        /// True if this is a travel move (no extrusion, just repositioning).
        bool is_travel{false};
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_SEGMENT_H_
