/**
 * @file aabb.h
 * @brief Axis-aligned bounding box for geometric computations.
 */

#ifndef MEDUSA_SRC_GEOMETRY_AABB_H_
#define MEDUSA_SRC_GEOMETRY_AABB_H_

#include <glm/glm.hpp>
#include <limits>

namespace geometry
{
    /**
     * @brief Axis-aligned bounding box.
     */
    struct AABB
    {
        glm::vec3 min{std::numeric_limits<float>::max()};
        glm::vec3 max{std::numeric_limits<float>::lowest()};

        /** @brief Expands the AABB to include the given point. */
        void expand(const glm::vec3& point)
        {
            min = glm::min(min, point);
            max = glm::max(max, point);
        }

        /** @brief Returns the center of the bounding box. */
        [[nodiscard]] glm::vec3 center() const { return 0.5f * (min + max); }

        /** @brief Returns the size (extent) along each axis. */
        [[nodiscard]] glm::vec3 size() const { return max - min; }

        /** @brief Returns the half-diagonal length (bounding sphere radius). */
        [[nodiscard]] float radius() const { return 0.5f * glm::length(max - min); }

        /** @brief Returns true if the AABB has been expanded at least once. */
        [[nodiscard]] bool isValid() const
        {
            return min.x <= max.x && min.y <= max.y && min.z <= max.z;
        }
    };
} // namespace geometry

#endif // MEDUSA_SRC_GEOMETRY_AABB_H_
