/**
 * @file layer.h
 * @brief Represents a single (possibly non-planar) slice layer.
 */

#ifndef MEDUSA_SRC_SLICING_LAYER_H_
#define MEDUSA_SRC_SLICING_LAYER_H_

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace slicing
{
    /**
     * @brief A single layer (slice) of the model.
     *
     * For planar slicing, all contour_points share the same Z coordinate.
     * For non-planar slicing, contour_points follow the curved surface.
     */
    struct Layer
    {
        /// Layer index (0-based, bottom to top).
        uint32_t index{0};

        /// Branch identifier: 0 = stem, 1..N = arm index (1-based).
        uint32_t branch_id{0};

        /// Ordered contour points defining the layer boundary.
        std::vector<glm::vec3> contour_points;

        /// Normal vector at each contour point (same size as contour_points).
        std::vector<glm::vec3> contour_normals;

        /// Mesh face index that produced each contour point (same size as
        /// contour_points). Used by infill to restrict ray-casts to the
        /// local iso-surface faces.
        std::vector<uint32_t> contour_face_ids;

        /// Layer thickness in mm.
        float thickness{0.2f};

        /// Up axis used when this layer was sliced (0=X, 1=Y, 2=Z).
        /// Consumed by InfillGenerator to determine the scanline normal axis.
        int up_axis{1};

        /// Number of contour points (before infill points were appended).
        /// Points at indices [0, contour_count) are contour,
        /// points at [contour_count, contour_points.size()) are infill.
        size_t contour_count{0};
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_LAYER_H_
