/**
 * @file i_slicer.h
 * @brief Abstract interface for slicing algorithms (Strategy pattern).
 */

#ifndef MEDUSA_SRC_SLICING_I_SLICER_H_
#define MEDUSA_SRC_SLICING_I_SLICER_H_

#include <string>
#include <vector>

#include "layer.h"

namespace geometry
{
    struct TriangleMesh;
}

namespace slicing
{
    /**
     * @brief Abstract base class for slicing algorithms.
     *
     * Implementations (PlanarSlicer, BaseSlicer, CrystalSlicer) are selected at runtime
     * via the Dear ImGui UI dropdown.
     */
    class ISlicer
    {
    public:
        virtual ~ISlicer() = default;

        /**
         * @brief Performs slicing on the given mesh.
         * @param mesh The input triangle mesh to slice.
         * @return Vector of generated layers.
         */
        virtual std::vector<Layer> slice(const geometry::TriangleMesh& mesh) = 0;

        /**
         * @brief Returns the display name of this algorithm (for UI).
         */
        [[nodiscard]] virtual std::string name() const = 0;

        /**
         * @brief Returns the current coverage ratio (0.0 – 1.0).
         *
         * Meaningful during and after slice(). For planar slicing, this
         * represents the fraction of Z-range covered.
         */
        [[nodiscard]] virtual float coverage() const { return 0.0f; }
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_I_SLICER_H_
