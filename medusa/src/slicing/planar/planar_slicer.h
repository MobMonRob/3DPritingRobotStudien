/**
 * @file planar_slicer.h
 * @brief Classical horizontal planar slicing algorithm.
 */

#ifndef MEDUSA_SRC_SLICING_PLANAR_SLICER_H_
#define MEDUSA_SRC_SLICING_PLANAR_SLICER_H_

#include "common/i_slicer.h"
#include "common/slicer_params.h"

namespace slicing
{
    /**
     * @brief Horizontal planar slicer.
     *
     * Slices the mesh with parallel horizontal planes at constant Z intervals.
     * Serves as reference implementation and pipeline validation tool.
     */
    class PlanarSlicer : public ISlicer
    {
    public:
        explicit PlanarSlicer(const SlicerParams& params = {});

        std::vector<Layer> slice(const geometry::TriangleMesh& mesh) override;
        [[nodiscard]] std::string name() const override { return "Planar"; }
        [[nodiscard]] float coverage() const override { return mCoverage; }

        /** @brief Mutable access to parameters. */
        SlicerParams& params() { return mParams; }
        [[nodiscard]] const SlicerParams& params() const { return mParams; }

    private:
        SlicerParams mParams;
        float mCoverage{0.0f};

        /**
         * @brief Intersects the mesh with a horizontal plane at given Y height.
         * @param mesh Input mesh.
         * @param y Plane height (Y-up convention).
         * @param points Output contour points.
         * @param normals Output normals at contour points.
         */
        static void intersectPlane(const geometry::TriangleMesh& mesh, float planePos, int upAxis,
                                   std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals);
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_PLANAR_SLICER_H_
