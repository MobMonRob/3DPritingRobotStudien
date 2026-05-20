/**
 * @file base_slicer.h
 * @brief Base slicing algorithm with overhang detection and queue-based
 *        multi-directional slicing.
 */

#ifndef MEDUSA_SRC_SLICING_BASE_SLICER_H_
#define MEDUSA_SRC_SLICING_BASE_SLICER_H_

#include <cstdint>
#include <queue>
#include <vector>

#include <glm/glm.hpp>

#include "common/i_slicer.h"
#include "common/slicer_params.h"

namespace geometry
{
    struct TriangleMesh;
}

namespace slicing
{
    /**
     * @brief Base slicing algorithm.
     *
     * Slices bottom-to-top with horizontal planes. When an overhang is
     * detected (cross-section extends beyond the previous layer), a virtual
     * wall is created and the overhanging region is queued. After finishing
     * the upward pass, queued regions are sliced with the growth direction
     * rotated 90° outward. This repeats until the whole part is covered.
     */
    class BaseSlicer : public ISlicer
    {
    public:
        explicit BaseSlicer(const BaseParams& params = {});

        std::vector<Layer> slice(const geometry::TriangleMesh& mesh) override;
        [[nodiscard]] std::string name() const override { return "Base"; }
        [[nodiscard]] float coverage() const override { return mCoverage; }

        BaseParams& params() { return mParams; }
        [[nodiscard]] const BaseParams& params() const { return mParams; }

    private:
        BaseParams mParams;
        float mCoverage{0.0f};

        /// A pending slicing run: direction + starting position.
        struct SliceRun
        {
            glm::vec3 growth_dir{0.0f, 1.0f, 0.0f};
            float start_h{0.0f};
            uint32_t branch_id{0};
        };

        /// Bounding rectangle in the UV plane perpendicular to growth.
        struct PlaneBounds
        {
            float uMin{0.0f}, uMax{0.0f};
            float vMin{0.0f}, vMax{0.0f};
            bool valid{false};
        };

        /// Execute one slicing run; may enqueue new runs for overhangs.
        void executeRun(const geometry::TriangleMesh& mesh,
                        const SliceRun& run,
                        std::vector<Layer>& layers,
                        std::queue<SliceRun>& pending,
                        uint32_t& layerIdx,
                        uint32_t& nextBranchId);

        /// Intersect the mesh with an arbitrary plane.
        static void intersectMeshWithPlane(const geometry::TriangleMesh& mesh,
                                           const glm::vec3& planePoint,
                                           const glm::vec3& planeNormal,
                                           std::vector<glm::vec3>& points,
                                           std::vector<glm::vec3>& normals);

        /// Build orthonormal U, V axes from a growth direction.
        static void computeUV(const glm::vec3& growth, glm::vec3& u, glm::vec3& v);

        /// Compute bounding rectangle of points projected onto U, V.
        static PlaneBounds computeBounds(const std::vector<glm::vec3>& points,
                                          const glm::vec3& u, const glm::vec3& v);

        /// Clip a segment to the allowed bounding box using Liang-Barsky algorithm.
        static bool clipSegment(glm::vec3& p0, glm::vec3& p1,
                                const glm::vec3& u, const glm::vec3& v,
                                const PlaneBounds& allowed);

        /// Clip segment pairs to the allowed region (filters and clips endpoints).
        static void filterPoints(std::vector<glm::vec3>& points,
                                  std::vector<glm::vec3>& normals,
                                  const glm::vec3& u, const glm::vec3& v,
                                  const PlaneBounds& allowed);
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_BASE_SLICER_H_
