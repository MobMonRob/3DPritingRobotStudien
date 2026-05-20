/**
 * @file crystal_slicer.h
 * @brief Public facade of the Crystal non-planar slicer.
 *
 * Crystal produces curvilinear print layers along iso-surfaces of a harmonic
 * scalar field Phi defined on the input mesh. The first N layers stay planar
 * (bed adhesion), then iso-layers follow the geometry's natural growth
 * direction. Branches (e.g. arms of a T-shape) are detected and printed in
 * an alternating round-robin pattern by default.
 *
 * Step 1 (current): Skeleton implementation. Returns no layers; downstream
 * pipeline still receives an empty toolpath. Subsequent steps fill in:
 *   - ScalarFieldSolver  (harmonic Phi via libigl + Eigen)
 *   - BaseLayerBuilder   (planar Z-layers + bed face detection)
 *   - IsosurfaceExtractor (per-face marching contours)
 *   - BranchTracker      (connected components + topology tracking)
 *   - PathGenerator      (perimeter + (u,v)-rectilinear infill)
 *   - PoseBuilder        (6-DOF poses from -grad Phi + tangent)
 *   - Branch scheduling  (IBranchScheduler strategy)
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_SLICER_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_SLICER_H_

#include <string>
#include <vector>

#include "crystal_params.h"
#include "common/i_slicer.h"
#include "common/layer.h"
#include "scalar_field.h"
#include "common/toolpath.h"
#include "triangle_mesh.h"

namespace geometry { struct TriangleMesh; }

namespace slicing::crystal
{
    /**
     * @brief Non-planar curvilinear slicer with branch-aware scheduling.
     *
     * Implements ISlicer for compatibility with the existing pipeline and UI,
     * but additionally exposes a fully-formed Toolpath via @ref toolpath().
     * The SlicingPipeline detects this and skips the generic InfillGenerator /
     * PathPlanner stages, so 6-DOF poses are preserved end-to-end.
     */
    class CrystalSlicer final : public ISlicer
    {
    public:
        explicit CrystalSlicer(CrystalParams params);

        // --- ISlicer interface ---
        [[nodiscard]] std::vector<Layer> slice(const geometry::TriangleMesh& mesh) override;
        [[nodiscard]] std::string name() const override { return "Crystal"; }
        [[nodiscard]] float coverage() const override { return mCoverage; }

        /**
         * @brief Returns the Toolpath produced by the most recent slice() call.
         *
         * Unlike the other slicers, Crystal computes per-waypoint 6-DOF
         * poses itself; downstream stages should consume this directly.
         */
        [[nodiscard]] const Toolpath& toolpath() const noexcept { return mToolpath; }

        /**
         * @brief Returns the harmonic scalar field Phi computed on the input
         *        mesh during the most recent slice() call.
         *
         * Per-vertex array of length mesh.numVertices() once Step 2 ran
         * successfully; empty otherwise.
         */
        [[nodiscard]] const ScalarField& scalar_field() const noexcept { return mScalarField; }

        /**
         * @brief Returns the (refined) mesh that the harmonic Phi-solve and
         *        iso-extraction were run on.
         *
         * Crystal subdivides a working copy of the input mesh before the
         * solve so the harmonic system has enough degrees of freedom on
         * coarse inputs (T-junctions, primitive shapes, …). This getter
         * exposes that refined mesh so visualisations (Phi heatmap, etc.)
         * can render against the same vertex set the Phi values are indexed
         * by. Empty until slice() has been called.
         */
        [[nodiscard]] const geometry::TriangleMesh& refined_mesh() const noexcept { return mRefinedMesh; }

        [[nodiscard]] const CrystalParams& params() const noexcept { return mParams; }

    private:
        CrystalParams        mParams;
        Toolpath               mToolpath{};
        ScalarField            mScalarField{};
        geometry::TriangleMesh mRefinedMesh{};
        float                  mCoverage{0.0f};

        /// Builds @ref mToolpath from already-scheduled @p layers. Each
        /// Layer's contour points are emitted as Segments carrying the
        /// branch_id; a Travel segment is inserted whenever consecutive
        /// segments are not spatially contiguous (which happens at every
        /// branch switch as well as between disconnected rings).
        Toolpath buildToolpath(const std::vector<Layer>& layers) const;
    };
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_CRYSTAL_SLICER_H_
