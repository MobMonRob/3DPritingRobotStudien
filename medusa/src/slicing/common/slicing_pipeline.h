/**
 * @file slicing_pipeline.h
 * @brief Orchestrates the complete Mesh → Slicer → Infill → Toolpath pipeline.
 */

#ifndef MEDUSA_SRC_SLICING_SLICING_PIPELINE_H_
#define MEDUSA_SRC_SLICING_SLICING_PIPELINE_H_

#include <memory>

#include "i_slicer.h"
#include "toolpath.h"

namespace geometry
{
    struct TriangleMesh;
}

namespace slicing
{
    /**
     * @brief Result of a complete slicing pipeline run.
     */
    struct PipelineResult
    {
        std::vector<Layer> layers;
        Toolpath toolpath;
        float coverage{0.0f};
        bool success{false};
    };

    /**
     * @brief Runs the full slicing pipeline: Mesh → Slicer → Infill → Toolpath.
     */
    class SlicingPipeline
    {
    public:
        /**
         * @brief Executes the pipeline with the given slicer.
         * @param slicer The slicing algorithm to use.
         * @param mesh The input mesh.
         * @return Pipeline result with layers and toolpath.
         */
        static PipelineResult execute(ISlicer& slicer, const geometry::TriangleMesh& mesh);
    };
} // namespace slicing

#endif // MEDUSA_SRC_SLICING_SLICING_PIPELINE_H_
