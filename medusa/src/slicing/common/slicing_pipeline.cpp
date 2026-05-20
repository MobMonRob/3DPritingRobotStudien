/**
 * @file slicing_pipeline.cpp
 * @brief Implementation of the slicing pipeline orchestrator.
 */

#include "slicing_pipeline.h"
#include "infill_generator.h"
#include "path_planner.h"
#include "triangle_mesh.h"
#include "logger.h"
#include "crystal/crystal_slicer.h"

namespace slicing
{
    PipelineResult SlicingPipeline::execute(ISlicer& slicer, const geometry::TriangleMesh& mesh)
    {
        PipelineResult result;

        MEDUSA_INFO("SlicingPipeline: starting with algorithm '{}'", slicer.name());

        // Step 1: Slice
        result.layers = slicer.slice(mesh);

        // Crystal produces its own 6-DOF toolpath internally (later steps).
        // Until that is wired up, fs->toolpath() is empty -> fall through to
        // the generic InfillGenerator/PathPlanner so users see the planar
        // base layers right away. Once Crystal emits real segments we
        // bypass the generic stages to preserve the per-waypoint poses.
        if (auto* fs = dynamic_cast<crystal::CrystalSlicer*>(&slicer);
            fs && !fs->toolpath().segments.empty())
        {
            result.toolpath = fs->toolpath();
            result.coverage = fs->coverage();
            result.success  = true;
            MEDUSA_INFO("SlicingPipeline: Crystal direct toolpath ({} segments, "
                        "{} layers, coverage={:.1f}%)",
                        result.toolpath.num_segments(), result.layers.size(),
                        result.coverage * 100.0f);
            return result;
        }

        if (result.layers.empty())
        {
            MEDUSA_WARN("SlicingPipeline: slicer produced no layers");
            return result;
        }
        MEDUSA_INFO("SlicingPipeline: {} layers from slicer", result.layers.size());

        // Step 2: Infill
        InfillGenerator::generate(result.layers);

        // Step 3: Toolpath
        result.toolpath = PathPlanner::plan(result.layers);
        result.coverage = slicer.coverage();
        result.success = !result.toolpath.segments.empty();

        MEDUSA_INFO("SlicingPipeline: complete ({} segments, coverage={:.1f}%)",
                    result.toolpath.num_segments(), result.coverage * 100.0f);

        return result;
    }
} // namespace slicing
