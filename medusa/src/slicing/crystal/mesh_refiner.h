/**
 * @file mesh_refiner.h
 * @brief Internal mesh subdivision for Crystal (Step 2 prerequisite).
 *
 * The harmonic Phi-solve has only as many degrees of freedom as the input
 * mesh has vertices. A coarse cube/T (≤ 50 verts) collapses Phi into a
 * trivial linear function over huge faces — every downstream stage then
 * inherits that under-resolution. We therefore refine the mesh internally
 * to a target maximum edge length BEFORE the solve.
 *
 * Strategy: iterative 4-1 split via libigl's igl::upsample. This is a pure
 * mid-edge insertion — no smoothing, no topology change, sharp features
 * (T-junction, pyramid pinch point, …) are preserved bit-exact. Loop
 * subdivision was rejected because it relocates existing vertices.
 *
 * Refinement only affects the local mesh used by the Crystal pipeline;
 * the original mesh from the loader is never modified (we work on a copy).
 */

#ifndef MEDUSA_SRC_SLICING_CRYSTAL_MESH_REFINER_H_
#define MEDUSA_SRC_SLICING_CRYSTAL_MESH_REFINER_H_

#include "triangle_mesh.h"

namespace slicing::crystal
{
    struct MeshRefinerConfig
    {
        /// Maximum allowed edge length after refinement (mm). The refiner
        /// keeps splitting until every edge is shorter than this OR
        /// max_iterations is reached.
        float min_edge_length{1.0f};

        /// Hard cap on subdivision iterations. Each iteration multiplies
        /// vertex count by ~4 — 8 iterations turn a 16-vertex T into
        /// ~260k verts, which is plenty for a harmonic solve. Beyond that
        /// the solve cost (O(V^1.5)) dominates and we hit memory pressure.
        int max_iterations{8};

        /// If the input mesh has fewer than this many vertices the refiner
        /// will warn and force at least one subdivision pass even when the
        /// max_edge_length is already satisfied. Catches over-decimated
        /// inputs that pass the edge test only because all faces are tiny
        /// slivers (pathological STL exports).
        int min_vertex_warn_threshold{200};
    };

    /**
     * @brief Refines @p mesh in-place by iterated 4-1 split.
     *
     * On entry @p mesh must be a copy of the original — this function
     * mutates vertices, faces, and bounds but clears auxiliary arrays
     * (face_normals, face_to_face, vertex_to_faces) since they would be
     * stale after subdivision and Crystal does not consume them.
     *
     * @param mesh  Mesh to refine in place.
     * @param cfg   Refinement parameters.
     * @return true on success (or no-op), false if the input is invalid.
     */
    bool refineMesh(geometry::TriangleMesh& mesh,
                    const MeshRefinerConfig& cfg = {});
} // namespace slicing::crystal

#endif // MEDUSA_SRC_SLICING_CRYSTAL_MESH_REFINER_H_
