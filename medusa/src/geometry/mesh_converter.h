/**
 * @file mesh_converter.h
 * @brief Conversion from raw interleaved vertex data to geometry::TriangleMesh.
 */

#ifndef MEDUSA_SRC_GEOMETRY_MESH_CONVERTER_H_
#define MEDUSA_SRC_GEOMETRY_MESH_CONVERTER_H_

#include <cstddef>
#include <vector>

#include "triangle_mesh.h"

namespace geometry
{
    /**
     * @brief Converts raw interleaved vertex data into an algorithmic TriangleMesh.
     *
     * The conversion:
     * 1. Extracts positions and normals from interleaved float data (pos.xyz + normal.xyz).
     * 2. Welds duplicate vertices (position-based) to build shared indices.
     * 3. Computes face normals.
     * 4. Builds face-to-face adjacency tables.
     * 5. Builds vertex-to-faces incidence lists.
     * 6. Computes the AABB.
     *
     * @param vertices Interleaved vertex data (pos.xyz + normal.xyz per vertex).
     * @param vertexCount Number of vertices (must be divisible by 3 for triangle faces).
     * @param weldEpsilon Tolerance for vertex welding (default: 1e-5).
     * @return A fully built TriangleMesh, or an empty mesh if input is invalid.
     */
    TriangleMesh convertFromInterleavedData(const float* vertices, size_t vertexCount,
                                            float weldEpsilon = 1e-5f);

    /**
     * @brief Convenience overload taking a vector of interleaved floats.
     * @param vertices Interleaved float data (6 floats per vertex: pos.xyz + normal.xyz).
     * @param weldEpsilon Tolerance for vertex welding.
     * @return A fully built TriangleMesh.
     */
    TriangleMesh convertFromInterleavedData(const std::vector<float>& vertices,
                                            float weldEpsilon = 1e-5f);

    /**
     * @brief Computes face normals for all triangles in the mesh.
     * @param mesh The mesh to compute normals for (face_normals will be resized and filled).
     */
    void computeFaceNormals(TriangleMesh& mesh);

    /**
     * @brief Builds face-to-face adjacency (face_to_face) from triangle indices.
     * @param mesh The mesh to build adjacency for.
     */
    void buildFaceAdjacency(TriangleMesh& mesh);

    /**
     * @brief Builds vertex-to-faces incidence lists.
     * @param mesh The mesh to build incidence for.
     */
    void buildVertexToFaces(TriangleMesh& mesh);

    /**
     * @brief Computes the AABB from vertex positions.
     * @param mesh The mesh to compute bounds for.
     */
    void computeBounds(TriangleMesh& mesh);

} // namespace geometry

#endif // MEDUSA_SRC_GEOMETRY_MESH_CONVERTER_H_
