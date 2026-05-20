/**
 * @file triangle_mesh.h
 * @brief Indexed triangle mesh with precomputed adjacency tables for algorithmic processing.
 */

#ifndef MEDUSA_SRC_GEOMETRY_TRIANGLE_MESH_H_
#define MEDUSA_SRC_GEOMETRY_TRIANGLE_MESH_H_

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "aabb.h"

namespace geometry
{
    /// Sentinel value indicating no adjacent face (boundary edge).
    constexpr uint32_t NO_ADJACENT_FACE = UINT32_MAX;

    /**
     * @brief Indexed triangle mesh with adjacency information.
     *
     * Semi-immutable: topology (vertices, faces, adjacency) is fixed after construction.
     * Annotations (e.g. per-face labels) should be stored in separate parallel structures.
     *
     * Adjacency data is required for front-propagation algorithms.
     */
    struct TriangleMesh
    {
        /// Per-vertex positions.
        std::vector<glm::vec3> vertices;

        /// Per-vertex normals.
        std::vector<glm::vec3> normals;

        /// Triangle index triples (indices into vertices/normals).
        std::vector<glm::uvec3> faces;

        /// Per-face normals (one per triangle).
        std::vector<glm::vec3> face_normals;

        /// Per-face adjacency: for face i, face_to_face[i][j] is the neighbor
        /// sharing edge j (edge 0 = v0-v1, edge 1 = v1-v2, edge 2 = v2-v0).
        /// Uses NO_ADJACENT_FACE for boundary edges.
        std::vector<glm::uvec3> face_to_face;

        /// Per-vertex list of incident face indices.
        std::vector<std::vector<uint32_t>> vertex_to_faces;

        /// Axis-aligned bounding box.
        AABB bounds;

        /** @brief Returns true if the mesh contains at least one face. */
        [[nodiscard]] bool isValid() const { return !faces.empty() && !vertices.empty(); }

        /** @brief Returns the number of vertices. */
        [[nodiscard]] size_t numVertices() const { return vertices.size(); }

        /** @brief Returns the number of faces (triangles). */
        [[nodiscard]] size_t numFaces() const { return faces.size(); }
    };
} // namespace geometry

#endif // MEDUSA_SRC_GEOMETRY_TRIANGLE_MESH_H_
