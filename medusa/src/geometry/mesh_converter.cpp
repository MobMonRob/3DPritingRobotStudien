/**
 * @file mesh_converter.cpp
 * @brief Implementation of interleaved vertex data to TriangleMesh conversion.
 */

#include "mesh_converter.h"
#include "logger.h"

#include <glm/geometric.hpp>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace geometry
{
    namespace
    {
        /**
         * @brief Hash functor for glm::vec3, used for vertex welding.
         */
        struct Vec3Hash
        {
            size_t operator()(const glm::vec3& v) const
            {
                auto h = std::hash<float>{};
                size_t seed = h(v.x);
                seed ^= h(v.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= h(v.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                return seed;
            }
        };

        /**
         * @brief Quantizes a position for vertex welding.
         */
        glm::vec3 quantize(const glm::vec3& v, float epsilon)
        {
            float inv = 1.0f / epsilon;
            return glm::vec3(
                std::round(v.x * inv) * epsilon,
                std::round(v.y * inv) * epsilon,
                std::round(v.z * inv) * epsilon
            );
        }

        /**
         * @brief Represents an undirected edge for adjacency building.
         */
        struct Edge
        {
            uint32_t v0;
            uint32_t v1;

            bool operator==(const Edge& other) const { return v0 == other.v0 && v1 == other.v1; }
        };

        struct EdgeHash
        {
            size_t operator()(const Edge& e) const
            {
                size_t seed = std::hash<uint32_t>{}(e.v0);
                seed ^= std::hash<uint32_t>{}(e.v1) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                return seed;
            }
        };

        struct EdgeFaceInfo
        {
            uint32_t face_index;
            uint32_t edge_index;
        };

    } // anonymous namespace


    TriangleMesh convertFromInterleavedData(const float* data, size_t vertexCount, float weldEpsilon)
    {
        TriangleMesh mesh;

        if (!data || vertexCount == 0)
        {
            MEDUSA_WARN("convertFromInterleavedData: invalid input (null/empty)");
            return mesh;
        }

        if (vertexCount % 3 != 0)
        {
            MEDUSA_WARN("convertFromInterleavedData: vertex count ({}) not divisible by 3", vertexCount);
            return mesh;
        }

        const size_t faceCount = vertexCount / 3;

        // Step 1: Extract raw positions and normals, weld vertices by position
        std::unordered_map<glm::vec3, uint32_t, Vec3Hash> vertexMap;
        vertexMap.reserve(vertexCount);

        std::vector<uint32_t> rawToWelded(vertexCount);

        for (size_t i = 0; i < vertexCount; ++i)
        {
            const size_t offset = i * 6;
            glm::vec3 pos(data[offset + 0], data[offset + 1], data[offset + 2]);
            glm::vec3 normal(data[offset + 3], data[offset + 4], data[offset + 5]);

            glm::vec3 quantized = quantize(pos, weldEpsilon);

            auto it = vertexMap.find(quantized);
            if (it != vertexMap.end())
            {
                rawToWelded[i] = it->second;
                mesh.normals[it->second] += normal;
            }
            else
            {
                auto idx = static_cast<uint32_t>(mesh.vertices.size());
                vertexMap[quantized] = idx;
                rawToWelded[i] = idx;
                mesh.vertices.push_back(pos);
                mesh.normals.push_back(normal);
            }
        }

        // Re-normalize accumulated normals
        for (auto& n : mesh.normals)
        {
            float len = glm::length(n);
            if (len > 1e-8f)
            {
                n /= len;
            }
        }

        // Step 2: Build face indices
        mesh.faces.reserve(faceCount);
        for (size_t f = 0; f < faceCount; ++f)
        {
            glm::uvec3 face(
                rawToWelded[f * 3 + 0],
                rawToWelded[f * 3 + 1],
                rawToWelded[f * 3 + 2]
            );
            mesh.faces.push_back(face);
        }

        // Step 3: Compute derived data
        computeFaceNormals(mesh);
        buildFaceAdjacency(mesh);
        buildVertexToFaces(mesh);
        computeBounds(mesh);

        MEDUSA_INFO("TriangleMesh built: {} vertices, {} faces (welded from {} raw vertices)",
                    mesh.vertices.size(), mesh.faces.size(), vertexCount);

        return mesh;
    }

    TriangleMesh convertFromInterleavedData(const std::vector<float>& vertices, float weldEpsilon)
    {
        size_t vertexCount = vertices.size() / 6;
        return convertFromInterleavedData(vertices.data(), vertexCount, weldEpsilon);
    }

    void computeFaceNormals(TriangleMesh& mesh)
    {
        mesh.face_normals.resize(mesh.faces.size());
        size_t degenerateCount = 0;

        for (size_t i = 0; i < mesh.faces.size(); ++i)
        {
            const auto& f = mesh.faces[i];
            const glm::vec3& v0 = mesh.vertices[f.x];
            const glm::vec3& v1 = mesh.vertices[f.y];
            const glm::vec3& v2 = mesh.vertices[f.z];

            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 normal = glm::cross(edge1, edge2);

            float len = glm::length(normal);
            if (len > 1e-8f)
            {
                normal /= len;
            }
            else
            {
                ++degenerateCount;
            }

            mesh.face_normals[i] = normal;
        }

        if (degenerateCount > 0)
        {
            MEDUSA_WARN("computeFaceNormals: {} degenerate face(s) with zero-length normal (zero-area triangles)",
                        degenerateCount);
        }
    }

    void buildFaceAdjacency(TriangleMesh& mesh)
    {
        const size_t faceCount = mesh.faces.size();
        mesh.face_to_face.resize(faceCount, glm::uvec3(NO_ADJACENT_FACE));

        // Map: directed edge (v_min, v_max) → list of (face, edge_index)
        // We use undirected edges (sorted vertex pair) to find shared edges
        std::unordered_map<Edge, std::vector<EdgeFaceInfo>, EdgeHash> edgeMap;
        edgeMap.reserve(faceCount * 3);

        for (uint32_t fi = 0; fi < faceCount; ++fi)
        {
            const auto& f = mesh.faces[fi];
            uint32_t verts[3] = {f.x, f.y, f.z};

            // Edge 0: v0-v1, Edge 1: v1-v2, Edge 2: v2-v0
            for (uint32_t ei = 0; ei < 3; ++ei)
            {
                uint32_t a = verts[ei];
                uint32_t b = verts[(ei + 1) % 3];

                // Undirected: always store with smaller index first
                Edge key{std::min(a, b), std::max(a, b)};
                edgeMap[key].push_back({fi, ei});
            }
        }

        // For each edge shared by exactly 2 faces, set adjacency
        size_t nonManifoldEdges = 0;
        for (const auto& [edge, infos] : edgeMap)
        {
            if (infos.size() == 2)
            {
                const auto& a = infos[0];
                const auto& b = infos[1];
                mesh.face_to_face[a.face_index][a.edge_index] = b.face_index;
                mesh.face_to_face[b.face_index][b.edge_index] = a.face_index;
            }
            else if (infos.size() > 2)
            {
                ++nonManifoldEdges;
            }
        }

        if (nonManifoldEdges > 0)
        {
            MEDUSA_WARN("buildFaceAdjacency: {} non-manifold edge(s) found (shared by >2 faces) — adjacency left unset",
                        nonManifoldEdges);
        }
        else
        {
            MEDUSA_DEBUG("buildFaceAdjacency: mesh is manifold ({} edges)", edgeMap.size());
        }
    }

    void buildVertexToFaces(TriangleMesh& mesh)
    {
        mesh.vertex_to_faces.resize(mesh.vertices.size());

        for (uint32_t fi = 0; fi < mesh.faces.size(); ++fi)
        {
            const auto& f = mesh.faces[fi];
            mesh.vertex_to_faces[f.x].push_back(fi);
            mesh.vertex_to_faces[f.y].push_back(fi);
            mesh.vertex_to_faces[f.z].push_back(fi);
        }
    }

    void computeBounds(TriangleMesh& mesh)
    {
        mesh.bounds = AABB{};
        for (const auto& v : mesh.vertices)
        {
            mesh.bounds.expand(v);
        }
        const glm::vec3 sz = mesh.bounds.size();
        MEDUSA_DEBUG("computeBounds: AABB size=({:.4f}, {:.4f}, {:.4f}), radius={:.4f}",
                     sz.x, sz.y, sz.z, mesh.bounds.radius());
    }

} // namespace geometry
