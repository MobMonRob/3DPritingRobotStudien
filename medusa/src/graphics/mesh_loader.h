/**
 * @file mesh_loader.h
 * @brief 3D mesh file loading via Assimp.
 */

#ifndef MEDUSA_SRC_GRAPHICS_MESH_LOADER_H_
#define MEDUSA_SRC_GRAPHICS_MESH_LOADER_H_

#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace graphics
{
    /**
 * @brief Axis-aligned bounding box.
 */
    struct MeshBounds
    {
        glm::vec3 min{0.0f};
        glm::vec3 max{0.0f};

        /** @brief Computes the center of the bounding box. */
        [[nodiscard]] glm::vec3 center() const { return 0.5f * (min + max); }

        /** @brief Computes the radius (half of max extent). */
        [[nodiscard]] float radius() const;
    };

    /**
 * @brief Result of loading a mesh file.
 */
    struct MeshLoadResult
    {
        std::vector<float> vertices; ///< Interleaved vertex data (pos.xyz + normal.xyz).
        MeshBounds bounds; ///< Bounding box of the mesh.

        /** @brief Returns true if loading was successful. */
        [[nodiscard]] bool isValid() const { return !vertices.empty(); }

        /** @brief Returns vertex count (vertices.size() / 6). */
        [[nodiscard]] size_t vertexCount() const { return vertices.size() / 6; }
    };

    /**
 * @brief Loads 3D mesh files using Assimp.
 *
 * Supports many formats: STL, OBJ, FBX, PLY, 3DS, etc.
 * This is a stateless utility class with static methods only.
 */
    class MeshLoader
    {
    public:
        MeshLoader() = delete;

        /**
     * @brief Loads a mesh file from a path.
     * @param path File path.
     * @return Load result (check isValid()).
     */
        static MeshLoadResult load(const std::string& path);

        /**
     * @brief Tries to load from multiple candidate paths.
     * @param paths Candidate file paths.
     * @return Load result from first successful path (check isValid()).
     */
        static MeshLoadResult loadFromCandidates(const std::vector<std::string>& paths);
    };
} // namespace graphics

#endif // MEDUSA_SRC_GRAPHICS_MESH_LOADER_H_