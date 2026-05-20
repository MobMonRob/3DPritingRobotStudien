/**
 * @file mesh.h
 * @brief GPU mesh representation (VAO/VBO management).
 */

#ifndef MEDUSA_SRC_GRAPHICS_MESH_H_
#define MEDUSA_SRC_GRAPHICS_MESH_H_

#include <string>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace graphics
{
    /**
 * @brief GPU-resident triangle mesh.
 *
 * Manages OpenGL VAO/VBO for an interleaved vertex buffer (pos.xyz + normal.xyz).
 * Use StlLoader to load mesh data, then call build() to upload to GPU.
 */
    class Mesh
    {
    public:
        /** @brief Vertex array object handle. */
        GLuint vao = 0;

        /** @brief Vertex buffer object handle. */
        GLuint vbo = 0;

        /** @brief Vertex count for glDrawArrays. */
        GLsizei count = 0;

        /** @brief Center of the bounding box (used for centering). */
        glm::vec3 center = glm::vec3(0.0f);

        /** @brief Minimum corner of the bounding box (used for floor-placement). */
        glm::vec3 boundsMin = glm::vec3(0.0f);

        /** @brief Approx. radius (half of max extent) used for normalization. */
        float radius = 1.0f;

        Mesh() = default;
        ~Mesh();

        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;
        Mesh(Mesh&&) = delete;
        Mesh& operator=(Mesh&&) = delete;

        /**
     * @brief Loads mesh from STL file candidates.
     *
     * Uses StlLoader internally. Tries each path until one succeeds.
     *
     * @param paths Candidate file paths.
     * @return True on success.
     */
        bool loadFromCandidates(const std::vector<std::string>& paths);

        /**
     * @brief Uploads interleaved vertex data to the GPU.
     *
     * @param vertices Interleaved vertex data (pos.xyz + normal.xyz per vertex).
     * @param meshCenter Center of the bounding box.
     * @param meshBoundsMin Minimum corner of the bounding box (for floor-placement).
     * @param meshRadius Radius for normalization.
     */
        void build(const std::vector<float>& vertices, const glm::vec3& meshCenter,
                   const glm::vec3& meshBoundsMin, float meshRadius);

        /**
     * @brief Draws the mesh with glDrawArrays(GL_TRIANGLES).
     */
        void draw() const;

        /**
     * @brief Releases GPU resources.
     */
        void clear();
    };
} // namespace graphics

#endif // MEDUSA_SRC_GRAPHICS_MESH_H_