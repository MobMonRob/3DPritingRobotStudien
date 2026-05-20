/**
 * @file mesh.cpp
 * @brief Implementation of GPU mesh management.
 */

#include "mesh.h"
#include "mesh_loader.h"
#include "logger.h"

#include <glad/glad.h>

namespace graphics
{
    Mesh::~Mesh()
    {
        clear();
    }

    void Mesh::clear()
    {
        if (vbo != 0)
        {
            glDeleteBuffers(1, &vbo);
            vbo = 0;
        }

        if (vao != 0)
        {
            glDeleteVertexArrays(1, &vao);
            vao = 0;
        }
        count = 0;
        center = glm::vec3(0.0f);
        radius = 1.0f;
    }

    bool Mesh::loadFromCandidates(const std::vector<std::string>& paths)
    {
        if (paths.empty())
        {
            MEDUSA_WARN("Mesh::loadFromCandidates called with empty path list");
            return false;
        }

        MEDUSA_INFO("Mesh load requested ({} candidate(s))", paths.size());

        const auto kResult = MeshLoader::loadFromCandidates(paths);
        if (!kResult.isValid())
        {
            MEDUSA_ERROR("Mesh load failed: no valid file found");
            return false;
        }

        float meshRadius = kResult.bounds.radius();
        if (meshRadius <= 0.0f)
        {
            MEDUSA_WARN("Mesh radius computed as <= 0. Forcing radius=1");
            meshRadius = 1.0f;
        }

        build(kResult.vertices, kResult.bounds.center(), meshRadius);

        MEDUSA_INFO("Mesh load successful (vertices={}, radius={})", count, radius);
        return true;
    }

    void Mesh::build(const std::vector<float>& vertices, const glm::vec3& meshCenter, const float meshRadius)
    {
        clear();

        count = static_cast<GLsizei>(vertices.size() / 6);
        if (count <= 0)
        {
            MEDUSA_WARN("Mesh::build called with empty vertex buffer");
            return;
        }

        center = meshCenter;
        radius = meshRadius;

        MEDUSA_DEBUG("Uploading mesh to GPU: vertices={}, bytes={}", count, vertices.size() * sizeof(float));

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)), vertices.data(),
                     GL_STATIC_DRAW);

        // Position attribute (location 0)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);

        // Normal attribute (location 1)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<const void*>(3 * sizeof(float)));

        glBindVertexArray(0);
    }

    void Mesh::draw() const
    {
        if (vao == 0 || count == 0)
        {
            return;
        }

        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, count);
        glBindVertexArray(0);
    }
} // namespace graphics