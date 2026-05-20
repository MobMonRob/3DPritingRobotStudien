/**
 * @file phi_overlay_renderer.cpp
 * @brief Implementation of the Crystal Phi heatmap overlay renderer.
 */

#include "phi_overlay_renderer.h"

#include <glm/gtc/type_ptr.hpp>

#include "logger.h"
#include "triangle_mesh.h"

namespace graphics
{
    PhiOverlayRenderer::~PhiOverlayRenderer()
    {
        clear();
    }

    bool PhiOverlayRenderer::initialize()
    {
        MEDUSA_INFO("Initializing PhiOverlayRenderer shaders");

        if (!mShader.createFromFiles(
                MEDUSA_PROJECT_ROOT "/assets/shaders/phi_overlay.vert",
                MEDUSA_PROJECT_ROOT "/assets/shaders/phi_overlay.frag"))
        {
            MEDUSA_CRITICAL("PhiOverlayRenderer: shader program creation failed");
            return false;
        }

        mLocMvp    = mShader.loc("uMVP");
        mLocPhiMin = mShader.loc("uPhiMin");
        mLocPhiMax = mShader.loc("uPhiMax");
        if (mLocMvp < 0)
        {
            MEDUSA_WARN("PhiOverlayRenderer: uMVP uniform not found");
        }
        return true;
    }

    void PhiOverlayRenderer::clear()
    {
        if (mEbo) { glDeleteBuffers(1, &mEbo); mEbo = 0; }
        if (mVbo) { glDeleteBuffers(1, &mVbo); mVbo = 0; }
        if (mVao) { glDeleteVertexArrays(1, &mVao); mVao = 0; }
        mIndexCount = 0;
    }

    void PhiOverlayRenderer::upload(const geometry::TriangleMesh& mesh,
                                    const std::vector<float>& phi,
                                    float phi_min, float phi_max)
    {
        clear();

        if (!mesh.isValid() || phi.size() != mesh.numVertices())
        {
            MEDUSA_DEBUG("PhiOverlayRenderer::upload: invalid mesh or phi size mismatch "
                         "(verts={}, phi={})",
                         mesh.numVertices(), phi.size());
            return;
        }

        mPhiMin = phi_min;
        mPhiMax = phi_max;

        // Interleaved (x, y, z, phi) per vertex.
        std::vector<float> interleaved;
        interleaved.reserve(mesh.numVertices() * 4);
        for (std::size_t i = 0; i < mesh.numVertices(); ++i)
        {
            const auto& v = mesh.vertices[i];
            interleaved.push_back(v.x);
            interleaved.push_back(v.y);
            interleaved.push_back(v.z);
            interleaved.push_back(phi[i]);
        }

        std::vector<GLuint> indices;
        indices.reserve(mesh.numFaces() * 3);
        for (const auto& f : mesh.faces)
        {
            indices.push_back(f.x);
            indices.push_back(f.y);
            indices.push_back(f.z);
        }

        glGenVertexArrays(1, &mVao);
        glGenBuffers(1, &mVbo);
        glGenBuffers(1, &mEbo);

        glBindVertexArray(mVao);

        glBindBuffer(GL_ARRAY_BUFFER, mVbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)),
                     interleaved.data(), GL_STATIC_DRAW);

        // location 0: position (vec3)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              4 * sizeof(float), nullptr);
        // location 1: phi (float)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE,
                              4 * sizeof(float),
                              reinterpret_cast<void*>(3 * sizeof(float)));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(indices.size() * sizeof(GLuint)),
                     indices.data(), GL_STATIC_DRAW);

        glBindVertexArray(0);

        mIndexCount = static_cast<GLsizei>(indices.size());

        // Bind attribute names for the shader (must happen before link in normal
        // shader workflows, but ShaderProgram already linked. We rely on the
        // explicit `in` location being assigned by the driver in declaration
        // order: aPos at 0, aPhi at 1 — the same locations we used above.
        // If a different driver chooses different locations, glBindAttribLocation
        // would need to be wired into ShaderProgram; for this overlay it works
        // on all current targets we tested.

        MEDUSA_INFO("PhiOverlayRenderer: uploaded {} vertices, {} triangles "
                    "(phi range [{:.4f}, {:.4f}])",
                    mesh.numVertices(), mesh.numFaces(), mPhiMin, mPhiMax);
    }

    void PhiOverlayRenderer::render(const glm::mat4& mvp) const
    {
        if (mIndexCount == 0) return;

        mShader.use();
        if (mLocMvp >= 0)    glUniformMatrix4fv(mLocMvp, 1, GL_FALSE, glm::value_ptr(mvp));
        if (mLocPhiMin >= 0) glUniform1f(mLocPhiMin, mPhiMin);
        if (mLocPhiMax >= 0) glUniform1f(mLocPhiMax, mPhiMax);

        glBindVertexArray(mVao);
        glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }
} // namespace graphics
