/**
 * @file grid_renderer.cpp
 * @brief Implementation of @ref GridRenderer.
 */
#include <vector>
#include <glm/gtc/type_ptr.hpp>

#include "grid_renderer.h"
#include "camera.h"
#include "logger.h"

GridRenderer::~GridRenderer()
{
    if (mVao != 0)
    {
        glDeleteVertexArrays(1, &mVao);
    }
    if (mVbo != 0)
    {
        glDeleteBuffers(1, &mVbo);
    }
}

bool GridRenderer::initialize()
{
    MEDUSA_INFO("Initializing GridRenderer");

    if (!mShaderProgram.createFromFiles(
            MEDUSA_PROJECT_ROOT "/assets/shaders/grid.vert",
            MEDUSA_PROJECT_ROOT "/assets/shaders/grid.frag"))
    {
        MEDUSA_CRITICAL("GridRenderer shader program creation failed");
        return false;
    }

    mMvpUniformLocation = mShaderProgram.loc("uMVP");
    if (mMvpUniformLocation < 0)
    {
        MEDUSA_WARN("GridRenderer: uMVP uniform location invalid");
    }

    // Grid parameters
    constexpr float kGridSize = 5.0f;
    constexpr int kGridLines = 11; // Lines per axis (odd number for center line)
    constexpr float kStep = (2.0f * kGridSize) / static_cast<float>(kGridLines - 1);

    std::vector<float> vertices;
    vertices.reserve(kGridLines * 2 * 2 * 3); // lines * 2 directions * 2 vertices * 3 floats

    // Lines parallel to X-axis (varying Z)
    for (int i = 0; i < kGridLines; ++i)
    {
        const float kZ = -kGridSize + static_cast<float>(i) * kStep;
        // Start point
        vertices.push_back(-kGridSize);
        vertices.push_back(0.0f);
        vertices.push_back(kZ);
        // End point
        vertices.push_back(kGridSize);
        vertices.push_back(0.0f);
        vertices.push_back(kZ);
    }

    // Lines parallel to Z-axis (varying X)
    for (int i = 0; i < kGridLines; ++i)
    {
        const float kX = -kGridSize + static_cast<float>(i) * kStep;
        // Start point
        vertices.push_back(kX);
        vertices.push_back(0.0f);
        vertices.push_back(-kGridSize);
        // End point
        vertices.push_back(kX);
        vertices.push_back(0.0f);
        vertices.push_back(kGridSize);
    }

    mLineCount = static_cast<GLsizei>(vertices.size() / 3);

    glGenVertexArrays(1, &mVao);
    glGenBuffers(1, &mVbo);

    glBindVertexArray(mVao);
    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)), vertices.data(),
                 GL_STATIC_DRAW);

    // Position attribute (location 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    glBindVertexArray(0);

    MEDUSA_DEBUG("GridRenderer initialized: VAO={}, VBO={}, lines={}", mVao, mVbo, mLineCount / 2);
    return true;
}

void GridRenderer::render(const Camera& camera, const glm::mat4& modelMatrix, const float aspect) const
{
    if (mVao == 0 || mLineCount == 0)
    {
        return;
    }

    const glm::mat4 kProj = camera.proj(aspect);
    const glm::mat4 kView = camera.view();
    const glm::mat4 kMvp = kProj * kView * modelMatrix;

    mShaderProgram.use();
    glUniformMatrix4fv(mMvpUniformLocation, 1, GL_FALSE, glm::value_ptr(kMvp));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindVertexArray(mVao);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, mLineCount);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
}