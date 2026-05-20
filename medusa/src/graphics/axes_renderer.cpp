/**
 * @file axes_renderer.cpp
 * @brief Implementation of @ref AxesRenderer.
 */
#include <glm/gtc/type_ptr.hpp>

#include "axes_renderer.h"
#include "camera.h"
#include "logger.h"

static auto kAxesVsSrc = R"GLSL(
    #version 150
    in vec3 aPos;
    in vec3 aColor;
    uniform mat4 uMVP;
    out vec3 vColor;
    void main() {
        vColor = aColor;
        gl_Position = uMVP * vec4(aPos, 1.0);
    }
)GLSL";

static auto kAxesFsSrc = R"GLSL(
    #version 150
    in vec3 vColor;
    out vec4 FragColor;
    void main() {
        FragColor = vec4(vColor, 1.0);
    }
)GLSL";

AxesRenderer::~AxesRenderer()
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

bool AxesRenderer::initialize()
{
    MEDUSA_INFO("Initializing AxesRenderer");

    if (!mShaderProgram.create(kAxesVsSrc, kAxesFsSrc))
    {
        MEDUSA_CRITICAL("AxesRenderer shader program creation failed");
        return false;
    }

    mMvpUniformLocation = mShaderProgram.loc("uMVP");
    if (mMvpUniformLocation < 0)
    {
        MEDUSA_WARN("AxesRenderer: uMVP uniform location invalid");
    }

    // Axis length
    constexpr float kLen = 5.0f;

    // Vertex data: position (3 floats) + color (3 floats)
    // clang-format off
    constexpr float kVertices[] = {
        // X-axis (Red)
        0.0f, 0.0f, 0.0f,   1.0f, 0.2f, 0.2f,
        kLen, 0.0f, 0.0f,   1.0f, 0.2f, 0.2f,
        // Y-axis (Green)
        0.0f, 0.0f, 0.0f,   0.2f, 1.0f, 0.2f,
        0.0f, kLen, 0.0f,   0.2f, 1.0f, 0.2f,
        // Z-axis (Blue)
        0.0f, 0.0f, 0.0f,   0.2f, 0.4f, 1.0f,
        0.0f, 0.0f, kLen,   0.2f, 0.4f, 1.0f,
    };
    // clang-format on

    glGenVertexArrays(1, &mVao);
    glGenBuffers(1, &mVbo);

    glBindVertexArray(mVao);
    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kVertices), kVertices, GL_STATIC_DRAW);

    // Position attribute (location 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);

    // Color attribute (location 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);

    MEDUSA_DEBUG("AxesRenderer initialized: VAO={}, VBO={}", mVao, mVbo);
    return true;
}

void AxesRenderer::render(const Camera& camera, const glm::mat4& modelMatrix, const float aspect) const
{
    if (mVao == 0)
    {
        return;
    }

    const glm::mat4 kProj = camera.proj(aspect);
    const glm::mat4 kView = camera.view();
    const glm::mat4 kMvp = kProj * kView * modelMatrix;

    mShaderProgram.use();
    glUniformMatrix4fv(mMvpUniformLocation, 1, GL_FALSE, glm::value_ptr(kMvp));

    glBindVertexArray(mVao);
    glLineWidth(2.5f);
    glDrawArrays(GL_LINES, 0, kLineVertexCount);
    glBindVertexArray(0);
}