#pragma once

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>

#include "shader_program.h"

class Camera;

/**
 * @file axes_renderer.h
 * @brief Renders coordinate axes (X, Y, Z) as colored lines.
 */

/**
 * @brief Renders 3D coordinate axes for orientation reference.
 *
 * @details Draws three lines from the origin:
 * - X-axis: Red
 * - Y-axis: Green
 * - Z-axis: Blue
 */
class AxesRenderer
{
public:
    AxesRenderer() = default;
    ~AxesRenderer();

    AxesRenderer(const AxesRenderer&) = delete;
    AxesRenderer& operator=(const AxesRenderer&) = delete;
    AxesRenderer(AxesRenderer&&) = delete;
    AxesRenderer& operator=(AxesRenderer&&) = delete;

    /**
     * @brief Initializes GPU resources (shaders, VAO/VBO).
     * @return True on success.
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Renders the coordinate axes.
     *
     * @param camera Camera providing projection and view matrices.
     * @param modelMatrix Model transform to apply to the axes.
     * @param aspect Viewport aspect ratio.
     */
    void render(const Camera& camera, const glm::mat4& modelMatrix, float aspect) const;

private:
    ShaderProgram mShaderProgram;
    GLuint mVao{0};
    GLuint mVbo{0};
    GLint mMvpUniformLocation{-1};

    static constexpr GLsizei kLineVertexCount = 6; // 2 vertices per axis, 3 axes
};