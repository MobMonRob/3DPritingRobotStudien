#pragma once

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>

#include "shader_program.h"

class Camera;

/**
 * @file grid_renderer.h
 * @brief Renders a grid in the XZ plane for spatial reference.
 */

/**
 * @brief Renders a ground grid in the XZ plane.
 *
 * @details Draws a square grid centered at the origin with subtle gray lines.
 * The grid helps with spatial orientation and scale perception.
 */
class GridRenderer
{
public:
    GridRenderer() = default;
    ~GridRenderer();

    GridRenderer(const GridRenderer&) = delete;
    GridRenderer& operator=(const GridRenderer&) = delete;
    GridRenderer(GridRenderer&&) = delete;
    GridRenderer& operator=(GridRenderer&&) = delete;

    /**
     * @brief Initializes GPU resources (shaders, VAO/VBO).
     * @return True on success.
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Renders the grid.
     *
     * @param camera Camera providing projection and view matrices.
     * @param modelMatrix Model transform to apply to the grid.
     * @param aspect Viewport aspect ratio.
     */
    void render(const Camera& camera, const glm::mat4& modelMatrix, float aspect) const;

private:
    ShaderProgram mShaderProgram;
    GLuint mVao{0};
    GLuint mVbo{0};
    GLint mMvpUniformLocation{-1};
    GLsizei mLineCount{0};
};