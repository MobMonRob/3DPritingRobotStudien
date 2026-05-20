#pragma once

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>

#include "shader_program.h"
#include "mesh.h"

class Camera;

/**
 * @file scene_renderer.h
 * @brief Encapsulates 3D scene rendering (shader setup, uniform handling, and draw calls).
 */

/**
 * @brief Renders a mesh using a simple Lambert-style shader.
 *
 * @details This class owns the OpenGL shader program and caches uniform locations.
 * The application provides the model matrix, while this renderer computes MVP and
 * submits draw calls.
 */
class SceneRenderer
{
public:
    /**
     * @brief Initializes GPU resources used by the renderer.
     *
     * @details Builds the internal shader program and caches uniform locations.
     *
     * @return True on success.
     */
    [[nodiscard]] bool initialize();

    /**
     * @brief Renders the scene into the current framebuffer.
     *
     * @details
     * - Sets viewport and clears color/depth.
     * - If @p mesh is empty (VAO == 0 or count == 0), only the background is drawn.
     *
     * @param window GLFW window used to query framebuffer size.
     * @param camera Camera providing projection and view matrices.
     * @param mesh Mesh to draw.
     * @param modelMatrix Model transform (provided by the application).
     * @param renderWireframe When true, draws using GL_LINE polygon mode.
     *
     * @note The normal matrix is derived as mat3(modelMatrix) which is correct for
     *       uniform scaling. If you introduce non-uniform scaling, use inverse-transpose.
     */
    void render(GLFWwindow* window, const Camera& camera, const graphics::Mesh& mesh, const glm::mat4& modelMatrix,
                bool renderWireframe) const;

private:
    /** @brief Shader program used for scene rendering. */
    ShaderProgram mShaderProgram;

    /** @brief Cached uniform location for the MVP matrix. */
    GLint mMvpUniformLocation{-1};

    /** @brief Cached uniform location for the normal matrix. */
    GLint mNormalMatrixUniformLocation{-1};

    /** @brief Cached uniform location for the wireframe mode flag. */
    GLint mWireframeUniformLocation{-1};
};