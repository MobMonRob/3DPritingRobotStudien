#ifndef UI_UI_RENDERER_H_
#define UI_UI_RENDERER_H_

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>

#include "shader_program.h"

/**
 * @file ui_renderer.h
 * @brief Minimal immediate-mode UI renderer (non-ImGui) for simple rectangles and buttons.
 */

/**
 * @class UIRenderer
 * @brief Provides rendering of simple 2D UI primitives such as colored rectangles and clickable buttons.
 *
 * This class is intended for custom UI elements that are not covered by ImGui. It manages its own OpenGL resources
 * and provides a minimal interface for drawing interactive UI components directly to the framebuffer.
 */
class UIRenderer
{
public:
    /** @brief Shader program used for UI rendering. */
    ShaderProgram shaderProgram;

    /** @brief VAO/VBO for a 2D rectangle (two floats per vertex). */
    GLuint uiVao{0}, uiVbo{0};

    /** @brief Uniform location for the UI color. */
    GLint colorUniformLocation{-1};

    /** @brief Previous mouse state (edge detection for clicks). */
    bool wasMouseDownLastFrame{false};

    /**
     * @brief Initializes the UI renderer, including shader compilation and GPU buffer allocation.
     *
     * This method must be called before any rendering is performed. It creates the shader program and allocates
     * the necessary OpenGL resources for drawing rectangles and buttons.
     *
     * @return True if initialization was successful, false otherwise.
     */
    bool init();

    /**
     * @brief Releases all GPU resources held by the renderer.
     *
     * This method should be called before application shutdown to avoid resource leaks.
     */
    void shutdown() const;

    /**
     * @brief Draws a clickable button rectangle.
     *
     * Coordinates are in framebuffer pixel space.
     *
     * @param win GLFW window (cursor & framebuffer queries).
     * @param x X position (pixels).
     * @param y Y position (pixels).
     * @param w Width (pixels).
     * @param h Height (pixels).
     * @param offColor Color when "off".
     * @param onColor Color when "on".
     * @param onState Current state.
     * @return True when the button was clicked this frame.
     */
    bool button(GLFWwindow* win, int x, int y, int w, int h, const glm::vec3& offColor, const glm::vec3& onColor,
                bool onState);
};

#endif  // UI_UI_RENDERER_H_