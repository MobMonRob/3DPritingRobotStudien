#ifndef MEDUSA_SRC_UI_IM_GUI_LAYER_H_
#define MEDUSA_SRC_UI_IM_GUI_LAYER_H_

#include <GLFW/glfw3.h>

/**
 * @file im_gui_layer.h
 * @brief ImGuiLayer encapsulates Dear ImGui context management and integration with GLFW/OpenGL.
 */

/**
 * @class ImGuiLayer
 * @brief Manages Dear ImGui context, frame lifecycle, and dockspace host window.
 *
 * - Creates and destroys the ImGui context.
 * - Initializes the GLFW and OpenGL3 backends.
 * - Provides a per-frame API (begin/end).
 * - Hosts a full-screen dockspace window.
 */
class ImGuiLayer
{
public:
    /**
     * @brief Initializes Dear ImGui for the given GLFW window.
     *
     * Enables keyboard navigation, docking, and multi-viewport support.
     * @param window Active GLFW window with a current OpenGL context.
     * @return True on success, false otherwise.
     */
    [[nodiscard]] bool init(GLFWwindow* window);

    /**
     * @brief Shuts down Dear ImGui and destroys the context.
     */
    void shutdown();

    /**
     * @brief Returns whether ImGui is initialized.
     * @return True if ImGui is initialized, false otherwise.
     */
    [[nodiscard]] bool isReady() const;

    /**
     * @brief Begins a new ImGui frame.
     */
    void beginFrame() const;

    /**
     * @brief Ends the current frame and renders ImGui draw data.
     *
     * If multi-viewport support is enabled, also updates and renders platform windows.
     */
    void endFrameAndRender() const;

    /**
     * @brief Draws the root dockspace host window (full-screen, with dockspace and menu bar).
     */
    static void drawDockspaceHost();

private:
    // True after successful init() and before shutdown().
    bool mReady{false};
};

#endif  // MEDUSA_SRC_UI_IM_GUI_LAYER_H_