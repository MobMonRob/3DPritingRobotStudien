/**
 * @file camera_controller.h
 * @brief Input controller for camera manipulation.
 */

#ifndef MEDUSA_SRC_CONTROLS_CAMERA_CONTROLLER_H_
#define MEDUSA_SRC_CONTROLS_CAMERA_CONTROLLER_H_

#include <GLFW/glfw3.h>
#include "camera.h"

/**
 * @brief Handles input for camera control (scroll for zoom, right mouse drag for rotation).
 *
 * This class separates input handling from the Camera class, allowing Camera
 * to remain a pure state/computation class.
 */
class CameraController
{
public:
    /**
     * @brief Updates camera rotation from right mouse button drag.
     *
     * @param window GLFW window used for cursor position and button state.
     * @param isUiCapturingMouse When true, drag input is ignored (e.g. ImGui is interacting).
     */
    void updateFromMouseDrag(GLFWwindow* window, bool isUiCapturingMouse);

    /**
     * @brief Updates camera distance (zoom) from scroll wheel input.
     *
     * @param yOffset Scroll wheel delta (positive = zoom in, negative = zoom out).
     */
    void updateFromScroll(double yOffset);

    /**
     * @brief Resets camera to default position and orientation.
     */
    void reset();

    /**
     * @brief Returns a reference to the underlying camera.
     * @return Camera reference.
     */
    Camera& getCamera() { return mCamera; }

    /**
     * @brief Returns a const reference to the underlying camera.
     * @return Const camera reference.
     */
    [[nodiscard]] const Camera& getCamera() const { return mCamera; }

private:
    Camera mCamera; ///< The camera being controlled.
    bool mIsDragging{false}; ///< True while dragging with right mouse button.
    double mPreviousMouseX{0.0}; ///< Last mouse X position.
    double mPreviousMouseY{0.0}; ///< Last mouse Y position.
    float mZoomSensitivity{0.5f}; ///< Zoom sensitivity factor.
    float mRotationSensitivity{0.2f}; ///< Rotation sensitivity (degrees per pixel).
};

#endif  // MEDUSA_SRC_CONTROLS_CAMERA_CONTROLLER_H_