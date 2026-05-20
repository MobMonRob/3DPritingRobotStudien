/**
 * @file model_controller.h
 * @brief Input controller for model transformation.
 */

#ifndef MEDUSA_SRC_CONTROLS_MODEL_CONTROLLER_H_
#define MEDUSA_SRC_CONTROLS_MODEL_CONTROLLER_H_

#include <GLFW/glfw3.h>
#include "model_transform.h"

/**
 * @brief Handles input for model rotation (left mouse button drag).
 *
 * This class separates input handling from the ModelTransform class, allowing
 * ModelTransform to remain a pure transformation computation class.
 */
class ModelController
{
public:
    /**
     * @brief Updates manual rotation from left mouse button drag.
     *
     * When auto-rotation is enabled, this function does nothing.
     *
     * @param window GLFW window used for cursor position and button state.
     * @param isUiCapturingMouse When true, drag input is ignored (e.g. ImGui is interacting).
     */
    void updateFromMouseDrag(GLFWwindow* window, bool isUiCapturingMouse);

    /**
     * @brief Returns a reference to the underlying transform.
     * @return ModelTransform reference.
     */
    ModelTransform& getTransform() { return mTransform; }

    /**
     * @brief Returns a const reference to the underlying transform.
     * @return Const ModelTransform reference.
     */
    [[nodiscard]] const ModelTransform& getTransform() const { return mTransform; }

private:
    ModelTransform mTransform; ///< The model transform being controlled.
    bool mIsDragging{false}; ///< True while dragging with left mouse button.
    double mPreviousMouseX{0.0}; ///< Last mouse X position.
    double mPreviousMouseY{0.0}; ///< Last mouse Y position.
};

#endif  // MEDUSA_SRC_CONTROLS_MODEL_CONTROLLER_H_