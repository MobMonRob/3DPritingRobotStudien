/**
 * @file model_controller.cpp
 * @brief Implementation of ModelController.
 */

#include "model_controller.h"
#include "logger.h"

#include <algorithm>

void ModelController::updateFromMouseDrag(GLFWwindow* window, const bool isUiCapturingMouse)
{
    if (mTransform.isAutoRotateEnabled)
        return;

    if (!window)
    {
        MEDUSA_WARN("ModelController::updateFromMouseDrag called with null window");
        return;
    }

    const int kLeftMouseButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    const bool kIsLeftMouseDown = kLeftMouseButtonState == GLFW_PRESS;
    const bool kIsLeftMouseUp = kLeftMouseButtonState == GLFW_RELEASE;
    double mouseX = 0.0;
    double mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (kIsLeftMouseDown && !isUiCapturingMouse)
    {
        if (!mIsDragging)
        {
            mIsDragging = true;
            mPreviousMouseX = mouseX;
            mPreviousMouseY = mouseY;
            MEDUSA_DEBUG("Model drag started");
        }

        const double kDeltaX = mouseX - mPreviousMouseX;
        const double kDeltaY = mouseY - mPreviousMouseY;
        mPreviousMouseX = mouseX;
        mPreviousMouseY = mouseY;

        constexpr float kSensitivity = 0.005f;
        float newYaw = mTransform.getManualYawRad() + static_cast<float>(kDeltaX) * kSensitivity;
        float newPitch = mTransform.getManualPitchRad() + static_cast<float>(kDeltaY) * kSensitivity;

        // Clamp pitch to prevent gimbal lock
        constexpr float kPitchLimitRad = 1.5f;
        const float kPreviousPitch = newPitch;
        newPitch = std::clamp(newPitch, -kPitchLimitRad, kPitchLimitRad);

        if (newPitch != kPreviousPitch)
        {
            MEDUSA_DEBUG("Model pitch clamped to {} rad", newPitch);
        }

        mTransform.setManualRotation(newYaw, newPitch);
        return;
    }

    if (mIsDragging)
    {
        if (isUiCapturingMouse)
        {
            if (kIsLeftMouseDown)
                MEDUSA_DEBUG("Model drag stopped (UI captured mouse)");
        }
        else
        {
            if (kIsLeftMouseUp)
                MEDUSA_DEBUG("Model drag stopped");
        }
    }

    mIsDragging = false;
}