/**
 * @file camera_controller.cpp
 * @brief Implementation of CameraController.
 */

#include "camera_controller.h"
#include "logger.h"

void CameraController::updateFromMouseDrag(GLFWwindow* window, const bool isUiCapturingMouse)
{
    if (!window)
    {
        MEDUSA_WARN("CameraController::updateFromMouseDrag called with null window");
        return;
    }

    const int kRightMouseButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    const bool kIsRightMouseDown = kRightMouseButtonState == GLFW_PRESS;
    const bool kIsRightMouseUp = kRightMouseButtonState == GLFW_RELEASE;
    double mouseX = 0.0;
    double mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (kIsRightMouseDown && !isUiCapturingMouse)
    {
        if (!mIsDragging)
        {
            mIsDragging = true;
            mPreviousMouseX = mouseX;
            mPreviousMouseY = mouseY;
            MEDUSA_DEBUG("Camera drag started");
        }

        const double kDeltaX = mouseX - mPreviousMouseX;
        const double kDeltaY = mouseY - mPreviousMouseY;
        mPreviousMouseX = mouseX;
        mPreviousMouseY = mouseY;

        // Update azimuth (horizontal rotation)
        const float kAzimuthDelta = static_cast<float>(kDeltaX) * mRotationSensitivity;
        mCamera.setAzimuthDegrees(mCamera.getAzimuthDegrees() + kAzimuthDelta);

        // Update elevation (vertical rotation)
        const float kElevationDelta = static_cast<float>(-kDeltaY) * mRotationSensitivity;
        mCamera.setElevationDegrees(mCamera.getElevationDegrees() + kElevationDelta);
        return;
    }

    if (mIsDragging)
    {
        if (isUiCapturingMouse)
        {
            if (kIsRightMouseDown)
                MEDUSA_DEBUG("Camera drag stopped (UI captured mouse)");
        }
        else
        {
            if (kIsRightMouseUp)
                MEDUSA_DEBUG("Camera drag stopped");
        }
    }

    mIsDragging = false;
}

void CameraController::updateFromScroll(const double yOffset)
{
    const float kCurrentDistance = mCamera.getDistance();
    const float kNewDistance = kCurrentDistance - static_cast<float>(yOffset) * mZoomSensitivity;
    MEDUSA_TRACE("CameraController: scroll offset={:.2f}, distance {:.3f} -> {:.3f}",
                 yOffset, kCurrentDistance, kNewDistance);
    mCamera.setDistance(kNewDistance);
}

void CameraController::reset()
{
    mCamera.reset();
    mIsDragging = false;
    MEDUSA_INFO("Camera controller reset");
}