/**
 * @file camera.cpp
 * @brief Implementation of the minimal orbit camera.
 */

#include "camera.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/trigonometric.hpp>
#include "logger.h"

namespace
{
    constexpr float kMinFovDegrees = 1.0f;
    constexpr float kMaxFovDegrees = 179.0f;
    constexpr float kMinDistance = 0.5f;
    constexpr float kMaxDistance = 50.0f;
    constexpr float kMinElevationDegrees = -89.0f;
    constexpr float kMaxElevationDegrees = 89.0f;
    constexpr float kEpsilon = 0.0001f;
}

void Camera::setFovDegrees(float degrees)
{
    const float kClamped = std::clamp(degrees, kMinFovDegrees, kMaxFovDegrees);
    if (std::abs(kClamped - degrees) > kEpsilon)
    {
        MEDUSA_WARN("Camera FOV clamped from {} to {} degrees", degrees, kClamped);
    }
    if (std::abs(kClamped - mFovDegrees) > kEpsilon)
    {
        mFovDegrees = kClamped;
        MEDUSA_INFO("Camera FOV set to {} degrees", mFovDegrees);
    }
}

float Camera::getFovDegrees() const
{
    return mFovDegrees;
}

void Camera::setDistance(float distance)
{
    const float kClamped = std::clamp(distance, kMinDistance, kMaxDistance);
    if (std::abs(kClamped - distance) > kEpsilon)
    {
        MEDUSA_WARN("Camera distance clamped from {} to {}", distance, kClamped);
    }
    if (std::abs(kClamped - mDistance) > kEpsilon)
    {
        mDistance = kClamped;
        MEDUSA_DEBUG("Camera distance set to {}", mDistance);
    }
}

float Camera::getDistance() const
{
    return mDistance;
}

void Camera::setElevationDegrees(float degrees)
{
    const float kClamped = std::clamp(degrees, kMinElevationDegrees, kMaxElevationDegrees);
    if (std::abs(kClamped - degrees) > kEpsilon)
    {
        MEDUSA_WARN("Camera elevation clamped from {} to {} degrees", degrees, kClamped);
    }
    if (std::abs(kClamped - mElevationDegrees) > kEpsilon)
    {
        mElevationDegrees = kClamped;
        MEDUSA_DEBUG("Camera elevation set to {} degrees", mElevationDegrees);
    }
}

float Camera::getElevationDegrees() const
{
    return mElevationDegrees;
}

void Camera::setAzimuthDegrees(const float degrees)
{
    // No clamping for azimuth - it wraps around
    if (std::abs(degrees - mAzimuthDegrees) > kEpsilon)
    {
        mAzimuthDegrees = degrees;
        MEDUSA_DEBUG("Camera azimuth set to {} degrees", mAzimuthDegrees);
    }
}

float Camera::getAzimuthDegrees() const
{
    return mAzimuthDegrees;
}

void Camera::setClipPlanes(float near, float far)
{
    if (near <= 0.0f || far <= near)
    {
        MEDUSA_WARN("Invalid clip planes (near={}, far={}). Must satisfy 0 < near < far", near, far);
        return;
    }
    mNearPlane = near;
    mFarPlane = far;
    MEDUSA_INFO("Clip planes set to near={}, far={}", mNearPlane, mFarPlane);
}

float Camera::getNearPlane() const
{
    return mNearPlane;
}

float Camera::getFarPlane() const
{
    return mFarPlane;
}

void Camera::reset()
{
    mFovDegrees = 60.0f;
    mDistance = 4.0f;
    mElevationDegrees = 30.0f;
    mAzimuthDegrees = 0.0f;
    mNearPlane = 0.1f;
    mFarPlane = 100.0f;
    MEDUSA_INFO("Camera reset to defaults");
}

glm::mat4 Camera::proj(const float aspect) const
{
    return glm::perspective(glm::radians(mFovDegrees), aspect, mNearPlane, mFarPlane);
}

glm::mat4 Camera::view() const
{
    const float kElevation = glm::radians(mElevationDegrees);
    const float kAzimuth = glm::radians(mAzimuthDegrees);
    const float kY = std::sin(kElevation) * mDistance;
    const float kXzRadius = std::cos(kElevation) * mDistance;
    const glm::vec3 kCam(
        std::sin(kAzimuth) * kXzRadius,
        kY,
        std::cos(kAzimuth) * kXzRadius
        );
    return glm::lookAt(kCam, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}