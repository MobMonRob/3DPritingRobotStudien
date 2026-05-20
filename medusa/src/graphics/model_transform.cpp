/**
 * @file model_transform.cpp
 * @brief Implementation of ModelTransform.
 */

#include "model_transform.h"
#include "logger.h"
#include "mesh.h"

#include <glm/gtc/matrix_transform.hpp>

void ModelTransform::resetManualRotation()
{
    mManualYawRad = 0.0f;
    mManualPitchRad = 0.0f;
    MEDUSA_INFO("Model rotation reset (manual yaw/pitch set to 0)");
}

void ModelTransform::setManualRotation(const float yawRad, const float pitchRad)
{
    mManualYawRad = yawRad;
    mManualPitchRad = pitchRad;
    MEDUSA_TRACE("ModelTransform: yaw={:.3f} rad, pitch={:.3f} rad", yawRad, pitchRad);
}

glm::mat4 ModelTransform::computeModelMatrix(const float elapsedSeconds, const graphics::Mesh& mesh) const
{
    glm::mat4 model(1.0f);

    // Apply rotation first (model rotates around world origin)
    if (isAutoRotateEnabled)
    {
        model = glm::rotate(model, elapsedSeconds * autoRotateSpeedRadPerSec, glm::vec3(0, 1, 0));
    }
    else
    {
        model = glm::rotate(model, mManualPitchRad, glm::vec3(1, 0, 0));
        model = glm::rotate(model, mManualYawRad, glm::vec3(0, 1, 0));
    }

    const float kSafeRadius = (mesh.radius > 0.0f) ? mesh.radius : 1.0f;
    if (mesh.radius <= 0.0f)
    {
        MEDUSA_WARN("Mesh radius is <= 0 ({}). Falling back to radius=1 for normalization", mesh.radius);
    }

    // Scale and center the mesh so it sits on the ground plane (Y=0).
    // X/Z: centered on world origin so Y-axis rotation looks correct.
    // Y:   bottom of the AABB placed at Y=0 (not the center), so the object
    //      rests on the grid and never goes below it.
    // Matrix multiplication is right-to-left: Rotation * Scale * Translation
    model = glm::scale(model, glm::vec3(1.5f / kSafeRadius));
    model = glm::translate(model, glm::vec3(-mesh.center.x, -mesh.boundsMin.y, -mesh.center.z));
    return model;
}

glm::mat4 ModelTransform::computeRotationMatrix(const float elapsedSeconds) const
{
    glm::mat4 rotation(1.0f);

    if (isAutoRotateEnabled)
    {
        rotation = glm::rotate(rotation, elapsedSeconds * autoRotateSpeedRadPerSec, glm::vec3(0, 1, 0));
    }
    else
    {
        rotation = glm::rotate(rotation, mManualPitchRad, glm::vec3(1, 0, 0));
        rotation = glm::rotate(rotation, mManualYawRad, glm::vec3(0, 1, 0));
    }

    return rotation;
}