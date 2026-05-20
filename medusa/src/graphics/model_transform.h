/**
 * @file model_transform.h
 * @brief Tracks model rotation interaction state and computes per-frame model matrices.
 */

#ifndef MEDUSA_SRC_GRAPHICS_MODEL_TRANSFORM_H_
#define MEDUSA_SRC_GRAPHICS_MODEL_TRANSFORM_H_

#include <glm/glm.hpp>

namespace graphics
{
    class Mesh;
}

/**
 * @brief Computes the model matrix used for rendering a mesh.
 *
 * In auto-rotate mode, the model rotates around the Y axis over time.
 * In manual mode, yaw/pitch are updated from mouse dragging.
 * The mesh is scaled and centered to fit the view (based on its radius/center).
 *
 * All angles are expressed in radians.
 */
class ModelTransform
{
public:
    /** Enables continuous automatic rotation over time. */
    bool isAutoRotateEnabled{true};

    /** Autorotation speed in radians per second. */
    float autoRotateSpeedRadPerSec{0.6f};

    /**
     * @brief Resets manual yaw/pitch to zero.
     */
    void resetManualRotation();

    /**
     * @brief Sets manual rotation angles.
     *
     * @param yawRad Yaw angle in radians.
     * @param pitchRad Pitch angle in radians (should be pre-clamped by caller).
     */
    void setManualRotation(float yawRad, float pitchRad);

    /**
     * @brief Returns the manual yaw angle.
     * @return Manual yaw in radians.
     */
    [[nodiscard]] float getManualYawRad() const { return mManualYawRad; }

    /**
     * @brief Returns the manual pitch angle.
     * @return Manual pitch in radians.
     */
    [[nodiscard]] float getManualPitchRad() const { return mManualPitchRad; }

    /**
     * @brief Computes the model matrix for the current frame.
     *
     * @param elapsed_seconds Seconds since application start.
     * @param mesh Mesh providing center and radius.
     * @return Model matrix.
     */
    [[nodiscard]] glm::mat4 computeModelMatrix(float elapsed_seconds, const graphics::Mesh& mesh) const;

    /**
     * @brief Computes a rotation-only matrix (for axes rendering).
     *
     * @param elapsed_seconds Seconds since application start.
     * @return Rotation matrix without mesh-specific translation/scale.
     */
    [[nodiscard]] glm::mat4 computeRotationMatrix(float elapsed_seconds) const;

private:
    /** Manual yaw angle in radians. */
    float mManualYawRad{0.0f};

    /** Manual pitch angle in radians. */
    float mManualPitchRad{0.0f};
};

#endif  // MEDUSA_SRC_GRAPHICS_MODEL_TRANSFORM_H_