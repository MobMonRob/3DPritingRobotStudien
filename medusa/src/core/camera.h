/**
 * @file camera.h
 * @brief Minimal orbit camera that provides projection and view matrices.
 */

#ifndef MEDUSA_SRC_CORE_CAMERA_H_
#define MEDUSA_SRC_CORE_CAMERA_H_

#include <glm/glm.hpp>

/**
 * @brief Simple orbit camera around the origin.
 *
 * The camera looks at the origin and moves on a vertical circle.
 */
class Camera
{
public:
    /**
     * @brief Sets the vertical field of view in degrees.
     * @param degrees Field of view in degrees.
     */
    void setFovDegrees(float degrees);

    /**
     * @brief Returns the vertical field of view in degrees.
     * @return Field of view in degrees.
     */
    [[nodiscard]] float getFovDegrees() const;

    /**
     * @brief Sets the distance to the origin.
     * @param distance Distance in world units.
     */
    void setDistance(float distance);

    /**
     * @brief Returns the distance to the origin.
     * @return Distance in world units.
     */
    [[nodiscard]] float getDistance() const;

    /**
     * @brief Sets the elevation angle in degrees.
     * @param degrees Elevation in degrees.
     */
    void setElevationDegrees(float degrees);

    /**
     * @brief Returns the elevation angle in degrees.
     * @return Elevation in degrees.
     */
    [[nodiscard]] float getElevationDegrees() const;

    /**
     * @brief Sets the azimuth angle in degrees.
     * @param degrees Azimuth in degrees.
     */
    void setAzimuthDegrees(float degrees);

    /**
     * @brief Returns the azimuth angle in degrees.
     * @return Azimuth in degrees.
     */
    [[nodiscard]] float getAzimuthDegrees() const;

    /**
     * @brief Sets the near and far clipping planes.
     * @param near Near clipping plane distance.
     * @param far Far clipping plane distance.
     */
    void setClipPlanes(float near, float far);

    /**
     * @brief Returns the near clipping plane distance.
     * @return Near plane distance.
     */
    [[nodiscard]] float getNearPlane() const;

    /**
     * @brief Returns the far clipping plane distance.
     * @return Far plane distance.
     */
    [[nodiscard]] float getFarPlane() const;

    /**
     * @brief Resets all camera parameters to default values.
     */
    void reset();

    /**
     * @brief Builds a perspective projection matrix.
     * @param aspect Aspect ratio (width / height).
     * @return Perspective projection matrix.
     */
    [[nodiscard]] glm::mat4 proj(float aspect) const;

    /**
     * @brief Builds the view matrix (lookAt) for the current orbit position.
     * @return View matrix.
     */
    [[nodiscard]] glm::mat4 view() const;

private:
    float mFovDegrees{60.0f}; ///< Vertical field of view in degrees.
    float mDistance{4.0f}; ///< Distance to the origin.
    float mElevationDegrees{30.0f}; ///< Elevation angle in degrees.
    float mAzimuthDegrees{0.0f}; ///< Azimuth angle in degrees.
    float mNearPlane{0.1f}; ///< Near clipping plane distance.
    float mFarPlane{100.0f}; ///< Far clipping plane distance.
};

#endif  // MEDUSA_SRC_CORE_CAMERA_H_