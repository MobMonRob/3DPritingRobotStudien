#include <gtest/gtest.h>

#include "camera_controller.h"

namespace
{
    constexpr float kDefaultFovDegrees = 60.0f;
    constexpr float kDefaultDistance = 4.0f;
    constexpr float kDefaultElevationDegrees = 30.0f;
    constexpr float kDefaultAzimuthDegrees = 0.0f;
    constexpr float kDefaultNearPlane = 0.1f;
    constexpr float kDefaultFarPlane = 100.0f;
    constexpr float kZoomSensitivity = 0.5f;
} // namespace

TEST(CameraController, ScrollAdjustsDistance)
{
    CameraController controller;

    const float before = controller.getCamera().getDistance();
    controller.updateFromScroll(1.0);

    EXPECT_FLOAT_EQ(controller.getCamera().getDistance(), before - kZoomSensitivity);
}

TEST(CameraController, ScrollClampsToMinDistance)
{
    CameraController controller;

    controller.updateFromScroll(20.0);

    EXPECT_FLOAT_EQ(controller.getCamera().getDistance(), 0.5f);
}

TEST(CameraController, ResetRestoresCameraDefaults)
{
    CameraController controller;

    controller.getCamera().setFovDegrees(80.0f);
    controller.getCamera().setDistance(12.0f);
    controller.getCamera().setElevationDegrees(-15.0f);
    controller.getCamera().setAzimuthDegrees(90.0f);
    controller.getCamera().setClipPlanes(0.2f, 200.0f);

    controller.reset();

    const Camera& camera = controller.getCamera();
    EXPECT_FLOAT_EQ(camera.getFovDegrees(), kDefaultFovDegrees);
    EXPECT_FLOAT_EQ(camera.getDistance(), kDefaultDistance);
    EXPECT_FLOAT_EQ(camera.getElevationDegrees(), kDefaultElevationDegrees);
    EXPECT_FLOAT_EQ(camera.getAzimuthDegrees(), kDefaultAzimuthDegrees);
    EXPECT_FLOAT_EQ(camera.getNearPlane(), kDefaultNearPlane);
    EXPECT_FLOAT_EQ(camera.getFarPlane(), kDefaultFarPlane);
}

TEST(CameraController, MouseDragWithNullWindowDoesNotChangeCamera)
{
    CameraController controller;

    controller.getCamera().setAzimuthDegrees(10.0f);
    controller.getCamera().setElevationDegrees(15.0f);

    controller.updateFromMouseDrag(nullptr, false);

    EXPECT_FLOAT_EQ(controller.getCamera().getAzimuthDegrees(), 10.0f);
    EXPECT_FLOAT_EQ(controller.getCamera().getElevationDegrees(), 15.0f);
}

