#include <gtest/gtest.h>

#include "camera.h"

namespace
{
    constexpr float kEpsilon = 1e-4f;
}

TEST(Camera, DefaultsAfterReset)
{
    Camera camera;

    camera.setFovDegrees(90.0f);
    camera.setDistance(10.0f);
    camera.setElevationDegrees(-10.0f);
    camera.setAzimuthDegrees(45.0f);
    camera.setClipPlanes(0.5f, 200.0f);

    camera.reset();

    EXPECT_FLOAT_EQ(camera.getFovDegrees(), 60.0f);
    EXPECT_FLOAT_EQ(camera.getDistance(), 4.0f);
    EXPECT_FLOAT_EQ(camera.getElevationDegrees(), 30.0f);
    EXPECT_FLOAT_EQ(camera.getAzimuthDegrees(), 0.0f);
    EXPECT_FLOAT_EQ(camera.getNearPlane(), 0.1f);
    EXPECT_FLOAT_EQ(camera.getFarPlane(), 100.0f);
}

TEST(Camera, FovIsClamped)
{
    Camera camera;

    camera.setFovDegrees(0.0f);
    EXPECT_FLOAT_EQ(camera.getFovDegrees(), 1.0f);

    camera.setFovDegrees(200.0f);
    EXPECT_FLOAT_EQ(camera.getFovDegrees(), 179.0f);
}

TEST(Camera, DistanceIsClamped)
{
    Camera camera;

    camera.setDistance(-5.0f);
    EXPECT_FLOAT_EQ(camera.getDistance(), 0.5f);

    camera.setDistance(100.0f);
    EXPECT_FLOAT_EQ(camera.getDistance(), 50.0f);
}

TEST(Camera, ElevationIsClamped)
{
    Camera camera;

    camera.setElevationDegrees(-120.0f);
    EXPECT_FLOAT_EQ(camera.getElevationDegrees(), -89.0f);

    camera.setElevationDegrees(120.0f);
    EXPECT_FLOAT_EQ(camera.getElevationDegrees(), 89.0f);
}

TEST(Camera, AzimuthIsNotClamped)
{
    Camera camera;

    camera.setAzimuthDegrees(720.0f);

    EXPECT_NEAR(camera.getAzimuthDegrees(), 720.0f, kEpsilon);
}

TEST(Camera, ClipPlanesRejectInvalidInput)
{
    Camera camera;

    camera.setClipPlanes(0.1f, 100.0f);

    camera.setClipPlanes(-1.0f, 10.0f);
    EXPECT_FLOAT_EQ(camera.getNearPlane(), 0.1f);
    EXPECT_FLOAT_EQ(camera.getFarPlane(), 100.0f);

    camera.setClipPlanes(5.0f, 2.0f);
    EXPECT_FLOAT_EQ(camera.getNearPlane(), 0.1f);
    EXPECT_FLOAT_EQ(camera.getFarPlane(), 100.0f);
}

