#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "camera.h"

TEST(CameraTest, DefaultValues) {
    Camera cam;
    EXPECT_EQ(cam.getFovDegrees(), 60.0f);
    EXPECT_EQ(cam.getDistance(), 4.0f);
    EXPECT_EQ(cam.getElevationDegrees(), 30.0f);
    EXPECT_EQ(cam.getAzimuthDegrees(), 0.0f);
}
