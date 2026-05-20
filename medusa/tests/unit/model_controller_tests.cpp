#include <gtest/gtest.h>

#include "model_controller.h"

TEST(ModelController, DragIgnoredWhenAutoRotateEnabled)
{
    ModelController controller;

    controller.getTransform().isAutoRotateEnabled = true;
    controller.getTransform().setManualRotation(0.2f, 0.3f);

    controller.updateFromMouseDrag(nullptr, false);

    EXPECT_FLOAT_EQ(controller.getTransform().getManualYawRad(), 0.2f);
    EXPECT_FLOAT_EQ(controller.getTransform().getManualPitchRad(), 0.3f);
}

TEST(ModelController, NullWindowDoesNotChangeRotation)
{
    ModelController controller;

    controller.getTransform().isAutoRotateEnabled = false;
    controller.getTransform().setManualRotation(0.4f, -0.2f);

    controller.updateFromMouseDrag(nullptr, false);

    EXPECT_FLOAT_EQ(controller.getTransform().getManualYawRad(), 0.4f);
    EXPECT_FLOAT_EQ(controller.getTransform().getManualPitchRad(), -0.2f);
}
