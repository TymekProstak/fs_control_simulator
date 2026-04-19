#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/common/BolideDescriptor.h"
#include "../include/common/config.h"

TEST(PathPlanningBolideDescriptorTest, Initialization)
{
    BolideDescriptor bolide(Vec2(), 0);

    ASSERT_EQ(0.0, bolide.getTheta());
    ASSERT_EQ(0.0, bolide.getPosition().x);
    ASSERT_EQ(0.0, bolide.getPosition().y);

    bolide.setCoordinates(Vec2(3, -3), 1);
    ASSERT_EQ(1, bolide.getTheta());
    ASSERT_EQ(3, bolide.getPosition().x);
    ASSERT_EQ(-3, bolide.getPosition().y);
}

TEST(PathPlanningBolideDescriptorTest, ConvertToRange)
{
    BolideDescriptor bolide(Vec2(), 0);
    // theta < 0
    bolide.setCoordinates(Vec2(), -M_PI);
    ASSERT_FLOAT_EQ(M_PI, bolide.getTheta());

    bolide.setCoordinates(Vec2(), -M_PI * 2);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    bolide.setCoordinates(Vec2(), -M_PI * 6 + EPS);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    // theta >= 2 * M_PI - EPS
    bolide.setCoordinates(Vec2(-1, 9), M_PI * 2);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    bolide.setCoordinates(Vec2(1, 1), -M_PI * 8 - EPS);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    bolide.setCoordinates(Vec2(1, 1), M_PI * 4);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    bolide.setCoordinates(Vec2(1, 1), EPS * 0.3);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    bolide.setCoordinates(Vec2(1, 1), EPS * 0.3 + 2 * M_PI);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());

    // 0 < theta < EPS
    bolide.setCoordinates(Vec2(1, 1), EPS * 0.3);
    ASSERT_FLOAT_EQ(0.0f, bolide.getTheta());
}

TEST(PathPlanningBolideDescriptorTest, ComputeSourcePoints)
{
    BolideDescriptor bolide(Vec2(), 0);
    bolide.setCoordinates(Vec2(1, 1), 0);

    ASSERT_FLOAT_EQ(bolide.getMiddleSourcePoint().x, 1 - BOLIDE_LENGTH_TILL_END);
    ASSERT_FLOAT_EQ(bolide.getMiddleSourcePoint().y, 1);

    ASSERT_FLOAT_EQ(bolide.getLeftSourcePoint().x, 1 - BOLIDE_LENGTH_TILL_END);
    ASSERT_FLOAT_EQ(bolide.getRightSourcePoint().x, 1 - BOLIDE_LENGTH_TILL_END);
    ASSERT_FLOAT_EQ(bolide.getLeftSourcePoint().y, 1 + BOLIDE_WIDTH / 2);
    ASSERT_FLOAT_EQ(bolide.getRightSourcePoint().y, 1 - BOLIDE_WIDTH / 2);

    bolide.setCoordinates(Vec2(1, 1), M_PI_2);

    ASSERT_FLOAT_EQ(bolide.getMiddleSourcePoint().x, 1);
    ASSERT_FLOAT_EQ(bolide.getMiddleSourcePoint().y, 1 - BOLIDE_LENGTH_TILL_END);

    ASSERT_FLOAT_EQ(bolide.getLeftSourcePoint().y, 1 - BOLIDE_LENGTH_TILL_END);
    ASSERT_FLOAT_EQ(bolide.getRightSourcePoint().y, 1 - BOLIDE_LENGTH_TILL_END);
    ASSERT_FLOAT_EQ(bolide.getLeftSourcePoint().x, 1 - BOLIDE_WIDTH / 2);
    ASSERT_FLOAT_EQ(bolide.getRightSourcePoint().x, 1 + BOLIDE_WIDTH / 2);
}