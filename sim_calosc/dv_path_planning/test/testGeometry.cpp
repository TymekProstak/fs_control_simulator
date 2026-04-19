#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/common/Geometry.h"
#include "../include/common/config.h"

TEST(PathPlanningGeometryTest, GetAngle)
{
    ASSERT_FLOAT_EQ(geometry::getAngle(Vec2(0, 1), Vec2(0, 1)), 0);
    ASSERT_FLOAT_EQ(geometry::getAngle(Vec2(0, 0), Vec2(1, 0)), 0);

    ASSERT_FLOAT_EQ(geometry::getAngle(Vec2(0, 1), Vec2(0, -1)), -M_PI);
    ASSERT_FLOAT_EQ(geometry::getAngle(Vec2(0, -11), Vec2(0, 3)), M_PI);

    ASSERT_FLOAT_EQ(geometry::getAngle(Vec2(0, 2), Vec2(2, 2)), -M_PI_4);
    ASSERT_FLOAT_EQ(geometry::getAngle(Vec2(2, 0), Vec2(2, 2)), M_PI_4);
}

TEST(PathPlanningGeometryTest, GetDistance)
{
    ASSERT_FLOAT_EQ(geometry::getDistance(Vec2(0, 1), Vec2(0, 1)), 0);
    ASSERT_FLOAT_EQ(geometry::getDistance(Vec2(0, 1), Vec2(0, -1)), 2);
    ASSERT_FLOAT_EQ(geometry::getDistance(Vec2(0, -11), Vec2(0, 3)), 14);
}

TEST(PathPlanningGeometryTest, RealtiveAngle2D)
{
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 1), Vec2(0, 1), 0), 0);             // 1: 0 degrees
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 1), Vec2(0, -1), 0), M_PI * 3 / 2); // 2: 270 degrees
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 0), Vec2(1, 1), 0), M_PI_4);        // 3: 45 degrees

    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 1), Vec2(0, 1), -M_PI_4), M_PI_4);           // 4: 45 deg
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 1), Vec2(0, -1), M_PI_2), M_PI);             // 5: 180 deg
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 0), Vec2(2, 2), M_PI_2), 2 * M_PI - M_PI_4); // 6: 315 deg

    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 0), Vec2(2, -0.00004), 0), 0); // 7: 0 deg

    // examples from logs
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(0, 0), Vec2(3.78, 1.56), 0), 0.39140511);
    ASSERT_FLOAT_EQ(geometry::relativeAngle2D(Vec2(-9.76562e-06, 0.000390625), Vec2(3.78, 1.56), 0), 0.39131591);
}

TEST(PathPlanningGeometryTest, CreateVec2FromAngleLength)
{
    ASSERT_EQ(Vec2(0, 0), geometry::createVec2(0, 0));
    ASSERT_EQ(Vec2(0, 0), geometry::createVec2(0, 0));

    Vec2 deg90 = geometry::createVec2(M_PI_2, 1);
    ASSERT_TRUE(deg90.x < EPS && deg90.x > -EPS);
    ASSERT_TRUE(deg90.y < EPS + 1.0 && deg90.y > -EPS + 1.0);

    Vec2 deg60 = geometry::createVec2(M_PI / 3, 1);
    ASSERT_TRUE(deg60.y < EPS + sqrt(3) / 2 && deg60.y > -EPS + sqrt(3) / 2);
    ASSERT_TRUE(deg60.x < EPS + 0.5 && deg60.x > -EPS + 0.5);

    Vec2 deg30 = geometry::createVec2(M_PI / 6, 1);
    ASSERT_TRUE(deg30.x < EPS + sqrt(3) / 2 && deg30.x > -EPS + sqrt(3) / 2);
    ASSERT_TRUE(deg30.y < EPS + 0.5 && deg30.y > -EPS + 0.5);

    Vec2 deg_minus30 = geometry::createVec2(-M_PI / 6, 1);
    ASSERT_TRUE(deg_minus30.x < EPS + sqrt(3) / 2 && deg_minus30.x > -EPS + sqrt(3) / 2);
    ASSERT_TRUE(deg_minus30.y < EPS + -0.5 && deg_minus30.y > -EPS + -0.5);

    Vec2 deg_minus60 = geometry::createVec2(-M_PI / 3, 1);
    ASSERT_TRUE(deg_minus60.y < EPS - sqrt(3) / 2 && deg_minus60.y > -EPS - sqrt(3) / 2);
    ASSERT_TRUE(deg_minus60.x < EPS + 0.5 && deg_minus60.x > -EPS + 0.5);

    Vec2 deg150 = geometry::createVec2(M_PI * 5 / 6, 1);
    ASSERT_TRUE(deg150.x < EPS - sqrt(3) / 2 && deg150.x > -EPS - sqrt(3) / 2);
    ASSERT_TRUE(deg150.y < EPS + 0.5 && deg150.y > -EPS + 0.5);

    Vec2 deg180 = geometry::createVec2(M_PI, 1);
    ASSERT_TRUE(deg180.x < EPS - 1 && deg180.x > -EPS - 1);
    ASSERT_TRUE(deg180.y < EPS && deg180.y > -EPS);

    Vec2 deg210 = geometry::createVec2(M_PI * 7 / 6, 1);
    ASSERT_TRUE(deg210.x < EPS - sqrt(3) / 2 && deg210.x > -EPS - sqrt(3) / 2);
    ASSERT_TRUE(deg210.y < EPS - 0.5 && deg210.y > -EPS - 0.5);

    // length < 0
    ASSERT_EQ(Vec2(0, 0), geometry::createVec2(0, -1));
    ASSERT_EQ(Vec2(0, 0), geometry::createVec2(0, -3));
}

TEST(PathPlanningGeometryTest, GetDirectedAngle)
{
    ASSERT_FLOAT_EQ(M_PI_2, geometry::getDirectedAngle(Vec2(1, 0), Vec2(0, 0), Vec2(0, 1)));
    ASSERT_FLOAT_EQ(M_PI_2 / 2, geometry::getDirectedAngle(Vec2(1, 0), Vec2(0, 0), Vec2(1, 1)));
    ASSERT_FLOAT_EQ(M_PI_2 / 2, geometry::getDirectedAngle(Vec2(1, 1), Vec2(0, 0), Vec2(0, 1)));
    ASSERT_FLOAT_EQ(M_PI, geometry::getDirectedAngle(Vec2(1, 0), Vec2(0, 0), Vec2(-1, 0)));         // 4
    ASSERT_FLOAT_EQ(M_PI, geometry::getDirectedAngle(Vec2(-1, 0), Vec2(0, 0), Vec2(1, 0)));         // 5
    ASSERT_FLOAT_EQ(M_PI * 3 / 2, geometry::getDirectedAngle(Vec2(1, 0), Vec2(0, 0), Vec2(0, -1))); // 6
    ASSERT_FLOAT_EQ(0, geometry::getDirectedAngle(Vec2(1, 0), Vec2(0, 0), Vec2(1, 0)));             // 7
    ASSERT_FLOAT_EQ(M_PI * 3 / 2, geometry::getDirectedAngle(Vec2(0, 1), Vec2(0, 0), Vec2(1, 0)));
    ASSERT_FLOAT_EQ(M_PI / 2, geometry::getDirectedAngle(Vec2(-1, 0), Vec2(0, 0), Vec2(0, -1)));
    ASSERT_FLOAT_EQ(M_PI + M_PI / 4, geometry::getDirectedAngle(Vec2(-1, 1), Vec2(0, 0), Vec2(1, 0)));  // 10
    ASSERT_FLOAT_EQ(M_PI - M_PI / 4, geometry::getDirectedAngle(Vec2(-1, -1), Vec2(0, 0), Vec2(1, 0))); // 11
    ASSERT_FLOAT_EQ(M_PI / 2, geometry::getDirectedAngle(Vec2(0, 0), Vec2(-1, 1), Vec2(0, 2)));         // 12
    ASSERT_FLOAT_EQ(M_PI / 2, geometry::getDirectedAngle(Vec2(-1, 0), Vec2(-2, 1), Vec2(-1, 2)));       // 13
}

TEST(PathPlanningGeometryTest, GetMiddlePoint)
{
    ASSERT_EQ(geometry::getMiddlePoint(Vec2(0, 1), Vec2(0, 1)), Vec2(0, 1));
    ASSERT_EQ(geometry::getMiddlePoint(Vec2(0, 2), Vec2(0, 1)), Vec2(0, 1.5));
    ASSERT_EQ(geometry::getMiddlePoint(Vec2(1, 0), Vec2(0, 1)), Vec2(0.5, 0.5));
}

TEST(PathPlanningGeometryTest, NormaliseAngle)
{
    ASSERT_FLOAT_EQ(geometry::normaliseAngle(0), 0);
    ASSERT_FLOAT_EQ(geometry::normaliseAngle(-M_PI), -M_PI);
    ASSERT_FLOAT_EQ(geometry::normaliseAngle(-M_PI + EPS * 0.2), -M_PI);
    ASSERT_FLOAT_EQ(geometry::normaliseAngle(-M_PI - EPS * 0.2), -M_PI);

    ASSERT_FLOAT_EQ(round(geometry::normaliseAngle(2 * M_PI)), 0); // numerical errors
}
