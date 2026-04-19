#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/common/Geometry.h"

TEST(PathPlanningVec2Test, AdditionsAndSubstractions)
{
    Vec2 v_zero(0, 0), v_one(1, 1), v_random(2137, 420);
    ASSERT_EQ(v_zero + v_zero, v_zero);
    ASSERT_EQ(v_zero - v_zero, v_zero);
    ASSERT_EQ(v_zero + v_one, v_one);
    ASSERT_EQ(v_zero - v_one, -v_one);
    ASSERT_EQ(v_random + v_one, Vec2(2138, 421));
    v_random += v_one;
    ASSERT_EQ(v_random, Vec2(2138, 421));
    v_random -= v_one;
    ASSERT_EQ(v_random, Vec2(2137, 420));
    ASSERT_EQ(Vec2(1, 1) + Vec2(2, -1), Vec2(3, 0));
    ASSERT_EQ(Vec2(1.5, -1.2) + Vec2(0.5, -1), Vec2(2.0, -2.2));
    ASSERT_EQ(-Vec2(1.5, -1.2) - Vec2(0.5, -1), -Vec2(2.0, -2.2));
}

TEST(PathPlanningVec2Test, MultiplicationsAndDivisions)
{
    Vec2 v_zero(0, 0), v_one(1, 1), v_two(2, 2), v_random(2137, 420);
    ASSERT_EQ(v_one * 0, v_zero);
    ASSERT_EQ(0 * v_one, v_zero);
    ASSERT_EQ(2 * v_one, v_two);

    Vec2 vec_div0 = Vec2(1, 1) / 0;
    ASSERT_FLOAT_EQ(vec_div0.x, 1000);
    ASSERT_FLOAT_EQ(vec_div0.y, 1000);

    auto vec_div0_2 = Vec2(2, 2);
    vec_div0_2 /= 0;
    ASSERT_FLOAT_EQ(vec_div0_2.x, 2000);
    ASSERT_FLOAT_EQ(vec_div0_2.y, 2000);

    ASSERT_EQ(v_random / 2, Vec2(1068.5, 210));
    ASSERT_EQ(v_random / 2 * 2, v_random);

    v_one *= 3;
    ASSERT_EQ(v_one, Vec2(3, 3));
    v_one /= -2;
    ASSERT_EQ(v_one, Vec2(-1.5, -1.5));
}

TEST(PathPlanningVec2Test, ComparisonOperators)
{
    ASSERT_TRUE(Vec2(1, 23) == Vec2(1, 23));
    ASSERT_TRUE(Vec2() == Vec2());
    ASSERT_FALSE(Vec2(1, 0) == Vec2(0, 0));
    ASSERT_FALSE(Vec2(1, 1) == Vec2(0, 0));
    ASSERT_FALSE(Vec2(1, 1) == Vec2(1, 0));

    ASSERT_TRUE(Vec2(1, 23) != Vec2(-1, 23));
    ASSERT_TRUE(Vec2(1, -23) != Vec2(1, 23));
    ASSERT_FALSE(Vec2() != Vec2());

    ASSERT_TRUE(Vec2(-1, 23) < Vec2(1, 23));
    ASSERT_TRUE(Vec2(-1, 3) < Vec2(-1, 23));
    ASSERT_FALSE(Vec2(-1, 23) < Vec2(-1, 23));
}

TEST(PathPlanningVec2Test, Length)
{
    ASSERT_FLOAT_EQ(Vec2(1, 0).length(), 1);
    ASSERT_FLOAT_EQ(Vec2(0, -1).length(), 1);
    ASSERT_FLOAT_EQ(Vec2(0, 0).length(), 0);
    ASSERT_FLOAT_EQ(Vec2().length(), 0);
}