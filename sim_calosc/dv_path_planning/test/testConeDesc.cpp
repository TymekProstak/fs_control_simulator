#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/cone_chain/Cone.h"

TEST(PathPlanningConeTest, Constructor)
{
    Cone cone_desc(ConeType::BLUE_LEFT, Vec2(1, 1));
    ASSERT_EQ(cone_desc.getPosition().x, 1);
    ASSERT_EQ(cone_desc.getPosition().y, 1);

    Cone cone_desc_without_vec(ConeType::UNDEFINED);
    ASSERT_EQ(cone_desc_without_vec.getPosition().x, 0);
    ASSERT_EQ(cone_desc_without_vec.getPosition().y, 0);
}