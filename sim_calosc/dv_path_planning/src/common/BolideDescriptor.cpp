#include "../../include/common/BolideDescriptor.h"
#include "../../include/common/config.h"

void BolideDescriptor::computeSourcePoints()
{
    Vec2 backward = geometry::createVec2(BolideDescriptor::getTheta(), BOLIDE_LENGTH_TILL_END);
    Vec2 left_vec = geometry::createVec2(BolideDescriptor::getTheta() + M_PI_2, BOLIDE_WIDTH / 2);
    leftSource = BolideDescriptor::getPosition() - backward + left_vec;

    Vec2 right_vec = geometry::createVec2(BolideDescriptor::getTheta() - M_PI_2, BOLIDE_WIDTH / 2);
    rightSource = BolideDescriptor::getPosition() - backward + right_vec;
    middleSource = BolideDescriptor::getPosition() - backward;
}