#ifndef BOLIDE_DESC_H
#define BOLIDE_DESC_H

#include "Vec2.h"
#include "Geometry.h"
#include "../cone_chain/Side.h"

/**
 * Representation of the vehicle.
 * This class was made as static (there is no possibility to create object of its type)
 * because there is only one vehicle in the entire simulation.
 *
 * @param theta angle in randians between vehicle vector and X axis
 */
struct BolideDescriptor
{
    BolideDescriptor(Vec2 position, float theta)
    {
        setCoordinates(position, theta);
    }

    void setCoordinates(Vec2 position, float theta)
    {
        this->position = position;
        this->theta = geometry::scaleAngleTo0_2pi(theta);
        computeSourcePoints();
    }

    float getTheta() const
    {
        return theta;
    }

    /**
     * Return the position of the vehicle's center. For reference, please refer to the `position` in the documentation.
     */
    Vec2 getPosition() const { return position; }

    Vec2 getSourcePoint(Side side) const
    {
        if (side == Side::LEFT)
            return leftSource;
        return rightSource;
    }

    /**
     * Return the left source point, which is the point at the left rear of the vehicle.
     * From this point the left cone chain algorithm starts calculations.
     */
    Vec2 getLeftSourcePoint() const { return leftSource; }

    /**
     * Return the right source point, which is the point at the right rear of the vehicle.
     * From this point the right cone chain algorithm starts calculations.
     */
    Vec2 getRightSourcePoint() const { return rightSource; }

    /**
     * Return the middle point between left source point and right source point.
     */
    Vec2 getMiddleSourcePoint() const { return middleSource; }

    bool isBehindVehicle(const Vec2 &point) const
    {
        float angle = geometry::getDirectedAngle(getMiddleSourcePoint(), getPosition(), point);
        return (angle <= M_PI_2 * 1.05 || angle >= 2 * M_PI - M_PI_2 * 1.05);
    }

private:
    float theta; // in radians in [0; 2pi)
    Vec2 position;
    Vec2 leftSource, rightSource, middleSource;
    Vec2 startPosition;

    void computeSourcePoints();
};

#endif