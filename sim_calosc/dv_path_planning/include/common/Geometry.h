#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Vec2.h"

namespace geometry
{
    /**
     *  Computes angle between vectors v1 and v2, directed from v1 to v2.
     *  Vectors are assumed to start at (0,0) coordinates and ends at (v.x,v.y) coordinates.

     @link https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
     @note atan2 returns arc tangent of y/x, in the interval [-pi,+pi] radians.
     @returns angle in radians
    */
    float getAngle(const Vec2 &v1, const Vec2 &v2);

    /**
     * Computes sitance between v1 and v2
     */
    float getDistance(const Vec2 &v1, const Vec2 &v2);

    /**
     * @brief Computes angle between which line segment [v1, v2] creates with base angle;
     * 
     * It differs from `getAngle` that here Vec2 is treated as a point and there it is
     * treated as a vector.
     *
     * @link https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
     * @param base_angle in radians
     * @returns angle in radians [0, 2*pi)
     */
    float relativeAngle2D(const Vec2 &v1, const Vec2 &v2, float base_angle);

    /**
     * Return agle in range [-pi; pi)
     */
    float normaliseAngle(float angle);

    /**
     * Return agle in range [0; 2pi)
     */
    float scaleAngleTo0_2pi(float angle);

    /**
     *  Creates Vec2 from angle measured from x axis and length.
     * @param angle in radians
     * @param length positive float
     * @return Vec2 object
     */
    Vec2 createVec2(float angle, float length);

    /**
     * Calculated angle between 3 points, where we start from v1, v2 is peek and end at v3.
     * @return angle in radians from range [0, 2pi)
     */
    float getDirectedAngle(const Vec2 &v1, const Vec2 &v2, const Vec2 &v3);

    /**
     * Computes middle point between v1 and v2
     */
    Vec2 getMiddlePoint(const Vec2 &v1, const Vec2 &v2);

} // namespace geometry

#endif