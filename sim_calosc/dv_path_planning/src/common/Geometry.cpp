#include "../../include/common/Geometry.h"
#include "../../include/common/config.h"

namespace geometry
{
    float getAngle(const Vec2 &v1, const Vec2 &v2)
    {
        return atan2(v2.y, v2.x) - atan2(v1.y, v1.x); // atan2 is in the interval [-pi,+pi] radians.
    }

    float getDistance(const Vec2 &v1, const Vec2 &v2)
    {
        float diff_x = v2.x - v1.x;
        float diff_y = v2.y - v1.y;
        return std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    float relativeAngle2D(const Vec2 &v1, const Vec2 &v2, float base_angle)
    {
        float angle = atan2(v2.y - v1.y, v2.x - v1.x) - base_angle;
        return scaleAngleTo0_2pi(angle);
    }

    float normaliseAngle(float angle)
    {
        while (angle < -M_PI)
            angle += 2 * M_PI;

        while (angle >= M_PI - EPS)
            angle -= 2 * M_PI;

        if (angle < -M_PI + EPS)
            angle = -M_PI;
        return angle;
    }

    float scaleAngleTo0_2pi(float angle)
    {
        while (angle < 0)
            angle += 2 * M_PI;

        while (angle >= 2 * M_PI - EPS)
            angle -= 2 * M_PI;

        if (angle < EPS)
            return 0.0;
        return angle;
    }

    Vec2 createVec2(float angle, float length)
    {
        if (length < 0)
            length = 0;
        float x = length * cos(normaliseAngle(angle));
        float y = length * sin(normaliseAngle(angle));
        return Vec2(x, y);
    }

    float getDirectedAngle(const Vec2 &v1, const Vec2 &v2, const Vec2 &v3)
    {
        float angle = getAngle(Vec2(), v3 - v2) - getAngle(Vec2(), v1 - v2);
        return scaleAngleTo0_2pi(angle);
    }

    Vec2 getMiddlePoint(const Vec2 &v1, const Vec2 &v2)
    {
        return (v2 - v1) * 0.5 + v1;
    }

} // namespace geometry