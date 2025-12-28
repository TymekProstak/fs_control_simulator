#ifndef VEC_2_H
#define VEC_2_H

#include <math.h>
#include <stdexcept>
#include <iostream>

#define _clamp(v,mn,mx)  ((v < mn) ? mn : ((v > mx) ? mx : v))

/**
 * Implementation of 2D vector.
 */
struct Vec2
{
    float x, y; // coordinates

    Vec2() : x(0), y(0) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(const float d) const { return Vec2(x * d, y * d); }
    Vec2 operator*(const Vec2 &v) const { return Vec2(x * v.x, y * v.y); }
    Vec2 operator/(const Vec2 &v) const { return Vec2(x / v.x, y / v.y); }

    Vec2 &operator+=(const Vec2 &v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vec2 &operator-=(const Vec2 &v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vec2 &operator*=(const float d)
    {
        x *= d;
        y *= d;
        return *this;
    }

    bool operator!=(const Vec2 &v) const { return (x != v.x) || (y != v.y); }
    bool operator==(const Vec2 &d) const { return (x == d.x) && (y == d.y); }

    /**
     * @note no mathematical meaning but necessary to use Vec2 in std::map
     */
    bool operator<(const Vec2 &v) const
    {
        if (x != v.x)
            return x < v.x;
        else
            return y < v.y;
    }

    /**
     * Division by a scalar
     *
     * @param d cannot be zero
     */
    Vec2 operator/(float d) const
    {
        if (d == 0)
        {
            // to avoid programm crashing it does not throws an error
            d = 0.001;
        }
        return Vec2(x / d, y / d);
    }

    /**
     * Division by scalar
     *
     * @param d cannot be zero
     * @throws invalid_argument when division by zero
     *
     */
    Vec2 &operator/=(float d)
    {
        if (d == 0)
        {
            // to avoid programm crashing it does not throws an error
            d = 0.001;
        }
        x /= d;
        y /= d;
        return *this;
    }

    /**
     * Enables multiplication in order: scalar * Vec2
     */
    friend Vec2 operator*(const float scalar, const Vec2 &vec)
    {
        return Vec2(scalar * vec.x, scalar * vec.y);
    }

    /**
     * Oposite vector
     */
    Vec2 operator-() const { return Vec2(-x, -y); }

    /// -----------------------------
    /// Additional vectors operations
    /// -----------------------------

    /**
     * Returns vector's length (Euclidean norm)
     */
    float length() const
    {
        return std::sqrt(x * x + y * y);
    }

    /**
     * Rotate vector by given angle, more about transformations:
     * https://en.wikipedia.org/wiki/Rotation_of_axes_in_two_dimensions
     */
    void rotate(float angle)
    {
        float newX = x * cos(angle) + y * sin(angle);
        float newY = x * -1 * sin(angle) + y * cos(angle);
        x = newX;
        y = newY;
    }

    float len_square() const { return x * x + y * y; }
    Vec2 normal() const { return Vec2(x, y) / length(); }

    Vec2 Get_Clamped(const float mn, const float mx) const { return Vec2{_clamp(x, mn, mx), _clamp(y, mn, mx)};  };
};

struct Vec2HashFunction
{
    size_t operator()(const Vec2 &vec) const
    {
        size_t xHash = std::hash<float>()(vec.x);
        size_t yHash = std::hash<float>()(vec.y) << 1;
        // return xHash ^ (yHash + 0x9e3779b9 + (xHash << 6) + (xHash >> 2)); // a little bit faster up to 10 nodes
        return xHash ^ yHash;
    }
};

#endif