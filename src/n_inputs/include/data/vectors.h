#pragma once

namespace newton
{
    struct Vector3
    {
        float x;
        float y;
        float z;

        Vector3() : x(0.f), y(0.f), z(0.f) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        Vector3(double x, double y, double z) : x(static_cast<float>(x)), y(static_cast<float>(y)), z(static_cast<float>(z)) {}

        Vector3 operator+=(const Vector3 &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        Vector3 operator*(const double &rhs)
        {
            x *= rhs;
            y *= rhs;
            z *= rhs;
            return *this;
        }
    };
} // namespace newton