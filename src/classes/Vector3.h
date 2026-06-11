/**
 * Vector3.h
 *
 * C++ struct used as a helper object for working with coordinates.
 *
 * Author: Jeanette Müller
 * Date: 2025
 */

#ifndef Vector3_h
#define Vector3_h

// 3D vector struct for position and direction
struct Vector3
{
    float x, y, z;

    Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3 &v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3 operator-(const Vector3 &v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3 operator*(float s) const
    {
        return Vector3(x * s, y * s, z * s);
    }

    Vector3 rotate(float angleRad) const
    {
        float c = cosf(angleRad);
        float s = sinf(angleRad);

        return Vector3(
            x * c - z * s,
            0,
            x * s + z * c);
    }

    // Rotation around the X axis
    Vector3 rotateX(const Vector3 &v, float angle)
    {
        float c = cosf(angle);
        float s = sinf(angle);
        return Vector3(
            v.x,
            v.y * c - v.z * s,
            v.y * s + v.z * c);
    }

    // Rotation around the Y axis
    Vector3 rotateY(const Vector3 &v, float angle)
    {
        float c = cosf(angle);
        float s = sinf(angle);
        return Vector3(
            v.x * c + v.z * s,
            v.y,
            -v.x * s + v.z * c);
    }

    // Rotation around the Z axis
    Vector3 rotateZ(const Vector3 &v, float angle)
    {
        float c = cosf(angle);
        float s = sinf(angle);
        return Vector3(
            v.x * c - v.y * s,
            v.x * s + v.y * c,
            v.z);
    }
};
#endif