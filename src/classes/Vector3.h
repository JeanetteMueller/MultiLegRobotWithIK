#ifndef Vector3_h
#define Vector3_h

// 3D Vektor Struktur für Position und Richtung
struct Vector3
{
    double x, y, z;

    Vector3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3 &v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3 operator-(const Vector3 &v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3 operator*(double s) const
    {
        return Vector3(x * s, y * s, z * s);
    }

    double length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3 normalize() const
    {
        double len = length();
        return (len > 0) ? Vector3(x / len, y / len, z / len) : Vector3();
    }
};

// Rotation um X-Achse
Vector3 rotateX(const Vector3 &v, double angle)
{
    double c = std::cos(angle);
    double s = std::sin(angle);
    return Vector3(
        v.x,
        v.y * c - v.z * s,
        v.y * s + v.z * c);
}

// Rotation um Y-Achse
Vector3 rotateY(const Vector3 &v, double angle)
{
    double c = std::cos(angle);
    double s = std::sin(angle);
    return Vector3(
        v.x * c + v.z * s,
        v.y,
        -v.x * s + v.z * c);
}

// Rotation um Z-Achse
Vector3 rotateZ(const Vector3 &v, double angle)
{
    double c = std::cos(angle);
    double s = std::sin(angle);
    return Vector3(
        v.x * c - v.y * s,
        v.x * s + v.y * c,
        v.z);
}

#endif