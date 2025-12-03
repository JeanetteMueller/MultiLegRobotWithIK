
#ifndef CircularRobot_h
#define CircularRobot_h

#include <array>
#include <cmath>

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

// Gelenkwinkel für ein Bein
struct JointAngles
{
    double theta0;  // Seitliche Rotation (horizontal)
    double theta1;  // Erstes vertikales Gelenk
    double theta2;  // Zweites vertikales Gelenk
    bool reachable; // Ist das Ziel erreichbar?

    JointAngles() : theta0(0), theta1(0), theta2(0), reachable(false) {}
};

// Roboter Klasse
class CircularRobot
{
private:
    int NUM_LEGS = 1;
    double BODY_RADIUS;     // mm
    double SEGMENT2_LENGTH; // mm
    double SEGMENT3_LENGTH; // mm

    Vector3 bodyPosition; // Position des Körperzentrums
    double tiltX;         // Neigung um X-Achse (Vorwärts/Rückwärts) in Radiant
    double tiltY;         // Neigung um Y-Achse (Seitwärts) in Radiant
    double tiltZ;         // Rotation um Z-Achse in Radiant

    std::vector<Vector3> footPositions; // Feste Fußpositionen auf dem Boden

public:
    CircularRobot(int numberOfLegs, double bodyRadius, double thighLength, double shinLength, double bodyPositionOverZero = 200.0)
    {
        NUM_LEGS = numberOfLegs;
        BODY_RADIUS = bodyRadius;
        SEGMENT2_LENGTH = thighLength;
        SEGMENT3_LENGTH = shinLength;

        bodyPosition = Vector3(0, bodyPositionOverZero, 0);

        tiltX = 0;
        tiltY = 0;
        tiltZ = 0;

        // Initialisiere Fußpositionen (gleichmäßig verteilt, 72° Abstand)
        for (int i = 0; i < NUM_LEGS; i++)
        {
            double angle = i * 72.0 * M_PI / 180.0;
            double footRadius = BODY_RADIUS - 10.0; // Abstand vom Zentrum
            footPositions.push_back(Vector3(
                std::cos(angle) * footRadius,
                0.0, // Boden
                std::sin(angle) * footRadius));
        }
    }

    // Setze Körperneigung (in Grad)
    void setTilt(double tiltXDeg, double tiltYDeg, double tiltZDeg)
    {
        tiltX = tiltXDeg * M_PI / 180.0;
        tiltY = tiltYDeg * M_PI / 180.0;
        tiltZ = tiltZDeg * M_PI / 180.0;
    }

    // Setze Körperhöhe
    void setBodyHeight(double height)
    {
        bodyPosition.y = height;
    }

    // Berechne Beinbasis-Position im Weltkoordinatensystem
    Vector3 getLegBasePosition(int legIndex) const
    {
        // Lokale Position am Körper (vor Rotation)
        double legAngle = legIndex * 72.0 * M_PI / 180.0;
        Vector3 localPos(
            std::cos(legAngle) * BODY_RADIUS,
            0.0,
            std::sin(legAngle) * BODY_RADIUS);

        // Rotiere mit Körperrotation (Reihenfolge: Z -> Y -> X)
        Vector3 rotated = rotateZ(localPos, tiltZ);
        rotated = rotateY(rotated, tiltY);
        rotated = rotateX(rotated, tiltX);

        // Addiere Körperposition
        return rotated + bodyPosition;
    }

    // Inverse Kinematik für ein Bein
    JointAngles calculateLegIK(int legIndex) const
    {
        JointAngles angles;

        // Basis-Position des Beins
        Vector3 basePos = getLegBasePosition(legIndex);

        // Ziel-Position (Fußpunkt)
        Vector3 footPos = footPositions[legIndex];

        // Vektor von Basis zum Fuß
        Vector3 toFoot = footPos - basePos;
        double distance = toFoot.length();

        // Theta0: Horizontale Rotation des ersten Gelenks
        angles.theta0 = std::atan2(toFoot.z, toFoot.x);

        // Projektion für 2D IK (in der vertikalen Ebene)
        double horizontalDist = std::sqrt(toFoot.x * toFoot.x + toFoot.z * toFoot.z);
        double verticalDist = toFoot.y;

        // Abstand zum Ziel in der 2D-Ebene
        double distToTarget = std::sqrt(
            horizontalDist * horizontalDist +
            verticalDist * verticalDist);

        // Längen der beiden IK-Segmente
        double l2 = SEGMENT2_LENGTH;
        double l3 = SEGMENT3_LENGTH;

        // Prüfe ob Ziel erreichbar ist
        if (distToTarget > l2 + l3 + 0.1 || distToTarget < std::abs(l2 - l3) - 0.1)
        {
            angles.reachable = false;
            return angles;
        }

        // Begrenze distToTarget für numerische Stabilität
        distToTarget = std::min(distToTarget, l2 + l3 - 0.1);
        distToTarget = std::max(distToTarget, std::abs(l2 - l3) + 0.1);

        // Theta2: Winkel des "Knie"-Gelenks (Cosinus-Satz)
        double cosTheta2 = (distToTarget * distToTarget - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
        cosTheta2 = std::max(-1.0, std::min(1.0, cosTheta2)); // Clamp für acos
        angles.theta2 = std::acos(cosTheta2);

        // Theta1: Winkel des ersten vertikalen Gelenks
        double alpha = std::atan2(verticalDist, horizontalDist);
        double cosBeta = (l2 * l2 + distToTarget * distToTarget - l3 * l3) / (2.0 * l2 * distToTarget);
        cosBeta = std::max(-1.0, std::min(1.0, cosBeta)); // Clamp für acos
        double beta = std::acos(cosBeta);

        angles.theta1 = alpha + beta;

        // Konvertiere theta2 für korrekte Gelenkrichtung
        angles.theta2 = M_PI - angles.theta2;

        angles.reachable = true;
        return angles;
    }

    // Berechne IK für alle Beine
    std::vector<JointAngles> calculateAllLegsIK() const
    {
        std::vector<JointAngles> allAngles;
        for (int i = 0; i < NUM_LEGS; i++)
        {
            allAngles.push_back(calculateLegIK(i));
        }
        return allAngles;
    }

    // Ausgabe der Gelenkwinkel
    void printLegAngles(int legIndex, const JointAngles &angles) const
    {
        Serial.print("Leg ");
        Serial.print(legIndex);
        Serial.print(": ");
        if (!angles.reachable)
        {
            Serial.println("NICHT ERREICHBAR!");
            return;
        }
        Serial.print(" 0: ");
        Serial.print(angles.theta0 * 180.0 / M_PI);
        Serial.print("° ");
        Serial.print(" 1: ");
        Serial.print(angles.theta1 * 180.0 / M_PI);
        Serial.print("° ");
        Serial.print(" 2: ");
        Serial.print(angles.theta2 * 180.0 / M_PI);
        Serial.println("°");
    }

    bool allLegAnglesAreReachable(const std::vector<JointAngles> &angles) const
    {
        uint8_t count = sizeof(angles) / sizeof(angles[0]);

        for (int i = 0; i < count; i++)
        {
            if (!angles[i].reachable)
            {
                return false;
            }
        }
        return true;
    }
};

#endif