/**
 * BodyPose.h
 *
 * C++ Struct um die zielwerte des Roboters zu definieren.
 *
 * Autor: Jeanette Müller
 * Datum: 2025
 */

#ifndef BodyPose_H
#define BodyPose_H

// Struktur für Körper-Parameter
struct BodyPose
{
    float height;     // Höhe des Körpers über dem Boden (mm)
    float tiltX;      // Kippen um X-Achse / Pitch (Radiant)
    float tiltZ;      // Kippen um Z-Achse / Roll (Radiant)
    float rotY;       // Drehung um Y-Achse / Yaw (Radiant)

    // Konstruktor mit Standardwerten
    BodyPose() : height(200.0), tiltX(0.0), tiltZ(0.0), rotY(0.0) {}

    // Konstruktor mit Winkeln in Grad
    static BodyPose fromDegrees(float height,
                                float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        BodyPose pose;
        pose.height = height;
        pose.tiltX = tiltXDeg * M_PI / 180.0f;
        pose.tiltZ = tiltZDeg * M_PI / 180.0f;
        pose.rotY = rotYDeg * M_PI / 180.0f;
        return pose;
    }
};

#endif