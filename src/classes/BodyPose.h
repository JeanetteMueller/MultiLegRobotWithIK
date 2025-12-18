#ifndef BodyPose_H
#define BodyPose_H

// Struktur für Körper-Parameter
struct BodyPose
{
    float height;     // Höhe des Körpers über dem Boden (mm)
    float footOffset; // X-Versatz der Füße vom Körperrand (mm)
    float tiltX;      // Kippen um X-Achse / Pitch (Radiant)
    float tiltZ;      // Kippen um Z-Achse / Roll (Radiant)
    float rotY;       // Drehung um Y-Achse / Yaw (Radiant)

    // Konstruktor mit Standardwerten
    BodyPose() : height(200.0), footOffset(10.0), tiltX(0.0), tiltZ(0.0), rotY(0.0) {}

    // Konstruktor mit Winkeln in Grad
    static BodyPose fromDegrees(float height, float footOffset,
                                float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        BodyPose pose;
        pose.height = height;
        pose.footOffset = footOffset;
        pose.tiltX = tiltXDeg * M_PI / 180.0f;
        pose.tiltZ = tiltZDeg * M_PI / 180.0f;
        pose.rotY = rotYDeg * M_PI / 180.0f;
        return pose;
    }
};

#endif