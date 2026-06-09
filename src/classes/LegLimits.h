/**
 * LegLimits.h
 *
 * C++ Structs für die Gelenk-Limits eines Beins (in Grad).
 *
 * Früher waren die Limits hart in LegAngles.h hinterlegt und damit nur für
 * Rocky passend. Über diese Structs werden sie pro Roboter in der definitions.h
 * eingepflegt und an jedes RobotLeg übergeben.
 *
 * Autor: Claude.ai & Jeanette Müller
 * Datum: 2025
 */

#ifndef LegLimits_H
#define LegLimits_H

// Erlaubter Wertebereich eines einzelnen Gelenks (in Grad).
// Default ist bewusst weit (±180°), damit ein nicht konfiguriertes Bein nicht
// versehentlich alle Posen blockiert.
struct Limits
{
    float min = -180.0f;
    float max = 180.0f;

    // true, wenn der Winkel innerhalb (inkl. Grenzen) liegt
    bool contains(float value) const
    {
        return value >= min && value <= max;
    }
};

// Limits für alle 3 Gelenke eines Beins.
struct LegLimits
{
    Limits coxa;  // Schwenk (swing) um die vertikale Achse
    Limits femur; // Anheben (lift)
    Limits tibia; // Knie (knee)
};

#endif // LegLimits_H
