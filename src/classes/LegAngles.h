/**
 * LegAngles.h
 *
 * C++ struct to return the computed angles for each leg.
 *
 * Author: Claude.ai & Jeanette Müller
 * Date: 2025
 */

#ifndef LegAngles_H
#define LegAngles_H

#include "LegLimits.h"

// Struct for the 3 joint angles of a leg (in radians)
struct LegAngles
{
    float coxa;  // θ0: coxa angle (around the vertical Y axis)
    float femur; // θ1: femur angle (around the horizontal axis)
    float tibia; // θ2: tibia/knee angle (around the horizontal axis)
    bool valid;  // true if the position is reachable

    bool debug = false;

    // return angles in degrees
    double coxaDeg() const { return coxa * 180.0 / M_PI; }
    double femurDeg() const { return femur * 180.0 / M_PI; }
    double tibiaDeg() const { return tibia * 180.0 / M_PI; }

    // Checks whether all 3 joint angles lie within the given limits.
    // The limits are set per leg in definitions.h (see LegLimits).
    bool allAnglesInLimit(const LegLimits &limits) const
    {
        bool ok = true;

        float swing = coxaDeg();
        float lift = femurDeg();
        float knee = tibiaDeg();

        if (!limits.coxa.contains(swing) || debug)
        {
            Serial.print(" swing not in limit: ");
            Serial.print(swing);
            if (!limits.coxa.contains(swing))
            {
                ok = false;
            }
        }

        if (!limits.femur.contains(lift) || debug)
        {
            Serial.print(" lift not in limit: ");
            Serial.print(lift);
            if (!limits.femur.contains(lift))
            {
                ok = false;
            }
        }

        if (!limits.tibia.contains(knee) || debug)
        {
            Serial.print(" knee not in limit: ");
            Serial.print(knee);
            if (!limits.tibia.contains(knee))
            {
                ok = false;
            }
        }

        return ok;
    }
};

#endif