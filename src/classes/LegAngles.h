/**
 * LegAngles.h
 *
 * C++ Struct zur Rückgabe der errechneten Winkel je Bein.
 *
 * Autor: Claude.ai & Jeanette Müller
 * Datum: 2025
 */

#ifndef LegAngles_H
#define LegAngles_H

#include "LegLimits.h"

// Struktur für die 3 Gelenkwinkel eines Beins (in Radiant)
struct LegAngles
{
    float coxa;  // θ0: Coxa-Winkel (um vertikale Y-Achse)
    float femur; // θ1: Femur-Winkel (um horizontale Achse)
    float tibia; // θ2: Tibia/Knie-Winkel (um horizontale Achse)
    bool valid;  // true wenn Position erreichbar

    bool debug = false;

    // Winkel in Grad zurückgeben
    double coxaDeg() const { return coxa * 180.0 / M_PI; }
    double femurDeg() const { return femur * 180.0 / M_PI; }
    double tibiaDeg() const { return tibia * 180.0 / M_PI; }

    // Prüft, ob alle 3 Gelenkwinkel innerhalb der übergebenen Limits liegen.
    // Die Limits werden pro Bein in der definitions.h gesetzt (siehe LegLimits).
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