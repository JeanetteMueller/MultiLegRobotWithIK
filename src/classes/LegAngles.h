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

    bool allAnglesInLimit(uint8_t legIndex) const
    {
        bool ok = true;

        float swing = coxaDeg();
        float lift = femurDeg();
        float knee = tibiaDeg();

        if (swing > 60 || swing < -60 || debug)
        {
            Serial.print(" swing not in limit: ");
            Serial.print(swing);
            if (swing > 60 || swing < -60)
            {
                ok = false;
            }
        }

        if (lift > 100 || lift < -100 || debug)
        {
            Serial.print(" lift not in limit: ");
            Serial.print(lift);
            if (lift > 90 || lift < -90)
            {
                ok = false;
            }
        }

        if (knee > 180 - 40 || knee < -180 - 40 || debug)
        {
            Serial.print(" knee not in limit: ");
            Serial.print(knee);
            if (knee > 180 - 40 || knee < -180 - 40)
            {
                ok = false;
            }
        }

        if (!ok || debug)
        {
            Serial.print(" -> leg: ");
            Serial.println(legIndex);
        }

        return ok;
    }
};

#endif