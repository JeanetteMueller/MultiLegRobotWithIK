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
    float swing; // θ0: Hüft-Schwenk (um Achse senkrecht zum Oberschenkel)
    float lift;  // θ1: Hüft-Heben
    float knee;  // θ2: Kniewinkel
    bool valid;   // true wenn Position erreichbar

    bool debug = false;

    // Winkel in Grad zurückgeben
    float swingDeg() const { return swing * 180.0 / M_PI; }
    float liftDeg() const { return lift * 180.0 / M_PI; }
    float kneeDeg() const { return knee * 180.0 / M_PI; }

    bool allAnglesInLimit(uint8_t legIndex, int legCount) const
    {
        bool ok = true;

        float lift = liftDeg();
        float swing = swingDeg();
        float knee = kneeDeg();

        if (lift > 90 || lift < -90 || debug)
        {
            Serial.print(" lift not in limit: ");
            Serial.print(lift);
            if (lift > 90 || lift < -90)
            {
                ok = false;
            }
        }

        if (swing > 50 || swing < -50 || debug)
        {
            Serial.print(" swing not in limit: ");
            Serial.print(swing);
            if (swing > 50 || swing < -50)
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