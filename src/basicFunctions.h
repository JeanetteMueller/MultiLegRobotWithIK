/**
 * basicFunctions.h
 *
 * Helper functions used across the library and convenient for sketches.
 *
 * Autor: Jeanette Müller
 * Datum: 2025
 */

#ifndef BASIC_FUNCTIONS_H
#define BASIC_FUNCTIONS_H

#include <Arduino.h>
#include <cmath>

#include "classes/LegAngles.h"

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float cosFromDegree(float degree)
{
    return cos(degree * M_PI / 180);
}
inline float degToRad(float deg)
{
    return deg * M_PI / 180.0f;
}

inline void printLegAngles(const LegAngles &angles, int legIndex)
{
    Serial.print("  Bein ");
    Serial.print(legIndex);
    Serial.print(": ");

    if (angles.valid)
    {

        Serial.print("θ0=");
        Serial.print(angles.coxaDeg());
        Serial.print("° (Schwenk), ");
        Serial.print("θ1=");
        Serial.print(angles.femurDeg());
        Serial.print("° (Heben), ");
        Serial.print("θ2=");
        Serial.print(angles.tibiaDeg());
        Serial.print("° (Knie)");
    }
    else
    {
        Serial.print("Position nicht erreichbar!");
    }
}

inline double getAngleFromVector(double x, double y)
{
    double a = abs(x);
    double b = abs(y);
    double c = sqrt(pow(abs(x), 2) + pow(abs(y), 2));

    double directionAngle = 0;

    if (x == 0 && y > 0)
    {
        directionAngle = 0;
    }
    else if (x == 0 && y < 0)
    {
        directionAngle = 180;
    }
    else if (x > 0 && y == 0)
    {
        directionAngle = 90;
    }
    else if (x < 0 && y == 0)
    {
        directionAngle = 270;
    }
    else
    {
        directionAngle = acos((c * c + a * a - b * b) / (2 * c * a)) * 180 / M_PI;

        if (y > 0)
        {
            if (x > 0)
            {
                directionAngle = 90 - directionAngle;
            }
            else if (x < 0)
            {
                directionAngle = 270 + directionAngle;
            }
        }
        else if (y < 0)
        {
            if (x > 0)
            {
                directionAngle = 90 + directionAngle;
            }
            else if (x < 0)
            {
                directionAngle = 270 - directionAngle;
            }
        }
    }
    return directionAngle;
}

#endif // BASIC_FUNCTIONS_H
