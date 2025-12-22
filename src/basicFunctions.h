/**
 * basicFunctions.h
 *
 * Helper functions.
 *
 * Autor: Jeanette Müller
 * Datum: 2025
 */

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float cosFromDegree(float degree)
{
    return cos(degree * M_PI / 180);
}



void printLegAngles(const LegAngles& angles, int legIndex) {
    Serial.print("  Bein ");
    Serial.print(legIndex);
    Serial.print(": ");

    if (angles.valid) {

        Serial.print("θ0=");
        Serial.print(angles.swingDeg());
        Serial.print("° (Schwenk), ");
        Serial.print("θ1=");
        Serial.print(angles.liftDeg());
        Serial.print("° (Heben), ");
        Serial.print("θ2=");
        Serial.print(angles.kneeDeg());
        Serial.print("° (Knie)");

    } else {
        Serial.print("Position nicht erreichbar!");
    }
}

float degToRad(float deg) {
    return deg * M_PI / 180.0f;
}