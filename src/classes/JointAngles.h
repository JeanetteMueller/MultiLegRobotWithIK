#ifndef JointAngles_h
#define JointAngles_h

// Gelenkwinkel für ein Bein
struct JointAngles
{
    double theta0;  // Erstes vertikales Gelenk
    double theta1;  // Seitliche Rotation (horizontal)
    double theta2;  // Zweites vertikales Gelenk
    bool reachable; // Ist das Ziel erreichbar?

    bool debug = false;

    JointAngles() : theta0(0), theta1(0), theta2(0), reachable(false) {}

    bool allAnglesInLimit(int legIndex, int legCount) const
    {
        bool ok = true;

        if (theta0 > 90 || theta0 < -90 || debug)
        {
            Serial.print(" theta0 not in limit: ");
            Serial.print(theta0);
            if (theta0 > 90 || theta0 < -90)
            {
                ok = false;
            }
        }

        if (theta1 > 50 || theta1 < -50 || debug)
        {
            Serial.print(" theta1 not in limit: ");
            Serial.print(theta1);
            if (theta1 > 50 || theta1 < -50)
            {
                ok = false;
            }
        }

        if (theta2 < 40 || debug)
        {
            Serial.print(" theta2 not in limit: ");
            Serial.print(theta2);
            if (theta2 < 40)
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