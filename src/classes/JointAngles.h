#ifndef JointAngles_h
#define JointAngles_h

// Gelenkwinkel für ein Bein
struct JointAngles
{
    double theta0;  // Erstes vertikales Gelenk
    double theta1;  // Seitliche Rotation (horizontal)
    double theta2;  // Zweites vertikales Gelenk
    bool reachable; // Ist das Ziel erreichbar?

    JointAngles() : theta0(0), theta1(0), theta2(0), reachable(false) {}

    bool allAnglesInLimit(int legIndex, int legCount) const
    {
        double degree0 = theta0 * 180.0 / M_PI;
        if (degree0 < 0 && legIndex > 1)
        {
            degree0 = 360 + degree0;
        }
        degree0 = degree0 - (legIndex * (360.0 / legCount));

        if (degree0 > 50 || degree0 < -50)
        {
            Serial.print("degree0 not in limit: ");
            Serial.println(degree0);
            return false;
        }

        double degree1 = theta1 * 180.0 / M_PI;
        if (degree1 > 90 || degree1 < -90)
        {
            Serial.print("degree1 not in limit: ");
            Serial.println(degree1);
            return false;
        }

        double degree2 = theta2 * 180.0 / M_PI;
        if (degree2 < 40)
        {
            Serial.print("degree2 not in limit: ");
            Serial.println(degree2);
            return false;
        }

        return true;
    }
};
#endif