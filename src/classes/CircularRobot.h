
#ifndef CircularRobot_h
#define CircularRobot_h

#include <array>
#include <cmath>

#include "Vector3.h"
#include "JointAngles.h"


// Roboter Klasse
class CircularRobot
{
private:
    unsigned long currentMillis = 0;
    unsigned long previousStepMillis = 0;

    int NUM_LEGS = 1;
    double BODY_RADIUS;     // mm
    double SEGMENT2_LENGTH; // mm
    double SEGMENT3_LENGTH; // mm

    Vector3 bodyPosition; // Position des Körperzentrums
    double tiltX;         // Neigung um X-Achse (Vorwärts/Rückwärts) in Radiant
    double tiltY;         // Neigung um Y-Achse (Seitwärts) in Radiant
    double tiltZ;         // Rotation um Z-Achse in Radiant

    std::vector<Vector3> baseFootPositions; // Feste Fußpositionen auf dem Boden
    std::vector<Vector3> lastTargetPosition;
    std::vector<Vector3> targetPosition;
    int currentMovingLeg = 1;
    int walkingStep = -1;
    int walkingStepCount = 5;
    double walk_x = 0;
    double walk_y = 0;
    double walkSpeed = 0;

public:
    CircularRobot(int numberOfLegs, double bodyRadius, double thighLength, double shinLength, double bodyPositionOverZero = 200.0)
    {
        NUM_LEGS = numberOfLegs;
        BODY_RADIUS = bodyRadius;
        SEGMENT2_LENGTH = thighLength;
        SEGMENT3_LENGTH = shinLength;

        bodyPosition = Vector3(0, bodyPositionOverZero, 0);

        tiltX = 0;
        tiltY = 0;
        tiltZ = 0;

        double spaceBetweenLegs = (360.0 / numberOfLegs);

        // Initialisiere Fußpositionen (gleichmäßig verteilt, 72° Abstand bei 5 Beinen)
        for (int i = 0; i < NUM_LEGS; i++)
        {
            double angle = i * spaceBetweenLegs * M_PI / 180.0;
            double footRadius = BODY_RADIUS + 60.0; // Abstand vom Zentrum
            baseFootPositions.push_back(Vector3(
                std::cos(angle) * footRadius,
                0.0, // Boden
                std::sin(angle) * footRadius));
        }
        lastTargetPosition = baseFootPositions;
        targetPosition = baseFootPositions;
    }

    void setWalkDirection(double x, double y, double speed)
    {
        if (walkingStep == 0)
        {
            // andere richtung und geschwindigkeit immer nur wenn alle Füße am Boden sind
            walk_x = x;
            walk_y = y;
            walkSpeed = speed;
        }
    }

    // Setze Körperneigung (in Grad)
    void setTilt(double tiltXDeg, double tiltYDeg, double tiltZDeg)
    {
        tiltX = tiltXDeg * M_PI / 180.0;
        tiltY = tiltYDeg * M_PI / 180.0;
        tiltZ = tiltZDeg * M_PI / 180.0;
    }

    // Setze Körperhöhe
    void setBodyHeight(double height)
    {
        bodyPosition.y = height;
    }

    void mainLoop()
    {
        currentMillis = millis();

        // if (currentMillis - previousStepMillis > 500)
        // {
        //     previousStepMillis = currentMillis;

        //     walkingStep++;

        //     lastTargetPosition = targetPosition;

        //     if (walkingStepCount == walkingStep)
        //     {
        //         walkingStep = 0;

        //         currentMovingLeg++;

        //         if (currentMovingLeg > NUM_LEGS)
        //         {
        //             currentMovingLeg = 1;
        //         }
        //     }

        //     Serial.print("currentMovingLeg: ");
        //     Serial.print(currentMovingLeg);
        //     Serial.print(" step: ");
        //     Serial.println(walkingStep);

        //     // for (int i = 0; i < NUM_LEGS; i++)
        //     // {
        //     //     Vector3 newTarget = getStepTargetPosition(i);
        //     //     targetPosition[i] = newTarget;
        //     // }
        // }
    }

    // Berechne Beinbasis-Position im Weltkoordinatensystem
    Vector3 getLegOriginPosition(int legIndex, int legCount) const
    {
        // Lokale Position am Körper (vor Rotation)
        double spaceBetweenLegs = (360.0 / legCount);
        double legAngle = legIndex * spaceBetweenLegs * M_PI / 180.0;
        Vector3 localPos(
            std::cos(legAngle) * BODY_RADIUS,
            0.0,
            std::sin(legAngle) * BODY_RADIUS);

        // Rotiere mit Körperrotation (Reihenfolge: Z -> Y -> X)
        Vector3 rotated = rotateZ(localPos, tiltZ);
        rotated = rotateY(rotated, tiltY);
        rotated = rotateX(rotated, tiltX);

        // Addiere Körperposition
        return rotated + bodyPosition;
    }

    Vector3 getStepTargetPosition(int legIndex) const
    {
        if (currentMovingLeg == legIndex)
        {
            Vector3 newTarget = baseFootPositions[legIndex];
            newTarget.x += walk_x / NUM_LEGS * (NUM_LEGS - legIndex);
            newTarget.y += walk_y / NUM_LEGS * (NUM_LEGS - legIndex);

            std::vector<Vector3> walkingWay = interpolateSinZ(lastTargetPosition[legIndex], newTarget, walkingStepCount);

            return walkingWay[walkingStep];
        }
        else
        {
            Vector3 newTarget = baseFootPositions[legIndex];
            newTarget.x -= walk_x / NUM_LEGS;
            newTarget.y -= walk_y / NUM_LEGS;

            std::vector<Vector3> walkingWay = interpolate(lastTargetPosition[legIndex], newTarget, walkingStepCount);

            return walkingWay[walkingStep];
        }
    }

    // Inverse Kinematik für ein Bein
    JointAngles calculateLegIK(int legIndex) const
    {
        JointAngles angles;

        // Basis-Position des Beins
        Vector3 basePos = getLegOriginPosition(legIndex, NUM_LEGS);

        // Ziel-Position (Fußpunkt)
        Vector3 footPos = targetPosition[legIndex];

        // Vektor von Basis zum Fuß
        Vector3 toFoot = footPos - basePos;
        double distance = toFoot.length();

        // Theta0: Horizontale Rotation des ersten Gelenks
        angles.theta0 = std::atan2(toFoot.z, toFoot.x);

        // Projektion für 2D IK (in der vertikalen Ebene)
        double horizontalDist = std::sqrt(toFoot.x * toFoot.x + toFoot.z * toFoot.z);
        double verticalDist = toFoot.y;

        // Abstand zum Ziel in der 2D-Ebene
        double distToTarget = std::sqrt(
            horizontalDist * horizontalDist +
            verticalDist * verticalDist);

        // Längen der beiden IK-Segmente
        double l2 = SEGMENT2_LENGTH;
        double l3 = SEGMENT3_LENGTH;

        // Prüfe ob Ziel erreichbar ist
        if (distToTarget > l2 + l3 + 0.1 || distToTarget < std::abs(l2 - l3) - 0.1)
        {
            angles.reachable = false;
            return angles;
        }

        // Begrenze distToTarget für numerische Stabilität
        distToTarget = std::min(distToTarget, l2 + l3 - 0.1);
        distToTarget = std::max(distToTarget, std::abs(l2 - l3) + 0.1);

        // Theta2: Winkel des "Knie"-Gelenks (Cosinus-Satz)
        double cosTheta2 = (distToTarget * distToTarget - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
        cosTheta2 = std::max(-1.0, std::min(1.0, cosTheta2)); // Clamp für acos
        angles.theta2 = std::acos(cosTheta2);

        // Theta1: Winkel des ersten vertikalen Gelenks
        double alpha = std::atan2(verticalDist, horizontalDist);
        double cosBeta = (l2 * l2 + distToTarget * distToTarget - l3 * l3) / (2.0 * l2 * distToTarget);
        cosBeta = std::max(-1.0, std::min(1.0, cosBeta)); // Clamp für acos
        double beta = std::acos(cosBeta);

        angles.theta1 = alpha + beta;

        // Konvertiere theta2 für korrekte Gelenkrichtung
        angles.theta2 = M_PI - angles.theta2;

        angles.reachable = true;
        return angles;
    }

    // Berechne IK für alle Beine
    std::vector<JointAngles> calculateAllLegsIK() const
    {
        std::vector<JointAngles> allAngles;
        for (int i = 0; i < NUM_LEGS; i++)
        {
            allAngles.push_back(calculateLegIK(i));
        }
        return allAngles;
    }

    // Ausgabe der Gelenkwinkel
    void printLegAngles(int legIndex, const JointAngles &angles) const
    {
        Serial.print("Leg ");
        Serial.print(legIndex);
        Serial.print(": ");
        if (!angles.reachable)
        {
            Serial.println("NICHT ERREICHBAR!");
            return;
        }

        Serial.print(" 0: ");
        Serial.print(angles.theta0 * 180.0 / M_PI);
        Serial.print("° ");
        Serial.print(" 1: ");
        Serial.print(angles.theta1 * 180.0 / M_PI);
        Serial.print("° ");
        Serial.print(" 2: ");
        Serial.print(angles.theta2 * 180.0 / M_PI);
        Serial.println("°");
    }

    bool allLegAnglesAreReachable(const std::vector<JointAngles> &angles) const
    {
        for (int i = 0; i < NUM_LEGS; i++)
        {
            if (!angles[i].reachable || !angles[i].allAnglesInLimit(i, NUM_LEGS))
            {
                return false;
            }
        }
        return true;
    }

    std::vector<Vector3> interpolate(Vector3 start, Vector3 end, int n) const
    {
        std::vector<Vector3> pts;
        pts.reserve(n + 2);

        // lineare Schritte für x und y
        double dx = (end.x - start.x) / (n + 1);
        double dy = (end.y - start.y) / (n + 1);

        // Punkte erzeugen
        for (int i = 0; i <= n + 1; ++i)
        {
            double t = double(i) / double(n + 1);

            Vector3 p = Vector3(start.x + dx * i,
                                start.y + dy * i,
                                0);

            pts.push_back(p);
        }

        return pts;
    }

    std::vector<Vector3> interpolateSinZ(Vector3 start, Vector3 end, int n) const
    {
        std::vector<Vector3> pts;
        pts.reserve(n + 2);

        // lineare Schritte für x und y
        double dx = (end.x - start.x) / (n + 1);
        double dy = (end.y - start.y) / (n + 1);

        // z-Werte sollen sinusförmig von z1 nach z2 verlaufen
        // t läuft von 0..1 — der Sinus moduliert den Verlauf
        auto zCurve = [&](double t)
        {
            // Sinus-Verlauf: 0 -> π
            // erzeugt "Beule" nach oben
            double s = std::sin(t * M_PI);
            return start.z + (end.z - start.z) * s;
        };

        // Punkte erzeugen
        for (int i = 0; i <= n + 1; ++i)
        {
            double t = double(i) / double(n + 1);

            Vector3 p = Vector3(start.x + dx * i,
                                start.y + dy * i,
                                zCurve(t));

            pts.push_back(p);
        }

        return pts;
    }
};

#endif