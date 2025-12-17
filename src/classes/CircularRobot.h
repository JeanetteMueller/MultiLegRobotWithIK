
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

    const int walkingStepCount = 5;

    double walk_x = 0;
    double walk_y = 0;
    double walkingLegLift = 80.0;

public:
    std::vector<Vector3> lastTargetPosition;
    std::vector<Vector3> targetPosition;
    int currentMovingLeg = 0;
    int walkingStep = -1;

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
            double footRadius = BODY_RADIUS + 120.0 + 40.0; // Abstand vom Zentrum
            baseFootPositions.push_back(Vector3(
                std::cos(angle) * footRadius,
                0.0, // Boden
                std::sin(angle) * footRadius));
        }
        lastTargetPosition = baseFootPositions;
        targetPosition = baseFootPositions;
    }

    void setWalkDirection(double x, double y)
    {
        if (walkingStep == 0)
        {
            // andere richtung und geschwindigkeit immer nur wenn alle Füße am Boden sind
            walk_x = x;
            walk_y = y;
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

        if (currentMillis - previousStepMillis >= 300)
        {
            previousStepMillis = currentMillis;

            walkingStep++;

            if (walkingStep > walkingStepCount)
            {
                walkingStep = 0;

                currentMovingLeg++;

                if (currentMovingLeg >= NUM_LEGS)
                {
                    currentMovingLeg = 0;

                    lastTargetPosition = targetPosition;
                }
            }

            // Serial.print("currentMovingLeg: ");
            // Serial.print(currentMovingLeg);
            // Serial.print(" step: ");
            // Serial.println(walkingStep);

            for (int i = 0; i < NUM_LEGS; i++)
            {

                Vector3 newTarget = getStepTargetPosition(i);
                targetPosition[i] = newTarget;
            }
        }
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
        Vector3 origin = lastTargetPosition[legIndex];

        if (currentMovingLeg == legIndex)
        {
            Vector3 newTarget = baseFootPositions[legIndex];
            if (walk_x != 0.0)
            {
                newTarget.x += (walk_x / 2) / NUM_LEGS;
            }
            if (walk_y != 0.0)
            {
                newTarget.z += (walk_y / 2) / NUM_LEGS;
            }

            if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
            {
                std::vector<Vector3> walkingWay = interpolateSin(origin, newTarget, walkingStepCount);

                return walkingWay[walkingStep];
            }
            return origin;
        }
        else
        {
            Vector3 newTarget = baseFootPositions[legIndex];
            if (walk_x != 0.0)
            {
                newTarget.x -= (walk_x / 2) / NUM_LEGS;
            }
            if (walk_y != 0.0)
            {
                newTarget.z -= (walk_y / 2) / NUM_LEGS;
            }

            if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
            {
                std::vector<Vector3> walkingWay = interpolate(origin, newTarget, walkingStepCount);

                return walkingWay[walkingStep];
            }
            return origin;
        }
    }

    // Inverse Kinematik für ein Bein mit Kugelgelenk
    JointAngles calculateLegIK(int legIndex) const
    {
        JointAngles angles;

        // Basis-Position des Beins
        Vector3 basePos = getLegOriginPosition(legIndex, NUM_LEGS);

        // Basiswinkel des Beins (radiale Richtung vom Zentrum)
        double legBaseAngle = (legIndex / (double)NUM_LEGS) * 2.0 * M_PI;

        // Ziel-Position (Fußpunkt)
        Vector3 footPos = targetPosition[legIndex];

        // Vektor von Basis zum Fuß
        Vector3 toFoot = footPos - basePos;

        // Theta1: Horizontale Rotation des zweiten Gelenks (links/rechts)
        double absoluteAngle = std::atan2(toFoot.z, toFoot.x);
        angles.theta1 = absoluteAngle - legBaseAngle;

        // Normalisiere auf [-PI, PI]
        angles.theta1 = std::atan2(std::sin(angles.theta1), std::cos(angles.theta1));

        // Projiziere auf die Ebene die durch theta1 definiert wird
        double horizontalDist = std::sqrt(toFoot.x * toFoot.x + toFoot.z * toFoot.z);

        // Lokale Koordinaten nach theta1-Rotation
        double localX = horizontalDist * std::cos(angles.theta1);
        double localY = toFoot.y;

        // 2D IK mit lokalen Koordinaten
        double distToTarget = std::sqrt(localX * localX + localY * localY);

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
        cosTheta2 = std::max(-1.0, std::min(1.0, cosTheta2));
        angles.theta2 = std::acos(cosTheta2);

        // Theta0: Vertikale Rotation (hoch/runter)
        double alpha = std::atan2(-localY, localX);

        double cosBeta = (l2 * l2 + distToTarget * distToTarget - l3 * l3) / (2.0 * l2 * distToTarget);
        cosBeta = std::max(-1.0, std::min(1.0, cosBeta));
        double beta = std::acos(cosBeta);

        angles.theta0 = alpha - beta;

        if (angles.theta1 * 180.0 / M_PI == 180.0)
        {
            angles.theta1 = 0;
        }

        angles.theta0 = angles.theta0 * 180.0 / M_PI;
        angles.theta1 = angles.theta1 * 180.0 / M_PI;
        angles.theta2 = angles.theta2 * 180.0 / M_PI;

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

        Serial.print(" 0: ");
        Serial.print(angles.theta0);
        Serial.print("° ");
        Serial.print(" 1: ");
        Serial.print(angles.theta1);
        Serial.print("° ");
        Serial.print(" 2: ");
        Serial.print(angles.theta2);

        if (!angles.reachable)
        {
            Serial.print("NICHT ERREICHBAR!");
        }

        Serial.println("°");
    }

    int allLegAnglesAreReachable(const std::vector<JointAngles> &angles) const
    {
        for (int i = 0; i < NUM_LEGS; i++)
        {
            if (!angles[i].reachable || !angles[i].allAnglesInLimit(i, NUM_LEGS))
            {
                return i;
            }
        }
        return -1;
    }

    std::vector<Vector3> interpolate(Vector3 start, Vector3 end, int n) const
    {
        std::vector<Vector3> pts;
        pts.reserve(n);

        pts.push_back(start);

        // lineare Schritte für x und y
        double dx = (end.x - start.x) / (n - 1);
        double dy = (end.y - start.y) / (n - 1);
        double dz = (end.z - start.z) / (n - 1);

        // Punkte erzeugen
        for (int i = 1; i < n; ++i)
        {
            Vector3 p = Vector3(start.x + dx * i,
                                start.y + dy * i,
                                start.z + dz * i);

            pts.push_back(p);
        }

        pts.push_back(end);

        return pts;
    }

    std::vector<Vector3> interpolateSin(Vector3 start, Vector3 end, int n) const
    {
        std::vector<Vector3> pts;
        pts.reserve(n);

        pts.push_back(start);

        // lineare Schritte für x und y
        double dx = (end.x - start.x) / (n - 1);
        double dy = (end.y - start.y) / (n - 1);
        double dz = (end.z - start.z) / (n - 1);

        // z-Werte sollen sinusförmig von z1 nach z2 verlaufen
        // t läuft von 0..1 — der Sinus moduliert den Verlauf

        // abstand zwischen start und ende
        double d = distance(start.x, start.z, end.x, end.z);

        // erzeuge kurvenpunkte
        std::vector<double> curve = sinCurve(d, n);

        // sinus kurve lift distance
        double liftDistance = walkingLegLift;

        // Punkte erzeugen
        for (int i = 1; i < n - 1; ++i)
        {
            double liftLeg = 0;
            if (d > 0)
            {
                liftLeg = curve[i];
            }

            Vector3 p = Vector3(start.x + dx * i,
                                (start.y + dy * i) + (liftLeg * liftDistance),
                                start.z + dz * i);

            pts.push_back(p);
        }
        pts.push_back(end);

        return pts;
    }

    double distance(double x1, double y1, double x2, double y2) const
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    std::vector<double> sinCurve(double distanz, int schritte, double amplitude = 1.0) const
    {
        std::vector<double> kurve;
        kurve.reserve(schritte);
        kurve.push_back(0);

        for (int i = 1; i < schritte - 1; i++)
        {
            // Position entlang der Distanz (0 bis distanz)
            double position = (distanz / (schritte - 1)) * i;

            // Winkel für Sinusfunktion (0 bis π, damit bei 0 startet und endet)
            double winkel = M_PI * position / distanz;

            // Sinuswert berechnen
            double wert = amplitude * std::sin(winkel);

            kurve.push_back(wert);
        }

        return kurve;
    }
};

#endif