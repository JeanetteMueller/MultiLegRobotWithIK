/**
 * PentapodKinematics.h
 *
 * C++ Klasse zur Berechnung der inversen Kinematik für einen 5-beinigen Roboter (Pentapod)
 * mit zylindrischem Körper und 3-DOF Beinen.
 *
 * Jedes Bein hat 3 Gelenke:
 *   - θ0 (Hüft-Schwenk): Rotation um Achse senkrecht zum Oberschenkel
 *   - θ1 (Hüft-Heben): Hebt/senkt den Oberschenkel
 *   - θ2 (Knie): Beugt/streckt das Knie
 *
 * Autor: Claude
 * Datum: 2025
 */

#ifndef PentapodKinematics_H
#define PentapodKinematics_H

#include <cmath>
#include <array>
#include <stdexcept>

#include "LegAngles.h"
#include "BodyPose.h"

class PentapodKinematics
{
public:
    static constexpr uint8_t NUM_LEGS = 5;           // Anzahl der Beine
    static constexpr uint8_t WALKING_STEP_COUNT = 6; // Anzahl der interpolierten Schritte für Bewegungsabläufe

    std::array<Vector3, NUM_LEGS> baseFootPositions; // Feste Fußpositionen auf dem Boden
    std::array<Vector3, NUM_LEGS> lastTargetPosition;
    std::array<Vector3, NUM_LEGS> targetPosition;
    uint8_t currentMovingLeg = 0;
    int8_t walkingStep = -1; // Zwischenschritt Nummer des aktuellen Bewegungsablauf

private:
    uint32_t previousStepMillis = 0;

    // Roboter-Geometrie (in mm)
    const float BODY_RADIUS;  // Radius des zylindrischen Körpers
    const float THIGH_LENGTH; // Länge des Oberschenkels (L1)
    const float SHIN_LENGTH;  // Länge des Unterschenkels (L2)

    float walk_x = 0;
    float walk_y = 0;

    BodyPose m_pose;

    static constexpr float rotateCoordinatesByLeg[NUM_LEGS] = {0, 144, -72, 72, -144};

public:
    /**
     * Konstruktor
     */
    PentapodKinematics(
        float bodyRadius,
        float thighLength,
        float shinLength) : BODY_RADIUS(bodyRadius),
                            THIGH_LENGTH(thighLength),
                            SHIN_LENGTH(shinLength)
    {

        float footRadius = BODY_RADIUS + 120.0; // Abstand vom Zentrum

        // Initialisiere Fußpositionen (gleichmäßig verteilt, 72° = 2π/5 versetzt)
        for (uint8_t i = 0; i < NUM_LEGS; i++)
        {
            float angle = i * (2.0 * M_PI / NUM_LEGS);

            baseFootPositions[i] = Vector3(
                cosf(angle) * footRadius,
                0.0, // Boden
                sinf(angle) * footRadius);
        }
        lastTargetPosition = baseFootPositions;
        targetPosition = baseFootPositions;
    }

    /**
     * Bereitet Bewegungsabläufe vor
     */
    void mainLoop()
    {
        if (millis() - previousStepMillis >= 50)
        {
            previousStepMillis = millis();

            walkingStep++;

            if (walkingStep >= WALKING_STEP_COUNT)
            {
                walkingStep = 0;

                lastTargetPosition = targetPosition;

                currentMovingLeg++;

                if (currentMovingLeg >= NUM_LEGS)
                {
                    currentMovingLeg = 0;
                }
            }

            Serial.print("currentMovingLeg: ");
            Serial.print(currentMovingLeg);
            Serial.print(" step: ");
            Serial.println(walkingStep);
        }
    }

    void prepareTargetPositions()
    {
        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            Vector3 newTarget;

            if (walk_x == 0 && walk_y == 0)
            {
                newTarget = baseFootPositions[legIndex];
            }
            else
            {
                newTarget = newTargetPosition(legIndex);
            }
            targetPosition[legIndex] = getStepTargetPosition(legIndex, newTarget);

            // Serial.print("Vector3 newTarget ");
            // Serial.print(newTarget.x);
            // Serial.print(" ");
            // Serial.println(newTarget.z);
        }
    }

    /**
     * Setzt die Körper-Pose
     */
    void setPose(const BodyPose &pose)
    {
        m_pose = pose;
    }

    /**
     * Setzt die Körper-Pose mit einzelnen Parametern (Winkel in Grad)
     */
    void setPose(float height, float footOffset,
                 float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        m_pose = BodyPose::fromDegrees(height, footOffset, tiltXDeg, tiltZDeg, rotYDeg);
    }

    /**
     * Setzt die Koordinaten abweichung fest, in die der Roboter sich bewegen soll
     */
    void setWalkDirection(float x, float y)
    {
        if (currentMovingLeg == 0 && walkingStep == 0)
        {
            // andere richtung und geschwindigkeit immer nur wenn alle Füße am Boden sind
            walk_x = x;
            walk_y = y;
        }
    }

    /**
     * Gibt die aktuelle Pose zurück
     */
    const BodyPose &getPose() const
    {
        return m_pose;
    }

    /**
     * Berechnet die Gelenkwinkel für ein einzelnes Bein
     *
     * @param legIndex Index des Beins (0-4)
     * @return LegAngles Struktur mit den 3 Winkeln
     */
    LegAngles calculateLegAngles(uint8_t legIndex) const
    {
        if (legIndex < 0 || legIndex >= NUM_LEGS)
        {
            Serial.println("Leg index must be 0-4");
        }

        LegAngles result;
        result.valid = false;

        const float baseLegAngle = legIndex * (2.0 * M_PI / NUM_LEGS);
        const float legAngleWithRot = baseLegAngle + m_pose.rotY;

        // Berechne Hüft-Position nach Körper-Rotation
        float hipX, hipY, hipZ;
        calculateHipPosition(legAngleWithRot, hipX, hipY, hipZ);

        // Fuß-Position (bleibt auf dem Boden, basiert auf Basis-Winkel ohne Körperdrehung)
        const float footX = targetPosition[legIndex].x;
        const float footY = targetPosition[legIndex].y;
        const float footZ = targetPosition[legIndex].z;

        // 3-DOF Inverse Kinematik berechnen
        result = solveIK3DOF(legIndex, hipX, hipY, hipZ, footX, footY, footZ, baseLegAngle);

        return result;
    }

    /**
     * Berechnet die Gelenkwinkel für alle 5 Beine
     *
     * @return Array mit 5 LegAngles Strukturen
     */
    std::array<LegAngles, NUM_LEGS> calculateAllLegAngles() const
    {
        std::array<LegAngles, NUM_LEGS> results;

        for (uint8_t i = 0; i < NUM_LEGS; ++i)
        {
            results[i] = calculateLegAngles(i);
        }

        return results;
    }

    /**
     * Prüft ob alle Beine ihre Zielposition erreichen können
     */
    bool isValidPose() const
    {
        for (uint8_t i = 0; i < NUM_LEGS; ++i)
        {
            if (!calculateLegAngles(i).valid)
            {
                return false;
            }
        }
        return true;
    }

private:
    /**
     * Berechnet die Hüft-Position nach Körper-Rotation
     */
    void calculateHipPosition(float legAngleWithRot,
                              float &hipX, float &hipY, float &hipZ) const
    {
        // Basis-Position am Körperrand
        const float baseHipX = BODY_RADIUS * cosf(legAngleWithRot);
        const float baseHipY = 0.0;
        const float baseHipZ = BODY_RADIUS * sinf(legAngleWithRot);

        // Rotation um X-Achse (Pitch)
        hipX = baseHipX;
        hipY = baseHipY * cosf(m_pose.tiltX) - baseHipZ * sinf(m_pose.tiltX);
        hipZ = baseHipY * sinf(m_pose.tiltX) + baseHipZ * cosf(m_pose.tiltX);

        // Rotation um Z-Achse (Roll)
        const float tempX = hipX * cosf(m_pose.tiltZ) - hipY * sinf(m_pose.tiltZ);
        const float tempY = hipX * sinf(m_pose.tiltZ) + hipY * cosf(m_pose.tiltZ);
        hipX = tempX;
        hipY = tempY;

        // Verschiebung zur Körperhöhe
        hipY += m_pose.height;
    }

    /**
     * 3-DOF Inverse Kinematik
     * Das Knie zeigt IMMER nach außen (positive radiale Richtung)
     */
    LegAngles solveIK3DOF(uint8_t legIndex, float hipX, float hipY, float hipZ,
                          float footX, float footY, float footZ,
                          float baseLegAngle) const
    {
        LegAngles result;
        result.valid = false;
        result.swing = 0.0;
        result.lift = 0.0;
        result.knee = 0.0;

        // Vektor von Hüfte zu Fuß
        const float deltaX = footX - hipX;
        const float deltaY = footY - hipY;
        const float deltaZ = footZ - hipZ;

        // Basis-Richtungen
        const float baseRadialX = cosf(baseLegAngle);
        const float baseRadialZ = sinf(baseLegAngle);
        const float tangentX = -sinf(baseLegAngle);
        const float tangentZ = cosf(baseLegAngle);

        // Projiziere auf lokales Koordinatensystem
        const float footRadial = deltaX * baseRadialX + deltaZ * baseRadialZ;
        const float footTangent = deltaX * tangentX + deltaZ * tangentZ;
        const float footVertical = deltaY;

        // Gesamte 3D-Distanz
        const float totalDist = sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

        // Prüfe Erreichbarkeit
        if (totalDist > THIGH_LENGTH + SHIN_LENGTH ||
            totalDist < std::abs(THIGH_LENGTH - SHIN_LENGTH))
        {
            return result; // Nicht erreichbar
        }

        // Kniewinkel mit Kosinussatz (negativ = Knie nach außen)
        const float cosKnee = (totalDist * totalDist - THIGH_LENGTH * THIGH_LENGTH -
                               SHIN_LENGTH * SHIN_LENGTH) /
                              (2.0 * THIGH_LENGTH * SHIN_LENGTH);
        const float kneeAngle = -acosf(fmaxf(-1.0, fminf(1.0, cosKnee)));

        // Horizontale Distanz zum Fuß
        const float horizDist = sqrtf(footRadial * footRadial + footTangent * footTangent);

        // Vorzeichen für horizontale Richtung
        const float horizSign = (footRadial >= 0) ? 1.0 : -1.0;

        // Lift-Winkel berechnen
        const float k1 = THIGH_LENGTH + SHIN_LENGTH * cosf(kneeAngle);
        const float k2 = SHIN_LENGTH * sinf(kneeAngle);
        const float liftAngle = atan2f(footVertical, horizDist * horizSign) - atan2f(k2, k1);

        // Schwenkwinkel
        const float swingAngle = atan2f(footTangent, footRadial);

        result.swing = swingAngle;
        result.lift = liftAngle;
        result.knee = kneeAngle;
        result.valid = result.allAnglesInLimit(legIndex, 5);

        if (!result.valid)
        {
            Serial.print("footX: ");
            Serial.print(footX);

            Serial.print("  footZ: ");
            Serial.println(footZ);
        }

        return result;
    }

    Vector3 newTargetPosition(uint8_t legIndex) const
    {
        Vector3 origin = lastTargetPosition[legIndex];
        Vector3 walkVector = Vector3(walk_x, 0, walk_y);
        Vector3 newTarget = origin;
        float rotation = rotateCoordinatesByLeg[legIndex];
        Vector3 rotated = walkVector.rotate(degToRad(rotation));

        if (currentMovingLeg == legIndex)
        {
            newTarget.x += rotated.x;
            newTarget.z += rotated.z;
        }
        else
        {
            newTarget.x -= rotated.x / (NUM_LEGS - 1);
            newTarget.z -= rotated.z / (NUM_LEGS - 1);
        }
        return newTarget;
    }

    Vector3 getStepTargetPosition(uint8_t legIndex, Vector3 newTarget) const
    {
        Vector3 origin = lastTargetPosition[legIndex];

        if (currentMovingLeg == legIndex)
        {
            if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
            {

                // Serial.print("active leg points: ");
                // Vector3 p0 = interpolateSin(origin, newTarget, 0, 1.5);
                // Serial.print(p0.x); Serial.print(" ");
                // Vector3 p1 = interpolateSin(origin, newTarget, 1, 1.5);
                // Serial.print(p1.x); Serial.print(" ");
                // Vector3 p2 = interpolateSin(origin, newTarget, 2, 1.5);
                // Serial.print(p2.x); Serial.print(" ");
                // Vector3 p3 = interpolateSin(origin, newTarget, 3, 1.5);
                // Serial.print(p3.x); Serial.print(" ");
                // Vector3 p4 = interpolateSin(origin, newTarget, 4, 1.5);
                // Serial.print(p4.x); Serial.print(" ");
                // Vector3 p5 = interpolateSin(origin, newTarget, 5, 1.5);
                // Serial.println(p5.x);

                return interpolateSin(origin, newTarget, walkingStep, 1.5);
            }
        }
        else
        {

            if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
            {
                return interpolateSin(origin, newTarget, walkingStep, 0);
            }
        }
        return origin;
    }

    Vector3 interpolateSin(Vector3 start, Vector3 end, uint8_t walkingStep, float curveMultiplier) const
    {
        if (walkingStep == 0)
        {
            return start;
        }
        else if (walkingStep == WALKING_STEP_COUNT - 1)
        {
            return end;
        }

        // lineare Schritte für x und y
        const float dx = (end.x - start.x) / (WALKING_STEP_COUNT - 1);
        const float dy = (end.y - start.y) / (WALKING_STEP_COUNT - 1);
        const float dz = (end.z - start.z) / (WALKING_STEP_COUNT - 1);

        float yAddition = 0;

        if (curveMultiplier > 0)
        {
            // abstand zwischen start und ende
            float d = distance(start.x, start.z, end.x, end.z);

            float liftLeg = 0;
            if (d > 0)
            {
                liftLeg = sinCurve(d, walkingStep);
                yAddition = (liftLeg * d * curveMultiplier);
            }
        }

        return Vector3(start.x + dx * walkingStep,
                       (start.y + dy * walkingStep) + yAddition,
                       start.z + dz * walkingStep);
    }

    float distance(float x1, float y1, float x2, float y2) const
    {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return sqrtf(dx * dx + dy * dy);
    }

    float sinCurve(float distanz, uint8_t walkingStep) const
    {
        if (walkingStep == 0)
        {
            return 0;
        }
        else if (walkingStep == WALKING_STEP_COUNT - 1)
        {
            return 0;
        }

        float amplitude = 1.0;
        // Position entlang der Distanz (0 bis distanz)
        float position = (distanz / (WALKING_STEP_COUNT - 1)) * walkingStep;

        // Winkel für Sinusfunktion (0 bis π, damit bei 0 startet und endet)
        float winkel = M_PI * position / distanz;

        // Sinuswert berechnen
        return amplitude * sinf(winkel);
    }

    float degToRad(float deg) const
    {
        return deg * M_PI / 180.0f;
    }
};

#endif // PENTAPOD_KINEMATICS_H
