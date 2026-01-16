/**
 * RobotWithKinematics.h
 *
 * C++ Klasse zur Berechnung der inversen Kinematik für einen 4-8 beinigen Roboter
 * mit zylindrischem Körper und 3-DOF Beinen.
 *
 * Jedes Bein hat 3 Gelenke:
 *   - θ0 (Hüft-Schwenk): Rotation um Achse senkrecht zum Oberschenkel
 *   - θ1 (Hüft-Heben): Hebt/senkt den Oberschenkel
 *   - θ2 (Knie): Beugt/streckt das Knie
 *
 * Autor: Claude.ai & Jeanette Müller
 * Datum: 2025
 */

#ifndef RobotWithKinematics_H
#define RobotWithKinematics_H

#include <cmath>
#include <array>
#include <stdexcept>

#include "LegAngles.h"
#include "BodyPose.h"

class RobotWithKinematics
{
public:
    uint8_t NUM_LEGS = 5; // Anzahl der Beine
    static constexpr uint8_t MAX_NUM_LEGS = 8;
    static constexpr uint8_t WALKING_STEP_COUNT = 20; // Anzahl der interpolierten Schritte für Bewegungsabläufe

    std::array<Vector3, MAX_NUM_LEGS> baseFootPositions; // Feste Fußpositionen auf dem Boden
    std::array<Vector3, MAX_NUM_LEGS> lastTargetPosition;
    std::array<Vector3, MAX_NUM_LEGS> targetPosition;

    uint8_t currentMovingLeg = 0;
    int8_t walkingStep = -1; // Zwischenschritt Nummer des aktuellen Bewegungsablauf

private:
    uint32_t previousStepMillis = 0;

    // Roboter-Geometrie (in mm)
    const float BODY_RADIUS; // Radius des zylindrischen Körpers

    const float COXA_LENGTH;  // Länge der Coxa (L0)
    const float FEMUR_LENGTH; // Länge des Oberschenkels (L1)
    const float TIBIA_LENGTH; // Länge des Unterschenkels (L2)
    float baseFootExtend;

    bool allMovesDone = true;

    float walk_x = 0;
    float walk_y = 0;
    float rotate_Body = 0;

    BodyPose m_pose;

    static constexpr float rotateCoordinatesByLeg[MAX_NUM_LEGS] = {0, -144, 72, -72, 144, 0, 0, 0};

    std::array<float, MAX_NUM_LEGS> m_legBaseAngles; // Basis-Winkel der Beine (72° versetzt)
public:
    /**
     * Konstruktor
     */
    RobotWithKinematics(
        float bodyRadius,
        uint8_t numberOfLegs,
        float coxaLength,
        float thighLength,
        float shinLength,
        float baseFootExtend) : BODY_RADIUS(bodyRadius),
                                NUM_LEGS(numberOfLegs),
                                COXA_LENGTH(coxaLength),
                                FEMUR_LENGTH(thighLength),
                                TIBIA_LENGTH(shinLength),
                                baseFootExtend(baseFootExtend)
    {
        float footRadius = BODY_RADIUS + baseFootExtend; // Abstand vom Zentrum

        // Initialisiere Fußpositionen (gleichmäßig verteilt, 72° = 2π/5 versetzt)
        for (uint8_t i = 0; i < NUM_LEGS; i++)
        {
            m_legBaseAngles[i] = i * (2.0 * M_PI / NUM_LEGS);

            baseFootPositions[i] = Vector3(
                cosf(m_legBaseAngles[i]) * footRadius,
                0.0, // Boden
                sinf(m_legBaseAngles[i]) * footRadius);
        }
        lastTargetPosition = baseFootPositions;
        targetPosition = baseFootPositions;
    }

    /**
     * Bereitet Bewegungsabläufe vor
     */
    void mainLoop()
    {
        if (millis() - previousStepMillis >= 12)
        {
            if (currentMovingLeg == 0 &&
                walkingStep == 0 &&
                (walk_x < 5 && walk_x > -5) &&
                (walk_y < 5 && walk_y > -5) &&
                (rotate_Body < 5 && rotate_Body > -5) &&
                allMovesDone == true)
            {
                // Serial.println("NO MOVEMENT AT ALL");
                return;
            }

            previousStepMillis = millis();

            walkingStep++;

            if (walkingStep >= WALKING_STEP_COUNT)
            {
                walkingStep = 0;

                lastTargetPosition = targetPosition;

                currentMovingLeg += 2;

                if (currentMovingLeg >= NUM_LEGS)
                {
                    currentMovingLeg = currentMovingLeg - NUM_LEGS;
                }
            }

            Serial.print("currentMovingLeg: ");
            Serial.print(currentMovingLeg);
            Serial.print("  step: ");
            Serial.print(walkingStep);

            Serial.print("  allMovesDone: ");
            Serial.println(allMovesDone);
        }
    }

    /**
     * Basis-Gelenkwerte für den Robot in transportfähigem Zustand
     */
    std::array<LegAngles, MAX_NUM_LEGS> standBy() const
    {
        std::array<LegAngles, MAX_NUM_LEGS> results;

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            LegAngles la;
            la.valid = true;
            la.coxa = degToRad(0);
            la.femur = degToRad(110);
            la.tibia = degToRad(80 - legIndex * (10 - legIndex));

            results[legIndex] = la;
        }
        return results;
    }

    /**
     * Gelenkwerte für den Übergang zwischen Standby und regulärem verarbeiten der Input Daten
     */
    std::array<LegAngles, MAX_NUM_LEGS> preOperationPositions() const
    {
        std::array<LegAngles, MAX_NUM_LEGS> results;

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            LegAngles la;
            la.valid = true;
            la.coxa = degToRad(0);
            la.femur = degToRad(90.24);
            la.tibia = degToRad(-132.75);

            results[legIndex] = la;
        }
        return results;
    }

    /**
     * Setze alle Zielkoordinaten zurück
     */
    void resetTargetPositions()
    {
        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            targetPosition[legIndex] = baseFootPositions[legIndex];
        }
    }

    /**
     * Zielkoordinaten vorbereiten bevor der Lauf-Loop starten kann
     */
    void prepareTargetPositions()
    {
        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            Vector3 newTarget;

            if (walk_x < 5 && walk_x > -5 &&
                walk_y < 5 && walk_y > -5 &&
                rotate_Body < 5 && rotate_Body > -5)
            {
                newTarget = baseFootPositions[legIndex];

                if (legIndex == currentMovingLeg)
                {
                    targetPosition[legIndex] = getStepTargetPosition(legIndex, newTarget);
                }

                if (currentMovingLeg == 0 && walkingStep == 0)
                {
                    allMovesDone = true;
                }
            }
            else
            {
                newTarget = newTargetPosition(legIndex);

                targetPosition[legIndex] = getStepTargetPosition(legIndex, newTarget);
            }
        }
    }

    /**
     * Setzt den Basis-Abstand zwischen dem Zentrum des Robots und den Füßen
     */
    void setBaseFootExtend(float newValue)
    {
        if (currentMovingLeg == 0 && walkingStep == 0)
        {
            if (baseFootExtend != newValue)
            {
                Serial.println("setBaseFootExtend");
                float footRadius = BODY_RADIUS + newValue; // Abstand vom Zentrum

                for (uint8_t i = 0; i < NUM_LEGS; i++)
                {
                    float angle = i * (2.0 * M_PI / NUM_LEGS);

                    baseFootPositions[i] = Vector3(
                        cosf(angle) * footRadius,
                        0.0, // Boden
                        sinf(angle) * footRadius);
                }

                baseFootExtend = newValue;
                allMovesDone = false;
            }
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
    void setPose(float height,
                 float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        m_pose = BodyPose::fromDegrees(height, tiltXDeg, tiltZDeg, rotYDeg);
    }

    /**
     * Setzt die Koordinaten abweichung fest, in die der Roboter sich bewegen soll
     */
    void setWalkDirection(float x, float y, float r)
    {
        if (currentMovingLeg == 0 && walkingStep == 0)
        {
            // andere richtung und geschwindigkeit immer nur wenn alle Füße am Boden sind
            if (walk_x != x || walk_y != y || rotate_Body != r)
            {
                walk_x = x;
                walk_y = y;
                rotate_Body = r;

                allMovesDone = false;
            }
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
     * Neigt den Körper so, dass das gewählte Bein am höchsten Punkt ist.
     *
     * @param legIndex Index des Beins (0-4), das am höchsten sein soll
     * @param tiltAngleDeg Neigungswinkel in Grad (positiv = Bein wird angehoben)
     */
    void tiltTowardsLeg(int legIndex, double tiltAngleDeg)
    {
        if (legIndex < 0 || legIndex >= NUM_LEGS)
        {
            throw std::out_of_range("Leg index must be 0-4");
        }

        const double tiltAngle = tiltAngleDeg * M_PI / 180.0;
        const double legAngle = m_legBaseAngles[legIndex];

        // Die Neigungsachse steht senkrecht zur Beinrichtung
        m_pose.tiltX = -tiltAngle * std::sin(legAngle);
        m_pose.tiltZ = tiltAngle * std::cos(legAngle);
    }

    /**
     * Neigt den Körper in eine beliebige Richtung.
     *
     * @param directionDeg Richtung der Neigung in Grad (0° = Bein 0 Richtung)
     * @param tiltAngleDeg Neigungswinkel in Grad
     */
    void tiltInDirection(double directionDeg, double tiltAngleDeg)
    {
        const double direction = directionDeg * M_PI / 180.0;
        const double tiltAngle = tiltAngleDeg * M_PI / 180.0;

        m_pose.tiltX = -tiltAngle * std::sin(direction);
        m_pose.tiltZ = tiltAngle * std::cos(direction);
    }

    /**
     * Setzt Neigung zu einem Bein und behält andere Parameter bei.
     */
    void setTiltTowardsLeg(int legIndex, double tiltAngleDeg)
    {
        double height = m_pose.height;
        double rotY = m_pose.rotY;

        tiltTowardsLeg(legIndex, tiltAngleDeg);

        m_pose.height = height;
        m_pose.rotY = rotY;
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
            Serial.println(" ->      Leg index must be 0-4");
        }

        LegAngles result;
        result.valid = false;
        result.coxa = 0.0;
        result.femur = 0.0;
        result.tibia = 0.0;

        const float baseLegAngle = m_legBaseAngles[legIndex];
        const float legAngleWithRot = baseLegAngle + m_pose.rotY;

        // Berechne Coxa-Basis Position nach Körper-Rotation
        float coxaBaseX, coxaBaseY, coxaBaseZ;
        calculateCoxaBasePosition(legAngleWithRot, coxaBaseX, coxaBaseY, coxaBaseZ);

        // Fuß-Position (bleibt auf dem Boden, basiert auf Basis-Winkel ohne Körperdrehung)
        // // Der Fuß ist radial vom Körper: BODY_RADIUS + COXA_LENGTH + footOffset
        // const float footRadialDist = BODY_RADIUS + COXA_LENGTH + baseFootExtend;
        // const float footX = footRadialDist * cosf(baseLegAngle);
        // const float footY = 0.0;
        // const float footZ = footRadialDist * sinf(baseLegAngle);
        const float footX = targetPosition[legIndex].x;
        const float footY = targetPosition[legIndex].y;
        const float footZ = targetPosition[legIndex].z;

        // 3-DOF Inverse Kinematik berechnen
        result = solveIK3DOF(legIndex, coxaBaseX, coxaBaseY, coxaBaseZ,
                             footX, footY, footZ, legAngleWithRot);

        return result;
    }

    /**
     * Berechnet die Gelenkwinkel für alle 5 Beine
     *
     * @return Array mit 5 LegAngles Strukturen
     */
    std::array<LegAngles, MAX_NUM_LEGS> calculateAllLegAngles() const
    {
        std::array<LegAngles, MAX_NUM_LEGS> results;

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; ++legIndex)
        {
            results[legIndex] = calculateLegAngles(legIndex);
        }

        return results;
    }

    /**
     * Prüft ob alle Beine ihre Zielposition erreichen können
     */
    bool isValidPose() const
    {
        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; ++legIndex)
        {
            if (!calculateLegAngles(legIndex).valid)
            {
                return false;
            }
        }
        return true;
    }
    /**
     * Gibt den Basis-Winkel eines Beins zurück (in Radiant)
     */
    float getLegBaseAngle(int legIndex) const
    {
        if (legIndex < 0 || legIndex >= NUM_LEGS)
        {
            Serial.println(" ->       Leg index must be 0-4");
        }
        return m_legBaseAngles[legIndex];
    }

    /**
     * Gibt die maximale Reichweite eines Beins zurück (Coxa + Femur + Tibia)
     */
    float getMaxReach() const
    {
        return COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH;
    }

    /**
     * Gibt die minimale Reichweite eines Beins zurück
     */
    float getMinReach() const
    {
        return COXA_LENGTH + fabsf(FEMUR_LENGTH - TIBIA_LENGTH);
    }

private:
    /**
     * Berechnet die Coxa-Basis Position nach Körper-Rotation
     */
    void calculateCoxaBasePosition(float legAngleWithRot,
                                   float &coxaBaseX, float &coxaBaseY, float &coxaBaseZ) const
    {
        // Basis-Position am Körperrand
        coxaBaseX = BODY_RADIUS * cosf(legAngleWithRot);
        coxaBaseY = 0.0;
        coxaBaseZ = BODY_RADIUS * sinf(legAngleWithRot);

        // Rotation um X-Achse (Pitch)
        float tempY = coxaBaseY * cosf(m_pose.tiltX) - coxaBaseZ * sinf(m_pose.tiltX);
        float tempZ = coxaBaseY * sinf(m_pose.tiltX) + coxaBaseZ * cosf(m_pose.tiltX);
        coxaBaseY = tempY;
        coxaBaseZ = tempZ;

        // Rotation um Z-Achse (Roll)
        float tempX = coxaBaseX * cosf(m_pose.tiltZ) - coxaBaseY * sinf(m_pose.tiltZ);
        tempY = coxaBaseX * sinf(m_pose.tiltZ) + coxaBaseY * cosf(m_pose.tiltZ);
        coxaBaseX = tempX;
        coxaBaseY = tempY;

        // Verschiebung zur Körperhöhe
        coxaBaseY += m_pose.height;
    }

    /**
     * 3-DOF Inverse Kinematik für Coxa-Femur-Tibia Konfiguration
     *
     * Die Coxa zeigt IMMER nach außen (in baseLegAngle Richtung)
     * Nur bei seitlicher Verschiebung (durch Körperdrehung) dreht sie sich
     */
    LegAngles solveIK3DOF(uint8_t legIndex, float coxaBaseX, float coxaBaseY, float coxaBaseZ,
                          float footX, float footY, float footZ,
                          float baseLegAngle) const
    {
        LegAngles result;
        result.valid = false;
        result.coxa = 0.0;
        result.femur = 0.0;
        result.tibia = 0.0;

        // Vektor von Coxa-Basis zu Fuß
        const float deltaX = footX - coxaBaseX;
        const float deltaY = footY - coxaBaseY;
        const float deltaZ = footZ - coxaBaseZ;

        // Basis-Richtung (radial nach außen)
        const float baseRadialX = cosf(baseLegAngle);
        const float baseRadialZ = sinf(baseLegAngle);

        // Tangentiale Richtung
        const float tangentX = -sinf(baseLegAngle);
        const float tangentZ = cosf(baseLegAngle);

        // Projiziere Fuß-Position auf lokales Koordinatensystem
        const float footRadial = deltaX * baseRadialX + deltaZ * baseRadialZ;
        const float footTangent = deltaX * tangentX + deltaZ * tangentZ;

        // Coxa-Winkel: nur für seitliche Abweichung (tangential)
        // Die Coxa zeigt primär nach außen, dreht sich aber für seitliche Korrektur
        float coxaAngle = atan2f(footTangent, fmaxf(footRadial, 0.001));

        // Normalisiere auf -π bis +π
        while (coxaAngle > M_PI)
            coxaAngle -= 2.0 * M_PI;
        while (coxaAngle < -M_PI)
            coxaAngle += 2.0 * M_PI;

        // Effektive Coxa-Richtung nach Drehung
        const float effCoxaAngle = baseLegAngle + coxaAngle;

        // Richtung der Coxa (nach außen)
        const float coxaDirX = cosf(effCoxaAngle);
        const float coxaDirZ = sinf(effCoxaAngle);

        // Position am Ende der Coxa
        const float coxaEndX = coxaBaseX + COXA_LENGTH * coxaDirX;
        const float coxaEndY = coxaBaseY; // Coxa ist horizontal
        const float coxaEndZ = coxaBaseZ + COXA_LENGTH * coxaDirZ;

        // Vektor von Coxa-Ende zu Fuß
        const float dx = footX - coxaEndX;
        const float dy = footY - coxaEndY;
        const float dz = footZ - coxaEndZ;

        // Projiziere auf die Coxa-Richtung (radial vom Coxa-Ende aus gesehen)
        // Positiv = nach außen, Negativ = zurück zum Körper
        const float footRadialFromCoxa = dx * coxaDirX + dz * coxaDirZ;

        // Gesamte horizontale Distanz
        const float legPlaneHoriz = sqrtf(dx * dx + dz * dz);
        const float legPlaneDist = sqrtf(legPlaneHoriz * legPlaneHoriz + dy * dy);

        // Prüfe Erreichbarkeit
        if (legPlaneDist > FEMUR_LENGTH + TIBIA_LENGTH ||
            legPlaneDist < fabsf(FEMUR_LENGTH - TIBIA_LENGTH))
        {
            return result; // Nicht erreichbar
        }

        // 2D IK für Femur und Tibia mit Kosinussatz
        const float cosKnee = (legPlaneDist * legPlaneDist -
                               FEMUR_LENGTH * FEMUR_LENGTH -
                               TIBIA_LENGTH * TIBIA_LENGTH) /
                              (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);

        // Tibia-Winkel (negativ = Knie nach außen gebeugt)
        const float tibiaAngle = -acosf(fmaxf(-1.0, fminf(1.0, cosKnee)));

        // Femur-Winkel
        const float k1 = FEMUR_LENGTH + TIBIA_LENGTH * cosf(tibiaAngle);
        const float k2 = TIBIA_LENGTH * sinf(tibiaAngle);

        // Wenn der Fuß zurück zum Körper zeigt, muss die horizontale Distanz negativ sein
        const float signedHoriz = (footRadialFromCoxa >= 0) ? legPlaneHoriz : -legPlaneHoriz;
        const float femurAngle = atan2f(dy, signedHoriz) - atan2f(k2, k1);

        result.coxa = coxaAngle;
        result.femur = femurAngle;
        result.tibia = tibiaAngle;
        result.valid = result.allAnglesInLimit(legIndex);

        return result;
    }

    Vector3 newTargetPosition(uint8_t legIndex) const
    {
        Vector3 origin = lastTargetPosition[legIndex];

        float x = walk_x;
        float y = walk_y;
        float r = rotate_Body;

        Vector3 walkVector = Vector3(x, 0, y);
        Vector3 newTarget = origin;
        float rotation = rotateCoordinatesByLeg[legIndex];
        Vector3 rotated = walkVector.rotate(degToRad(rotation));

        // zur hälfte im negativen, zur hälfte im positiven um die vole brandbreite der Beine in der Bewegung zu nutzen
        // rotated.x = rotated.x / 2 - rotated.x;
        // rotated.y = rotated.y / 2 - rotated.y;

        rotated.x = -rotated.x;

        if (legIndex == 0) {
            rotated.z += r;
        } else if (legIndex == 1) {
            rotated.x -= r;
        } else if (legIndex == 2) {
            rotated.z -= r;
        } else if (legIndex == 3) {
            rotated.z -= r;
        } else if (legIndex == 4) {
            rotated.x += r;
        } 

        float numberOfLegs = static_cast<float>(NUM_LEGS);

        if (currentMovingLeg == legIndex)
        {
            // Gesamtbewegung durch 5 mal 4 da ein Bein Schreitet während andere nur schieben
            newTarget.x -= rotated.x / numberOfLegs * (numberOfLegs - 1);
            newTarget.z -= rotated.z / numberOfLegs * (numberOfLegs - 1);
        }
        else
        {
            // Gesamtbewegung durch die Anzahl der Beine
            newTarget.x += rotated.x / numberOfLegs;
            newTarget.z += rotated.z / numberOfLegs;
        }
        return newTarget;
    }

    Vector3 getStepTargetPosition(uint8_t legIndex, Vector3 newTarget) const
    {
        Vector3 origin = lastTargetPosition[legIndex];
        if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
        {
            float curveMultiplier = 0.0;

            if (currentMovingLeg == legIndex)
            {
                curveMultiplier = 1.1;
            }

            return interpolateSin(origin, newTarget, walkingStep, curveMultiplier);
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

                if (yAddition < 10)
                {
                    yAddition = 10.0;
                }
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

#endif // RobotWithKinematics_H
