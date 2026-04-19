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
    static constexpr uint8_t WALKING_STEP_COUNT = 70; // Anzahl der interpolierten Schritte für Bewegungsabläufe

    std::array<Vector3, MAX_NUM_LEGS> baseFootPositions; // Feste Fußpositionen auf dem Boden
    std::array<Vector3, MAX_NUM_LEGS> lastTargetPosition;
    std::array<Vector3, MAX_NUM_LEGS> targetPosition;

    uint8_t currentMovingLegA = 0;
    uint8_t currentMovingLegB = 0;
    int8_t walkingStep = 0; // Zwischenschritt Nummer des aktuellen Bewegungsablauf

    enum WalkState : uint8_t
    {
        Walk_Idle = 0, // steht still, Laufzyklus eingefroren bei step=0, phase=0
        Walk_Active,   // Laufzyklus läuft
        Walk_Stopping  // Input losgelassen, Zyklus noch sauber zu Ende fahren
    };
    WalkState walkState = Walk_Idle;

    // ---- Special Pose State Machine ----
    enum SpecialPoseState : uint8_t
    {
        SP_Idle = 0,   // keine Sonderpose aktiv
        SP_MovingInA,  // Bein A fährt in die Sonderposition
        SP_MovingInB,  // Bein B fährt in die Sonderposition
        SP_Holding,    // Sonderpose wird gehalten
        SP_MovingOutA, // Bein A fährt zurück in die Ausgangsposition
        SP_MovingOutB  // Bein B fährt zurück in die Ausgangsposition
    };
    SpecialPoseState specialPoseState = SP_Idle;
    uint8_t activeSpecialPose = 0;     // welche Sonderpose (0..3)
    uint8_t specialPoseLegA = 2;       // welches Bein zuerst
    uint8_t specialPoseLegB = 3;       // welches Bein als zweites
    bool specialPoseRequested = false; // wird jeden Loop von doSpecialPose() gesetzt
    int8_t specialPoseStep = 0;        // Interpolationsschritt 0..WALKING_STEP_COUNT-1
    uint32_t previousSpecialStepMillis = 0;

    // Start-/Zielposition für die aktuelle Teilbewegung (Vector3, in Welt-Koordinaten)
    Vector3 specialMoveStart;
    Vector3 specialMoveTarget;
    uint8_t specialMovingLeg = 0; // das Bein, das sich gerade bewegt

private:
    uint32_t previousStepMillis = 0;

    // Roboter-Geometrie (in mm)
    const float BODY_RADIUS; // Radius des zylindrischen Körpers

    const float COXA_LENGTH;  // Länge der Coxa (L0)
    const float FEMUR_LENGTH; // Länge des Oberschenkels (L1)
    const float TIBIA_LENGTH; // Länge des Unterschenkels (L2)

    float baseFootExtend;
    float maxStepWidth;
    float stepSmoothness = 1.0f; // 0.0 = linear, 1.0 = smoother
    float walk_x = 0;
    float walk_y = 0;
    float rotate_Body = 0;

    uint8_t currentPhase = 0;

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
        float baseFootExtend,
        float maxStepWidth) : BODY_RADIUS(bodyRadius),
                              NUM_LEGS(numberOfLegs),
                              COXA_LENGTH(coxaLength),
                              FEMUR_LENGTH(thighLength),
                              TIBIA_LENGTH(shinLength),
                              baseFootExtend(baseFootExtend),
                              maxStepWidth(maxStepWidth)
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
        // Zustandsübergänge: auf Input-Änderung reagieren
        switch (walkState)
        {
        case Walk_Idle:
            // Im Idle steht der Zyklus bei step=0, phase=0. Losfahren wenn Input kommt.
            if (hasWalkInput())
            {
                walkState = Walk_Active;
                previousStepMillis = millis(); // sauber starten
            }
            else
            {
                // nichts tun, Uhr ruht
                return;
            }
            break;

        case Walk_Active:
            if (!hasWalkInput())
            {
                walkState = Walk_Stopping;
            }
            break;

        case Walk_Stopping:
            // Input kommt wieder rein bevor Zyklus fertig ist -> zurück in Active
            if (hasWalkInput())
            {
                walkState = Walk_Active;
            }
            break;
        }

        // Zeit-Takt
        if (millis() - previousStepMillis < 4)
        {
            return;
        }
        previousStepMillis = millis();

        walkingStep++;

        if (walkingStep >= WALKING_STEP_COUNT)
        {
            walkingStep = 0;
            lastTargetPosition = targetPosition;

            currentPhase++;
            if (currentPhase >= NUM_LEGS)
            {
                currentPhase = 0;
            }

            currentMovingLegA += 1;
            currentMovingLegB = currentMovingLegA + 2;
            if (currentMovingLegA >= NUM_LEGS)
                currentMovingLegA -= NUM_LEGS;
            if (currentMovingLegB >= NUM_LEGS)
                currentMovingLegB -= NUM_LEGS;

            // Zyklus ist an einem sauberen Nullpunkt.
            // Wenn wir gerade am Stoppen sind -> jetzt in Idle wechseln.
            if (walkState == Walk_Stopping && currentPhase == 0)
            {
                walkState = Walk_Idle;
            }
        }
    }

    /**
     * true wenn aktuell Input vorliegt, der tatsächlich Bewegung auslösen soll.
     */
    bool hasWalkInput() const
    {
        float minSpeed = maxStepWidth / 3;
        if (walk_x >= minSpeed || walk_x <= -minSpeed)
            return true;
        if (walk_y >= minSpeed || walk_y <= -minSpeed)
            return true;
        if (rotate_Body >= 10 || rotate_Body <= -10)
            return true;
        return false;
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
            la.femur = degToRad(90.24);
            la.tibia = degToRad(90 - legIndex * 7.3);

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

        bool walking = hasWalkInput();

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            // Beine, die an der Sonderpose beteiligt sind, NICHT überschreiben.
            // specialPoseLoop() schreibt für diese Beine die Zielposition.
            if (legIsInSpecialPose(legIndex))
            {
                continue;
            }

            Vector3 newTarget = walking
                                    ? newTargetPosition(legIndex)
                                    : baseFootPositions[legIndex];

            targetPosition[legIndex] = getStepTargetPosition(legIndex, newTarget);
        }
    }

    /**
     * Setzt den Basis-Abstand zwischen dem Zentrum des Robots und den Füßen
     */
    void setBaseFootExtend(float newValue)
    {
        if (currentPhase == 0 && walkingStep == 0 && baseFootExtend != newValue)
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
        }
    }

    /**
     * Setzt die Stärke des Ease-In/Ease-Out für Schrittbewegungen
     * 0.0 = linear, 0.5 = Smoothstep, 1.0 = Smootherstep
     */
    void setStepSmoothness(float value)
    {
        stepSmoothness = fmaxf(0.0f, fminf(1.0f, value));
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
        if (currentPhase == 0 && walkingStep == 0)
        {
            if (walk_x != x || walk_y != y || rotate_Body != r)
            {
                walk_x = x;
                walk_y = y;
                rotate_Body = r;
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

    /**
     * Aktiviert eine bestimmte Sonderpose. Muss jeden Loop aufgerufen werden,
     * solange die Pose aktiv sein soll. Wird sie nicht mehr aufgerufen,
     * fahren die Beine nacheinander wieder in ihre Ausgangsposition zurück.
     */
    void doSpecialPose(uint8_t pose)
    {
        specialPoseRequested = true;

        // Neue Pose gewünscht, während Idle -> Sequenz starten
        if (specialPoseState == SP_Idle)
        {
            activeSpecialPose = pose;
            configureSpecialPoseLegs(pose);
            startSpecialMove(specialPoseLegA, targetForSpecialPose(specialPoseLegA, pose));
            specialPoseState = SP_MovingInA;
        }
    }

    /**
     * Liefert true, solange gerade eine Sonderpose-Sequenz läuft oder gehalten wird.
     * Während dieser Zeit dürfen die regulären Walk-Ziele nicht auf die Beine
     * geschrieben werden, die an der Sonderpose beteiligt sind.
     */
    bool isInSpecialPose() const
    {
        return specialPoseState != SP_Idle;
    }

    /**
     * true wenn das angegebene Bein gerade Teil einer Sonderpose ist.
     */
    bool legIsInSpecialPose(uint8_t legIndex) const
    {
        if (specialPoseState == SP_Idle)
            return false;
        return legIndex == specialPoseLegA || legIndex == specialPoseLegB;
    }

    /**
     * Tick für die Sonderpose-State-Machine. Muss einmal pro Loop aufgerufen
     * werden, nach mainLoop() und bevor calculateAllLegAngles() die Winkel rechnet.
     * Schreibt die interpolierten Zielpositionen in targetPosition[] für die
     * beteiligten Beine.
     */
    void specialPoseLoop()
    {
        if (specialPoseState == SP_Idle)
        {
            specialPoseRequested = false;
            return;
        }

        // Zeitbasierte Interpolation, gleiche Kadenz wie mainLoop (4ms/Step)
        if (millis() - previousSpecialStepMillis >= 4)
        {
            previousSpecialStepMillis = millis();

            // In Haltephase nicht weiterinterpolieren, nur auf Loslassen warten
            if (specialPoseState == SP_Holding)
            {
                if (!specialPoseRequested)
                {
                    // Loslassen -> Rückweg starten mit Bein A
                    startSpecialMove(specialPoseLegA, baseFootPositions[specialPoseLegA]);
                    specialPoseState = SP_MovingOutA;
                }
            }
            else
            {
                // Bewegungs-Phasen: Schritt hochzählen
                specialPoseStep++;

                if (specialPoseStep >= WALKING_STEP_COUNT)
                {
                    // Teilbewegung abgeschlossen -> nächste Phase
                    specialPoseStep = 0;
                    advanceSpecialPoseState();
                }
            }
        }

        // Zielposition(en) für die beteiligten Beine in targetPosition schreiben
        applySpecialPoseTargets();

        // Flag für nächsten Loop zurücksetzen; doSpecialPose() muss es wieder setzen
        specialPoseRequested = false;
    }

private:
    void configureSpecialPoseLegs(uint8_t pose)
    {
        // Welches Bein-Paar für welche Pose. Bei 5 Beinen gegenüberliegend = +2 oder +3.
        switch (pose)
        {
        case 0:
            specialPoseLegA = 3;
            specialPoseLegB = 2;
            break;
        case 1:
            specialPoseLegA = 0;
            specialPoseLegB = 1;
            break;
        default:
            specialPoseLegA = 2;
            specialPoseLegB = 3;
            break;
        }
    }

    /**
     * Startet eine Teil-Bewegung eines Beins: merkt sich Start (aktuelle Zielposition),
     * merkt sich Ziel und setzt den Interpolations-Schrittzähler zurück.
     */
    void startSpecialMove(uint8_t legIndex, Vector3 target)
    {
        specialMovingLeg = legIndex;
        specialMoveStart = targetPosition[legIndex];
        specialMoveTarget = target;
        specialPoseStep = 0;
        previousSpecialStepMillis = millis();
    }

    /**
     * Liefert die Ziel-Koordinate (Weltkoordinaten) für ein Bein in einer Sonderpose.
     *
     * Die Koordinaten werden relativ zur baseFootPosition des jeweiligen Beins
     * definiert, damit die Pose automatisch mit der 72°-Verteilung der Beine
     * mitdreht. So musst du die Pose nur einmal "vom ersten Bein aus gedacht"
     * hinschreiben und sie passt für jedes andere Bein auch.
     *
     * offset.x = radial nach außen (+) / zum Körper hin (-)  [aus Sicht des Beins]
     * offset.y = nach oben (+) / nach unten (-)
     * offset.z = tangential im Uhrzeigersinn (+) / gegen den Uhrzeigersinn (-)
     */
    Vector3 targetForSpecialPose(uint8_t legIndex, uint8_t pose) const
    {
        Vector3 offset = offsetForSpecialPose(legIndex, pose);

        // Offset in das radial/tangential-Koordinatensystem des Beins drehen
        const float angle = m_legBaseAngles[legIndex];
        const float radialX = cosf(angle);
        const float radialZ = sinf(angle);
        const float tangentX = -sinf(angle);
        const float tangentZ = cosf(angle);

        Vector3 base = baseFootPositions[legIndex];
        return Vector3(
            base.x + offset.x * radialX + offset.z * tangentX,
            base.y + offset.y,
            base.z + offset.x * radialZ + offset.z * tangentZ);
    }

    /**
     * Hier werden die Koordinaten-Offsets der Sonderpose definiert,
     * gemessen relativ zur baseFootPosition eines Beins.
     *
     * x = radial (+ = nach außen, - = zum Körper), y = hoch/runter, z = seitlich
     */
    Vector3 offsetForSpecialPose(uint8_t legIndex, uint8_t pose) const
    {
        switch (pose)
        {
        case 0:
            if (legIndex == specialPoseLegA)
            {
                return Vector3(-20.0f, 0.0f, 105.0f);
            }
            else if (legIndex == specialPoseLegB)
            {
                return Vector3(-20.0f, 0.0f, -105.0f);
            }
        case 1:
            return Vector3(0.0f, 0.0f, 0.0f);
        default:
            return Vector3(0.0f, 0.0f, 0.0f);
        }
    }

    /**
     * Schreibt die interpolierte Position der Sonderpose in targetPosition[]
     * für das gerade bewegte Bein und hält bereits bewegte Beine auf ihrer Endposition.
     */
    void applySpecialPoseTargets()
    {
        switch (specialPoseState)
        {
        case SP_MovingInA:
        {
            // Bein A bewegt sich zum Sonderpose-Ziel
            targetPosition[specialMovingLeg] = interpolateSin(
                specialMoveStart, specialMoveTarget, specialPoseStep, 1.0f);
            break;
        }
        case SP_MovingInB:
        {
            // Bein A bleibt auf seinem Ziel, Bein B fährt
            targetPosition[specialPoseLegA] = targetForSpecialPose(specialPoseLegA, activeSpecialPose);
            targetPosition[specialMovingLeg] = interpolateSin(
                specialMoveStart, specialMoveTarget, specialPoseStep, 1.0f);
            break;
        }
        case SP_Holding:
        {
            // Beide Beine halten ihre Sonderposition
            targetPosition[specialPoseLegA] = targetForSpecialPose(specialPoseLegA, activeSpecialPose);
            targetPosition[specialPoseLegB] = targetForSpecialPose(specialPoseLegB, activeSpecialPose);
            break;
        }
        case SP_MovingOutA:
        {
            // Bein B bleibt noch oben, Bein A geht zurück
            targetPosition[specialPoseLegB] = targetForSpecialPose(specialPoseLegB, activeSpecialPose);
            targetPosition[specialMovingLeg] = interpolateSin(
                specialMoveStart, specialMoveTarget, specialPoseStep, 1.0f);
            break;
        }
        case SP_MovingOutB:
        {
            // Bein A ist schon unten, Bein B geht jetzt zurück
            targetPosition[specialPoseLegA] = baseFootPositions[specialPoseLegA];
            targetPosition[specialMovingLeg] = interpolateSin(
                specialMoveStart, specialMoveTarget, specialPoseStep, 1.0f);
            break;
        }
        default:
            break;
        }
    }

    /**
     * Wird aufgerufen wenn eine Teil-Bewegung ihre WALKING_STEP_COUNT Schritte
     * abgeschlossen hat -> Übergang in die nächste Phase.
     */
    void advanceSpecialPoseState()
    {
        switch (specialPoseState)
        {
        case SP_MovingInA:
            // Bein A ist angekommen -> Bein B starten
            startSpecialMove(specialPoseLegB, targetForSpecialPose(specialPoseLegB, activeSpecialPose));
            specialPoseState = SP_MovingInB;
            break;

        case SP_MovingInB:
            // Beide Beine in Position -> halten
            specialPoseState = SP_Holding;
            break;

        case SP_MovingOutA:
            // Bein A ist zurück -> Bein B zurückbewegen
            startSpecialMove(specialPoseLegB, baseFootPositions[specialPoseLegB]);
            specialPoseState = SP_MovingOutB;
            break;

        case SP_MovingOutB:
            // Alles zurück -> Idle
            specialPoseState = SP_Idle;
            // lastTargetPosition synchronisieren, damit die Laufmechanik sauber weiterläuft
            lastTargetPosition[specialPoseLegA] = baseFootPositions[specialPoseLegA];
            lastTargetPosition[specialPoseLegB] = baseFootPositions[specialPoseLegB];
            break;

        default:
            break;
        }
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
        float x = walk_x;
        float y = walk_y;
        float r = rotate_Body;

        Vector3 walkVector = Vector3(x, 0, y);
        float rotation = rotateCoordinatesByLeg[legIndex];
        Vector3 rotated = walkVector.rotate(degToRad(rotation));
        rotated.x = -rotated.x;
        rotated = modifyVectorToRotateOnPosition(legIndex, rotated, r);

        Vector3 newTarget = lastTargetPosition[legIndex];

        if (currentMovingLegA != currentMovingLegB)
        {
            // Lösung mit 2 beinen in der Luft

            if (currentMovingLegA == legIndex || currentMovingLegB == legIndex)
            {
                newTarget.x -= rotated.x * 0.25;
                newTarget.z -= rotated.z * 0.25;
            }
            else
            {
                newTarget.x += rotated.x / 6.0;
                newTarget.z += rotated.z / 6.0;
            }
        }
        else
        {
            // Lösung mit nur einem Bein in der Luft

            float numberOfLegs = static_cast<float>(NUM_LEGS);

            if (currentMovingLegA == legIndex || currentMovingLegB == legIndex)
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
        }

        return newTarget;
    }

    Vector3 modifyVectorToRotateOnPosition(uint8_t legIndex, Vector3 vector, float rotation) const
    {
        float angle = m_legBaseAngles[legIndex];

        // Tangentiale Richtung am Kreis
        vector.x += rotation * (-sinf(angle));
        vector.z += rotation * cosf(angle);

        return vector;
    }

    Vector3 getStepTargetPosition(uint8_t legIndex, Vector3 newTarget) const
    {
        Vector3 origin = lastTargetPosition[legIndex];
        if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
        {
            float curveMultiplier = 0.0;

            if (currentMovingLegA == legIndex || currentMovingLegB == legIndex)
            {
                curveMultiplier = 1.0;
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

        // Normalisierter Fortschritt (0.0 bis 1.0)
        float t = static_cast<float>(walkingStep) / (WALKING_STEP_COUNT - 1);

        // Ease-In/Ease-Out nur für Beine in der Luft
        // stepSmoothness steuert die Stärke: 0.0 = linear, höher = weicher
        float tPos = t;
        if (curveMultiplier > 0 && stepSmoothness > 0)
        {
            // Symmetrische Potenz-Kurve: t wird um 0.5 zentriert,
            // Exponent < 1 → schnell starten/landen wird zu sanft starten/landen
            float centered = 2.0f * t - 1.0f; // -1 bis +1
            float sign = (centered >= 0) ? 1.0f : -1.0f;
            float exponent = 1.0f / (1.0f + stepSmoothness * 6.0f); // 1.0 (linear) bis 0.2 (sehr weich)
            tPos = 0.5f + 0.5f * sign * powf(fabsf(centered), exponent);
        }

        // X/Z mit Ease-In/Ease-Out interpolieren
        float x = start.x + (end.x - start.x) * tPos;
        float y = start.y + (end.y - start.y) * tPos;
        float z = start.z + (end.z - start.z) * tPos;

        // Y-Bogen für Beine in der Luft
        float yAddition = 0;
        if (curveMultiplier > 0)
        {
            float d = distance(start.x, start.z, end.x, end.z);
            if (d > 0)
            {
                // Sinus-Bogen basiert auf linearem t (nicht eased),
                // damit die Höhe symmetrisch bleibt
                yAddition = sinf(M_PI * t) * d * 0.5f * curveMultiplier;
                if (yAddition < 10)
                {
                    yAddition = 10.0f * sinf(M_PI * t);
                }
            }
        }

        return Vector3(x, y + yAddition, z);
    }

    float distance(float x1, float y1, float x2, float y2) const
    {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return sqrtf(dx * dx + dy * dy);
    }

    float sinCurve(float distanz, uint8_t walkingStep, float amplitude) const
    {
        if (walkingStep == 0)
        {
            return 0;
        }
        else if (walkingStep == WALKING_STEP_COUNT - 1)
        {
            return 0;
        }
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