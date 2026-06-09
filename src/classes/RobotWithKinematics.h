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

static_assert(NUMBER_OF_LEGS >= 4, "MultiLegRobotWithIK requires at least 4 legs");

#ifndef RobotWithKinematics_H
#define RobotWithKinematics_H

#include <cmath>
#include <array>
#include <stdexcept>

#include "Vector3.h"
#include "LegAngles.h"
#include "BodyPose.h"

#include "RobotLeg.h"

class RobotWithKinematics
{
public:
    const uint8_t NUM_LEGS = 0; // Anzahl der Beine
    uint8_t NUM_OF_MOVABLE_LEGS = 0; // Anzahl der Beine die maximal gleichzeitig vom Boden gehoben werden dürfen
private:
    const uint16_t WALKING_STEP_COUNT; // Anzahl der interpolierten Schritte für Bewegungsabläufe
    const uint16_t MAIN_LOOP_DELAY;    // Millis zwischen denen der MainLoop ausgeführt wird.

    RobotLeg *legs = nullptr;

    uint8_t *currentMovingLegs = nullptr; // Indizes der Beine, die aktuell in der Luft sind
    uint16_t walkingStep = 0; // Zwischenschritt Nummer des aktuellen Bewegungsablauf

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

    uint32_t previousStepMillis = 0;

    const float maxStepWidth;
    float walk_x = 0;
    float walk_y = 0;
    float rotate_Body = 0;

    uint8_t currentPhase = 0;

    BodyPose m_pose;

    // Quotient, wie weit das Koerperzentrum beim Laufen zum gegenueberliegenden
    // Bein verschoben wird (Anteil von BODY_RADIUS). 0 = aus. Wirkt nur bei 4 Beinen.
    float bodyShiftFactor = 0.0f;

public:
    /**
     * Konstruktor
     */
    RobotWithKinematics(
        uint8_t numberOfLegs,
        float maxStepWidth,
        uint16_t walkingStepCount,
        uint16_t mainLoopDelay,
        RobotLeg *legs) : NUM_LEGS(numberOfLegs),
                          maxStepWidth(maxStepWidth),
                          WALKING_STEP_COUNT(walkingStepCount),
                          MAIN_LOOP_DELAY(mainLoopDelay),
                          legs(legs)
    {
        // Bei weniger als 5 Beinen ist nur ein Bein gleichzeitig in der Luft sinnvoll
        // (sonst kippt der Roboter). Ab 5 Beinen kann die Hälfte gleichzeitig laufen.
        NUM_OF_MOVABLE_LEGS = (numberOfLegs < 5) ? 1 : numberOfLegs / 2;

        currentMovingLegs = new uint8_t[NUM_OF_MOVABLE_LEGS];
        const uint8_t spacing = NUM_LEGS / NUM_OF_MOVABLE_LEGS;
        for (uint8_t i = 0; i < NUM_OF_MOVABLE_LEGS; i++)
        {
            currentMovingLegs[i] = (i * spacing) % NUM_LEGS;
        }

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            Serial.print("leg ");
            Serial.print(legIndex);
            Serial.print(" baseAngle ");
            Serial.print(legs[legIndex].baseAngle);
            Serial.println("");
        }
    }

    ~RobotWithKinematics()
    {
        delete[] currentMovingLegs;
    }

    /**
     * true wenn das angegebene Bein aktuell zum Satz der bewegten Beine gehört.
     */
    bool isLegCurrentlyMoving(uint8_t legIndex) const
    {
        for (uint8_t i = 0; i < NUM_OF_MOVABLE_LEGS; i++)
        {
            if (currentMovingLegs[i] == legIndex)
            {
                return true;
            }
        }
        return false;
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
        if (millis() - previousStepMillis < MAIN_LOOP_DELAY)
        {
            return;
        }
        previousStepMillis = millis();

        walkingStep++;

        if (walkingStep >= WALKING_STEP_COUNT)
        {
            walkingStep = 0;

            for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
            {
                legs[legIndex].lastTargetPosition = legs[legIndex].targetPosition;
            }

            currentPhase++;
            if (currentPhase >= NUM_LEGS)
            {
                currentPhase = 0;
            }

            // Alle aktuell bewegten Beine um eins weiterrotieren (modulo NUM_LEGS).
            // Da alle Einträge synchron rotieren, bleibt der ursprünglich gewählte
            // Abstand zwischen den bewegten Beinen erhalten.
            for (uint8_t i = 0; i < NUM_OF_MOVABLE_LEGS; i++)
            {
                currentMovingLegs[i] += 1;
                if (currentMovingLegs[i] >= NUM_LEGS)
                {
                    currentMovingLegs[i] -= NUM_LEGS;
                }
            }

            // Zyklus ist an einem sauberen Nullpunkt.
            // Wenn wir gerade am Stoppen sind -> jetzt in Idle wechseln.
            if (walkState == Walk_Stopping && currentPhase == 0)
            {
                walkState = Walk_Idle;
            }
        }
    }

    /**
     * Zielkoordinaten vorbereiten bevor der Lauf-Loop starten kann
     */
    void prepareTargetPositions()
    {
        bool walking = hasWalkInput();

        // Koerperzentrum zum Gegenbein verschieben (nur 4-Bein-Roboter).
        updateBodyShift();

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            // Beine, die an der Sonderpose beteiligt sind, NICHT überschreiben.
            // specialPoseLoop() schreibt für diese Beine die Zielposition.
            if (legIsInSpecialPose(legIndex))
            {
                continue;
            }

            bool isCurrentMovingLeg = isLegCurrentlyMoving(legIndex);

            Vector3 newTarget = walking
                                    ? legs[legIndex].newTargetPosition(walk_x, walk_y, rotate_Body, NUM_OF_MOVABLE_LEGS, isCurrentMovingLeg)
                                    : legs[legIndex].baseFootPosition;

            legs[legIndex].targetPosition = legs[legIndex].getStepTargetPosition(WALKING_STEP_COUNT, walkingStep, newTarget, isCurrentMovingLeg);
        }
    }

    /**
     * Setzt den Basis-Abstand zwischen dem Zentrum des Robots und den Füßen
     */
    void setBaseFootExtend(float newValue)
    {
        for (uint8_t i = 0; i < NUM_LEGS; i++)
        {
            float footRadius = legs[i].BODY_RADIUS + newValue; // Abstand vom Basis Servo
            // Home-Position am ECHTEN Montagewinkel des Beins (nicht gleichmaessig
            // i*360/N), sonst stimmen Fuss- und Hueft-Winkel bei ungleichmaessig
            // verteilten Beinen nicht ueberein und die Coxa schwenkt zur Seite.
            float angle = legs[i].baseAngle;

            legs[i].baseFootPosition = Vector3(
                cosf(angle) * footRadius,
                0.0, // Boden
                sinf(angle) * footRadius);

            legs[i].baseFootExtend = newValue;
        }
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
        if (walk_x != x || walk_y != y || rotate_Body != r)
        {
            walk_x = x;
            walk_y = y;
            rotate_Body = r;
        }
    }

    /**
     * Setzt Laufrichtung, Fuss-Abstand und Koerper-Pose in einem Aufruf.
     * Die Parameter werden nur an einem sauberen Zyklus-Nullpunkt
     * (currentPhase == 0 && walkingStep == 0) uebernommen, damit sich
     * Ziele nicht mitten im Schritt aendern.
     */
    void applyControls(float walkX, float walkY, float rotateBody,
                       float footExtend,
                       float height, float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        if (currentPhase == 0 && walkingStep == 0)
        {
            setWalkDirection(walkX, walkY, rotateBody);
            setBaseFootExtend(footExtend);
            setPose(height, tiltXDeg, tiltZDeg, rotYDeg);
        }
    }

    /**
     * Setzt den Quotienten fuer die Koerper-Schwerpunktverschiebung beim Laufen.
     * Anteil von BODY_RADIUS (z.B. 0.3). 0 = aus. Wirkt nur bei 4-beinigen Robotern.
     */
    void setBodyShiftFactor(float factor)
    {
        bodyShiftFactor = factor;
    }

    /**
     * Verschiebt das Koerperzentrum beim Laufen kreisfoermig zum jeweils
     * gegenueberliegenden Bein des aktuell angehobenen Beins. Damit wandert der
     * Schwerpunkt (Akku in der Mitte) in das Stuetzdreieck der 3 stehenden Beine,
     * sodass der Roboter beim Anheben eines Beins nicht in dessen Richtung kippt.
     *
     * Nur fuer 4-beinige Roboter (Single-Leg-Gangart). Bei mehr Beinen passiert nichts.
     */
    void updateBodyShift()
    {
        if (NUM_LEGS != 4 || bodyShiftFactor == 0.0f || !hasWalkInput())
        {
            m_pose.bodyShiftX = 0.0f;
            m_pose.bodyShiftZ = 0.0f;
            return;
        }

        const uint8_t m = currentMovingLegs[0]; // aktuell angehobenes Bein
        const uint8_t prevLeg = (m + NUM_LEGS - 1) % NUM_LEGS;
        const uint8_t nextLeg = (m + 1) % NUM_LEGS;

        // Richtung jeweils zum gegenueberliegenden Bein (echter baseAngle, da die
        // Beine ungleichmaessig verteilt sein koennen).
        const float oppPrev = legs[(prevLeg + 2) % NUM_LEGS].baseAngle;
        const float oppCurr = legs[(m + 2) % NUM_LEGS].baseAngle;
        const float oppNext = legs[(nextLeg + 2) % NUM_LEGS].baseAngle;

        // Fortschritt im aktuellen Zyklus (0..1). In der Mitte (t=0.5) ist das Bein
        // maximal angehoben -> dort soll der Koerper genau zum Gegenbein zeigen.
        const float t = static_cast<float>(walkingStep) / static_cast<float>(WALKING_STEP_COUNT);

        float angle;
        if (t < 0.5f)
        {
            angle = lerpAngleRad(oppPrev, oppCurr, t + 0.5f);
        }
        else
        {
            angle = lerpAngleRad(oppCurr, oppNext, t - 0.5f);
        }

        const float shiftDistance = bodyShiftFactor * legs[0].BODY_RADIUS;
        m_pose.bodyShiftX = shiftDistance * cosf(angle);
        m_pose.bodyShiftZ = shiftDistance * sinf(angle);
    }

    /**
     * Interpoliert zwischen zwei Winkeln (Radiant) ueber den kuerzesten Weg.
     */
    static float lerpAngleRad(float a, float b, float t)
    {
        float d = b - a;
        while (d > M_PI)
            d -= 2.0f * M_PI;
        while (d < -M_PI)
            d += 2.0f * M_PI;
        return a + d * t;
    }

    /**
     * Berechnet die Gelenkwinkel für alle 5 Beine
     *
     * @return Array mit 5 LegAngles Strukturen
     */
    std::array<LegAngles, NUMBER_OF_LEGS> calculateAllLegAngles() const
    {
        std::array<LegAngles, NUMBER_OF_LEGS> results;

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; ++legIndex)
        {
            results[legIndex] = legs[legIndex].calculateLegAngles(m_pose);
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
            if (!legs[legIndex].calculateLegAngles(m_pose).valid)
            {
                return false;
            }
        }
        return true;
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
                    startSpecialMove(specialPoseLegA, legs[specialPoseLegA].baseFootPosition);
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
     * true wenn das angegebene Bein gerade Teil einer Sonderpose ist.
     */
    bool legIsInSpecialPose(uint8_t legIndex) const
    {
        if (specialPoseState == SP_Idle)
            return false;
        return legIndex == specialPoseLegA || legIndex == specialPoseLegB;
    }

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
        specialMoveStart = legs[legIndex].targetPosition;
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
        const float angle = legs[legIndex].baseAngle;
        const float radialX = cosf(angle);
        const float radialZ = sinf(angle);
        const float tangentX = -sinf(angle);
        const float tangentZ = cosf(angle);

        Vector3 base = legs[legIndex].baseFootPosition;
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
            legs[specialMovingLeg].targetPosition = legs[specialMovingLeg].interpolateSin(WALKING_STEP_COUNT,
                                                                                          specialMoveStart,
                                                                                          specialMoveTarget,
                                                                                          specialPoseStep,
                                                                                          1.0f);
            break;
        }
        case SP_MovingInB:
        {
            // Bein A bleibt auf seinem Ziel, Bein B fährt
            legs[specialPoseLegA].targetPosition = targetForSpecialPose(specialPoseLegA, activeSpecialPose);
            legs[specialMovingLeg].targetPosition = legs[specialMovingLeg].interpolateSin(WALKING_STEP_COUNT,
                                                                                          specialMoveStart,
                                                                                          specialMoveTarget,
                                                                                          specialPoseStep,
                                                                                          1.0f);
            break;
        }
        case SP_Holding:
        {
            // Beide Beine halten ihre Sonderposition
            legs[specialPoseLegA].targetPosition = targetForSpecialPose(specialPoseLegA, activeSpecialPose);
            legs[specialPoseLegB].targetPosition = targetForSpecialPose(specialPoseLegB, activeSpecialPose);
            break;
        }
        case SP_MovingOutA:
        {
            // Bein B bleibt noch oben, Bein A geht zurück
            legs[specialPoseLegB].targetPosition = targetForSpecialPose(specialPoseLegB, activeSpecialPose);
            legs[specialMovingLeg].targetPosition = legs[specialMovingLeg].interpolateSin(WALKING_STEP_COUNT,
                                                                                          specialMoveStart,
                                                                                          specialMoveTarget,
                                                                                          specialPoseStep,
                                                                                          1.0f);
            break;
        }
        case SP_MovingOutB:
        {
            // Bein A ist schon unten, Bein B geht jetzt zurück
            legs[specialPoseLegA].targetPosition = legs[specialPoseLegA].baseFootPosition;
            legs[specialMovingLeg].targetPosition = legs[specialMovingLeg].interpolateSin(WALKING_STEP_COUNT,
                                                                                          specialMoveStart,
                                                                                          specialMoveTarget,
                                                                                          specialPoseStep,
                                                                                          1.0f);
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
            startSpecialMove(specialPoseLegB, legs[specialPoseLegB].baseFootPosition);
            specialPoseState = SP_MovingOutB;
            break;

        case SP_MovingOutB:
            // Alles zurück -> Idle
            specialPoseState = SP_Idle;
            // lastTargetPosition synchronisieren, damit die Laufmechanik sauber weiterläuft
            legs[specialPoseLegA].lastTargetPosition = legs[specialPoseLegA].baseFootPosition;
            legs[specialPoseLegB].lastTargetPosition = legs[specialPoseLegB].baseFootPosition;
            break;

        default:
            break;
        }
    }
};

#endif // RobotWithKinematics_H