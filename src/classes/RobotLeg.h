
#ifndef RobotLeg_H
#define RobotLeg_H

#include "Vector3.h"
#include "LegLimits.h"

class RobotLeg
{

public:
    // Erlaubte Gelenk-Limits dieses Beins (in Grad). Werden in der definitions.h
    // gesetzt; Default ist weit (±180°), damit nichts ungewollt blockiert.
    LegLimits legLimits;

    // Roboter-Geometrie (in mm)
    const float BODY_RADIUS; // Radius des zylindrischen Körpers

    const float COXA_LENGTH;   // Länge der Coxa (L0)
    const float FEMUR_LENGTH;  // Länge des Oberschenkels (L1)
    const float TIBIA_LENGTH;  // Länge des Unterschenkels (L2)
    const float HEIGHT_OFFSET; // Abweichung von regulären Position über dem Boden
    float baseFootExtend;      // Abstand zwischen erstem Servo und Fuß. Bei 0 wäre der Fuß direkt unter dem ersten Servo plaziert

    const float rotateCoordinates; // wird im Konstruktor aus baseAngle berechnet (-2 * baseAngle, Grad)

    const double baseAngle; // Basis-Winkel der Beine (72° versetzt)

    Vector3 baseFootPosition; // Feste Fußpositionen auf dem Boden
    Vector3 lastTargetPosition;
    Vector3 targetPosition;

private:
public:
    /**
     * Konstruktor
     */
    RobotLeg(float bodyRadius,
             float coxaLength,
             float thighLength,
             float shinLength,
             float heightOffset,
             float baseFootExtend,
             double baseAngleDeg,
             LegLimits legLimits = LegLimits()) : legLimits(legLimits),
                                    BODY_RADIUS(bodyRadius),
                                    COXA_LENGTH(coxaLength),
                                    FEMUR_LENGTH(thighLength),
                                    TIBIA_LENGTH(shinLength),
                                    HEIGHT_OFFSET(heightOffset),
                                    baseFootExtend(baseFootExtend),

                                    // rotateCoordinates dreht den Lauf-Befehlsvektor pro Bein und
                                    // ergibt sich immer aus -2 * baseAngle (in Grad). Frueher ein
                                    // eigener Parameter, jetzt berechnet, damit beide nie auseinanderlaufen.
                                    // (vor baseAngle initialisiert, da frueher deklariert)
                                    rotateCoordinates(-2.0f * baseAngleDeg),
                                    baseAngle(baseAngleDeg * M_PI / 180.0)
    {
        float footRadius = BODY_RADIUS + baseFootExtend; // Abstand vom Zentrum

        baseFootPosition = Vector3(
            cosf(baseAngle) * footRadius,
            0.0, // Boden
            sinf(baseAngle) * footRadius);

        lastTargetPosition = baseFootPosition;
        targetPosition = baseFootPosition;
    }

    /**
     * Berechnet die Gelenkwinkel für ein einzelnes Bein
     *
     * @param legIndex Index des Beins (0-4)
     * @return LegAngles Struktur mit den 3 Winkeln
     */
    LegAngles calculateLegAngles(BodyPose m_pose) const
    {
        const float baseLegAngle = baseAngle;
        const float legAngleWithRot = baseLegAngle + m_pose.rotY;

        // Berechne Coxa-Basis Position nach Körper-Rotation
        float coxaBaseX, coxaBaseY, coxaBaseZ;
        calculateCoxaBasePosition(m_pose, legAngleWithRot, coxaBaseX, coxaBaseY, coxaBaseZ);

        // Fuß-Position (bleibt auf dem Boden, basiert auf Basis-Winkel ohne Körperdrehung)
        // // Der Fuß ist radial vom Körper: BODY_RADIUS + COXA_LENGTH + footOffset
        // const float footRadialDist = BODY_RADIUS + COXA_LENGTH + baseFootExtend;
        // const float footX = footRadialDist * cosf(baseLegAngle);
        // const float footY = 0.0;
        // const float footZ = footRadialDist * sinf(baseLegAngle);
        const float footX = targetPosition.x;
        const float footY = targetPosition.y;
        const float footZ = targetPosition.z;

        // 3-DOF Inverse Kinematik berechnen
        LegAngles result = solveIK3DOF(coxaBaseX, coxaBaseY, coxaBaseZ,
                                       footX, footY, footZ, legAngleWithRot);

        return result;
    }

    /**
     * Berechnet die Coxa-Basis Position nach Körper-Rotation
     */
    void calculateCoxaBasePosition(BodyPose m_pose, float legAngleWithRot,
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

        // Verschiebung zur Körperhöhe mit offset, falls nicht alle Beine auf der selben Ebene montiert sind.
        coxaBaseY += m_pose.height + HEIGHT_OFFSET;

        // Horizontale Verschiebung des Körperzentrums (z.B. um den Schwerpunkt beim
        // Laufen über das Stützdreieck der 3 stehenden Beine zu bringen). Reine
        // Translation der gesamten Hüft-Ebene -> Füße bleiben stehen, Körper wandert.
        coxaBaseX += m_pose.bodyShiftX;
        coxaBaseZ += m_pose.bodyShiftZ;
    }

    /**
     * 3-DOF Inverse Kinematik für Coxa-Femur-Tibia Konfiguration
     *
     * Die Coxa zeigt IMMER nach außen (in baseLegAngle Richtung)
     * Nur bei seitlicher Verschiebung (durch Körperdrehung) dreht sie sich
     */
    LegAngles solveIK3DOF(float coxaBaseX, float coxaBaseY, float coxaBaseZ,
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
            // Serial.println(" WINKEL IST NICHT ERREICHBAR ");
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
        result.valid = result.allAnglesInLimit(legLimits);

        return result;
    }

    Vector3 newTargetPosition(float walk_x, float walk_y, float rotate_Body, uint8_t numMovingLegs, bool isCurrentMovingLeg) const
    {
        float x = walk_x;
        float y = walk_y;
        float r = rotate_Body;

        Vector3 walkVector = Vector3(x, 0, y);
        float rotation = rotateCoordinates;
        Vector3 rotated = walkVector.rotate(degToRad(rotation));
        rotated.x = -rotated.x;
        rotated = modifyVectorToRotateOnPosition(rotated, r);

        Vector3 newTarget = lastTargetPosition;

        const float numberOfLegs = static_cast<float>(NUMBER_OF_LEGS);

        if (numMovingLegs == 1)
        {
            // Single-Leg-Gangart (z.B. bei nur 4 Beinen, wo zwei in der Luft zum Kippen führen würde).
            // Bewegt: (N-1)/N, Stehend: 1/N
            if (isCurrentMovingLeg)
            {
                newTarget.x -= rotated.x / numberOfLegs * (numberOfLegs - 1);
                newTarget.z -= rotated.z / numberOfLegs * (numberOfLegs - 1);
            }
            else
            {
                newTarget.x += rotated.x / numberOfLegs;
                newTarget.z += rotated.z / numberOfLegs;
            }
        }
        else
        {
            // Mehrere Beine in der Luft. Verallgemeinerung der ursprünglichen
            // 2-Bein-Werte (0.25 und 1/6):
            //   bewegt  = 1 / (M + 2)
            //   stehend = M / ((M + 2) * (N - M))
            // Das hält die Bilanz (Summe über alle Beine = 0) erhalten und reproduziert
            // für M = 2, N = 5 exakt die alten Werte.
            const float movingLegs = static_cast<float>(numMovingLegs);
            const float stationaryLegs = numberOfLegs - movingLegs;

            const float movingFactor = 1.0f / (movingLegs + 2.0f);
            const float stationaryFactor = movingLegs / ((movingLegs + 2.0f) * stationaryLegs);

            if (isCurrentMovingLeg)
            {
                newTarget.x -= rotated.x * movingFactor;
                newTarget.z -= rotated.z * movingFactor;
            }
            else
            {
                newTarget.x += rotated.x * stationaryFactor;
                newTarget.z += rotated.z * stationaryFactor;
            }
        }

        return newTarget;
    }

    Vector3 modifyVectorToRotateOnPosition(Vector3 vector, float rotation) const
    {
        float angle = baseAngle;

        // Tangentiale Richtung am Kreis
        vector.x += rotation * (-sinf(angle));
        vector.z += rotation * cosf(angle);

        return vector;
    }

    Vector3 getStepTargetPosition(uint16_t stepCount, uint16_t step, Vector3 newTarget, bool isMoving) const
    {
        Vector3 origin = lastTargetPosition;
        if (origin.x != newTarget.x || origin.y != newTarget.y || origin.z != newTarget.z)
        {
            float curveMultiplier = isMoving ? 1.0 : 0.0;

            return interpolateSin(stepCount, origin, newTarget, step, curveMultiplier);
        }

        return origin;
    }

    Vector3 interpolateSin(uint16_t stepCount, Vector3 start, Vector3 end, uint8_t walkingStep, float curveMultiplier) const
    {
        if (walkingStep == 0)
        {
            return start;
        }
        else if (walkingStep == stepCount - 1)
        {
            return end;
        }

        // Normalisierter Fortschritt (0.0 bis 1.0)
        float t = static_cast<float>(walkingStep) / (stepCount - 1);

        // Ease-In/Ease-Out nur für Beine in der Luft
        // stepSmoothness steuert die Stärke: 0.0 = linear, höher = weicher
        float tPos = t;
        if (curveMultiplier > 0)
        {
            // Symmetrische Potenz-Kurve: t wird um 0.5 zentriert,
            // Exponent < 1 → schnell starten/landen wird zu sanft starten/landen
            float centered = 2.0f * t - 1.0f; // -1 bis +1
            float sign = (centered >= 0) ? 1.0f : -1.0f;
            float exponent = 1.0f / 2.0f; // 1.0 (linear) bis 0.2 (sehr weich)
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
                float peak = d * 0.5f * curveMultiplier;

                float minHeight = 30.0f;
                if (peak < minHeight)
                {
                    peak = minHeight;
                }
                yAddition = sinf(M_PI * t) * peak;
            }
        }

        return Vector3(x, y + yAddition, z);
    }

    /**
     * Berechnet die distanz zwischen zwei punkten auf der Bodenebene
     */
    float distance(float x1, float y1, float x2, float y2) const
    {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return sqrtf(dx * dx + dy * dy);
    }

    /**
     * Rechnet Degree zu Radiant um
     */
    float degToRad(float deg) const
    {
        return deg * M_PI / 180.0f;
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
};

#endif // RobotLeg_H