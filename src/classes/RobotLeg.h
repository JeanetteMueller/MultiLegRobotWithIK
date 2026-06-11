
#ifndef RobotLeg_H
#define RobotLeg_H

#include "Vector3.h"
#include "LegLimits.h"
#include "AxisOffset.h"

class RobotLeg
{

public:
    // Allowed joint limits of this leg (in degrees). Set in definitions.h;
    // the default is wide (±180°) so nothing is blocked unintentionally.
    LegLimits legLimits;

    // Sideways offset of the knee joints along their rotation axis (in mm).
    // Default 0 -> coxa/femur/tibia form a straight line when viewed from above.
    AxisOffset axisOffset;

    // Robot geometry (in mm)
    const float BODY_RADIUS; // radius of the cylindrical body

    const float COXA_LENGTH;   // length of the coxa (L0)
    const float FEMUR_LENGTH;  // length of the upper leg / femur (L1)
    const float TIBIA_LENGTH;  // length of the lower leg / tibia (L2)
    const float HEIGHT_OFFSET; // deviation from the regular position above the ground
    float baseFootExtend;      // distance between the first servo and the foot. At 0 the foot sits directly below the first servo

    const float rotateCoordinates; // computed in the constructor from baseAngle (-2 * baseAngle, degrees)

    const double baseAngle; // base angle of the legs (72° apart)

    Vector3 baseFootPosition; // fixed foot positions on the ground
    Vector3 lastTargetPosition;
    Vector3 targetPosition;

private:
public:
    /**
     * Constructor
     */
    RobotLeg(float bodyRadius,
             float coxaLength,
             float thighLength,
             float shinLength,
             float heightOffset,
             float baseFootExtend,
             double baseAngleDeg,
             LegLimits legLimits = LegLimits(),
             AxisOffset axisOffset = AxisOffset()) : legLimits(legLimits),
                                    axisOffset(axisOffset),
                                    BODY_RADIUS(bodyRadius),
                                    COXA_LENGTH(coxaLength),
                                    FEMUR_LENGTH(thighLength),
                                    TIBIA_LENGTH(shinLength),
                                    HEIGHT_OFFSET(heightOffset),
                                    baseFootExtend(baseFootExtend),

                                    // rotateCoordinates rotates the walk command vector per leg and
                                    // is always derived from -2 * baseAngle (in degrees). Formerly a
                                    // separate parameter, now computed so the two never drift apart.
                                    // (initialized before baseAngle, since declared earlier)
                                    rotateCoordinates(-2.0f * baseAngleDeg),
                                    baseAngle(baseAngleDeg * M_PI / 180.0)
    {
        float footRadius = BODY_RADIUS + baseFootExtend; // distance from the center

        baseFootPosition = Vector3(
            cosf(baseAngle) * footRadius,
            0.0, // ground
            sinf(baseAngle) * footRadius);

        lastTargetPosition = baseFootPosition;
        targetPosition = baseFootPosition;
    }

    /**
     * Computes the joint angles for a single leg
     *
     * @param legIndex index of the leg (0-4)
     * @return LegAngles struct with the 3 angles
     */
    LegAngles calculateLegAngles(BodyPose m_pose) const
    {
        const float baseLegAngle = baseAngle;
        const float legAngleWithRot = baseLegAngle + m_pose.rotY;

        // Compute the coxa base position after body rotation
        float coxaBaseX, coxaBaseY, coxaBaseZ;
        calculateCoxaBasePosition(m_pose, legAngleWithRot, coxaBaseX, coxaBaseY, coxaBaseZ);

        // Foot position (stays on the ground, based on the base angle without body rotation)
        // // The foot is radial from the body: BODY_RADIUS + COXA_LENGTH + footOffset
        // const float footRadialDist = BODY_RADIUS + COXA_LENGTH + baseFootExtend;
        // const float footX = footRadialDist * cosf(baseLegAngle);
        // const float footY = 0.0;
        // const float footZ = footRadialDist * sinf(baseLegAngle);
        const float footX = targetPosition.x;
        const float footY = targetPosition.y;
        const float footZ = targetPosition.z;

        // Compute 3-DOF inverse kinematics
        LegAngles result = solveIK3DOF(coxaBaseX, coxaBaseY, coxaBaseZ,
                                       footX, footY, footZ, legAngleWithRot);

        return result;
    }

    /**
     * Computes the coxa base position after body rotation
     */
    void calculateCoxaBasePosition(BodyPose m_pose, float legAngleWithRot,
                                   float &coxaBaseX, float &coxaBaseY, float &coxaBaseZ) const
    {
        // Base position at the body edge
        coxaBaseX = BODY_RADIUS * cosf(legAngleWithRot);
        coxaBaseY = 0.0;
        coxaBaseZ = BODY_RADIUS * sinf(legAngleWithRot);

        // Rotation around the X axis (pitch)
        float tempY = coxaBaseY * cosf(m_pose.tiltX) - coxaBaseZ * sinf(m_pose.tiltX);
        float tempZ = coxaBaseY * sinf(m_pose.tiltX) + coxaBaseZ * cosf(m_pose.tiltX);
        coxaBaseY = tempY;
        coxaBaseZ = tempZ;

        // Rotation around the Z axis (roll)
        float tempX = coxaBaseX * cosf(m_pose.tiltZ) - coxaBaseY * sinf(m_pose.tiltZ);
        tempY = coxaBaseX * sinf(m_pose.tiltZ) + coxaBaseY * cosf(m_pose.tiltZ);
        coxaBaseX = tempX;
        coxaBaseY = tempY;

        // Shift to body height with offset, in case not all legs are mounted on the same plane.
        coxaBaseY += m_pose.height + HEIGHT_OFFSET;

        // Horizontal shift of the body center (e.g. to move the center of mass while
        // walking over the support triangle of the 3 standing legs). Pure
        // translation of the whole hip plane -> feet stay put, body moves.
        coxaBaseX += m_pose.bodyShiftX;
        coxaBaseZ += m_pose.bodyShiftZ;
    }

    /**
     * 3-DOF inverse kinematics for the coxa-femur-tibia configuration
     *
     * The coxa ALWAYS points outward (in the baseLegAngle direction)
     * It only rotates for a sideways offset (caused by body rotation)
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

        // Vector from coxa base to foot
        const float deltaX = footX - coxaBaseX;
        const float deltaZ = footZ - coxaBaseZ;

        // Base direction (radially outward)
        const float baseRadialX = cosf(baseLegAngle);
        const float baseRadialZ = sinf(baseLegAngle);

        // Tangential direction
        const float tangentX = -sinf(baseLegAngle);
        const float tangentZ = cosf(baseLegAngle);

        // Project the foot position onto the local coordinate system
        const float footRadial = deltaX * baseRadialX + deltaZ * baseRadialZ;
        const float footTangent = deltaX * tangentX + deltaZ * tangentZ;

        // Total sideways offset of the two knee joints along their rotation
        // axis. The femur and tibia axes are parallel (both tangential), so only
        // their sum matters for the foot position.
        const float axisOffsetTotal = axisOffset.femur + axisOffset.tibia;

        // Coxa angle: points primarily outward, but rotates for sideways
        // correction. Without an offset the coxa points straight at the foot
        // (atan2). With an offset the foot must keep a tangential distance of
        // axisOffsetTotal to the coxa line -> rotate the coxa by an additional
        // asin(offset / distance).
        const float footHorizDist = sqrtf(footRadial * footRadial + footTangent * footTangent);
        const float footBearing = atan2f(footTangent, fmaxf(footRadial, 0.001));
        float offsetCorrection = 0.0f;
        if (footHorizDist > 0.001f)
        {
            // clamp in case the offset is larger than the horizontal distance
            const float ratio = fmaxf(-1.0f, fminf(1.0f, axisOffsetTotal / footHorizDist));
            offsetCorrection = asinf(ratio);
        }
        float coxaAngle = footBearing - offsetCorrection;

        // Normalize to -π .. +π
        while (coxaAngle > M_PI)
            coxaAngle -= 2.0 * M_PI;
        while (coxaAngle < -M_PI)
            coxaAngle += 2.0 * M_PI;

        // Effective coxa direction after rotation
        const float effCoxaAngle = baseLegAngle + coxaAngle;

        // Direction of the coxa (outward)
        const float coxaDirX = cosf(effCoxaAngle);
        const float coxaDirZ = sinf(effCoxaAngle);

        // Position at the end of the coxa
        const float coxaEndX = coxaBaseX + COXA_LENGTH * coxaDirX;
        const float coxaEndY = coxaBaseY; // the coxa is horizontal
        const float coxaEndZ = coxaBaseZ + COXA_LENGTH * coxaDirZ;

        // Vector from coxa end to foot
        const float dx = footX - coxaEndX;
        const float dy = footY - coxaEndY;
        const float dz = footZ - coxaEndZ;

        // Project onto the coxa direction (radial as seen from the coxa end)
        // Positive = outward, negative = back toward the body
        const float footRadialFromCoxa = dx * coxaDirX + dz * coxaDirZ;

        // Horizontal distance WITHIN the femur-tibia plane: only the radial
        // component matters here. The tangential part (= sideways knee axis
        // offset) lies along the knee rotation axes and is not bridged by the
        // 2D IK. Without an offset footRadialFromCoxa equals the total horizontal
        // distance, so the behavior stays unchanged.
        const float legPlaneHoriz = fabsf(footRadialFromCoxa);
        const float legPlaneDist = sqrtf(footRadialFromCoxa * footRadialFromCoxa + dy * dy);

        // Check reachability
        if (legPlaneDist > FEMUR_LENGTH + TIBIA_LENGTH ||
            legPlaneDist < fabsf(FEMUR_LENGTH - TIBIA_LENGTH))
        {
            // Serial.println(" ANGLE IS NOT REACHABLE ");
            return result; // not reachable
        }

        // 2D IK for femur and tibia using the law of cosines
        const float cosKnee = (legPlaneDist * legPlaneDist -
                               FEMUR_LENGTH * FEMUR_LENGTH -
                               TIBIA_LENGTH * TIBIA_LENGTH) /
                              (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);

        // Tibia angle (negative = knee bent outward)
        const float tibiaAngle = -acosf(fmaxf(-1.0, fminf(1.0, cosKnee)));

        // Femur angle
        const float k1 = FEMUR_LENGTH + TIBIA_LENGTH * cosf(tibiaAngle);
        const float k2 = TIBIA_LENGTH * sinf(tibiaAngle);

        // If the foot points back toward the body, the horizontal distance must be negative
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
            // Single-leg gait (e.g. with only 4 legs, where two in the air would tip the robot).
            // Moving: (N-1)/N, standing: 1/N
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
            // Multiple legs in the air. Generalization of the original
            // 2-leg values (0.25 and 1/6):
            //   moving  = 1 / (M + 2)
            //   standing = M / ((M + 2) * (N - M))
            // This keeps the balance (sum over all legs = 0) and reproduces
            // the old values exactly for M = 2, N = 5.
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

        // Tangential direction along the circle
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

        // Normalized progress (0.0 to 1.0)
        float t = static_cast<float>(walkingStep) / (stepCount - 1);

        // Ease-in/ease-out only for legs in the air
        // stepSmoothness controls the strength: 0.0 = linear, higher = softer
        float tPos = t;
        if (curveMultiplier > 0)
        {
            // Symmetric power curve: t is centered around 0.5,
            // exponent < 1 → fast start/land becomes a soft start/land
            float centered = 2.0f * t - 1.0f; // -1 to +1
            float sign = (centered >= 0) ? 1.0f : -1.0f;
            float exponent = 1.0f / 2.0f; // 1.0 (linear) to 0.2 (very soft)
            tPos = 0.5f + 0.5f * sign * powf(fabsf(centered), exponent);
        }

        // Interpolate X/Z with ease-in/ease-out
        float x = start.x + (end.x - start.x) * tPos;
        float y = start.y + (end.y - start.y) * tPos;
        float z = start.z + (end.z - start.z) * tPos;

        // Y arc for legs in the air
        float yAddition = 0;
        if (curveMultiplier > 0)
        {
            float d = distance(start.x, start.z, end.x, end.z);
            if (d > 0)
            {
                // The sine arc is based on linear t (not eased),
                // so the height stays symmetric
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
     * Computes the distance between two points on the ground plane
     */
    float distance(float x1, float y1, float x2, float y2) const
    {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return sqrtf(dx * dx + dy * dy);
    }

    /**
     * Converts degrees to radians
     */
    float degToRad(float deg) const
    {
        return deg * M_PI / 180.0f;
    }

    /**
     * Returns the maximum reach of a leg (coxa + femur + tibia)
     */
    float getMaxReach() const
    {
        return COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH;
    }

    /**
     * Returns the minimum reach of a leg
     */
    float getMinReach() const
    {
        return COXA_LENGTH + fabsf(FEMUR_LENGTH - TIBIA_LENGTH);
    }
};

#endif // RobotLeg_H