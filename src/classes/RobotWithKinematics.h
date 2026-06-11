/**
 * RobotWithKinematics.h
 *
 * C++ class for computing the inverse kinematics of a 4-8 legged robot
 * with a cylindrical body and 3-DOF legs.
 *
 * Each leg has 3 joints:
 *   - θ0 (hip swing): rotation around an axis perpendicular to the femur
 *   - θ1 (hip lift): raises/lowers the femur
 *   - θ2 (knee): bends/extends the knee
 *
 * Author: Claude.ai & Jeanette Müller
 * Date: 2025
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
    const uint8_t NUM_LEGS = 0; // number of legs
    uint8_t NUM_OF_MOVABLE_LEGS = 0; // max number of legs that may be lifted off the ground at the same time
private:
    const uint16_t WALKING_STEP_COUNT; // number of interpolated steps per movement
    const uint16_t MAIN_LOOP_DELAY;    // millis between mainLoop executions.

    RobotLeg *legs = nullptr;

    uint8_t *currentMovingLegs = nullptr; // indices of the legs that are currently in the air
    uint16_t walkingStep = 0; // sub-step number of the current movement

    enum WalkState : uint8_t
    {
        Walk_Idle = 0, // standing still, walk cycle frozen at step=0, phase=0
        Walk_Active,   // walk cycle running
        Walk_Stopping  // input released, finish the cycle cleanly
    };
    WalkState walkState = Walk_Idle;

    // ---- Special Pose State Machine ----
    enum SpecialPoseState : uint8_t
    {
        SP_Idle = 0,   // no special pose active
        SP_MovingInA,  // leg A moves into the special position
        SP_MovingInB,  // leg B moves into the special position
        SP_Holding,    // special pose is being held
        SP_MovingOutA, // leg A moves back to the home position
        SP_MovingOutB  // leg B moves back to the home position
    };

    SpecialPoseState specialPoseState = SP_Idle;
    uint8_t activeSpecialPose = 0;     // which special pose (0..3)
    uint8_t specialPoseLegA = 2;       // which leg first
    uint8_t specialPoseLegB = 3;       // which leg second
    bool specialPoseRequested = false; // set every loop by doSpecialPose()
    int8_t specialPoseStep = 0;        // interpolation step 0..WALKING_STEP_COUNT-1
    uint32_t previousSpecialStepMillis = 0;

    // Start/target position for the current sub-movement (Vector3, in world coordinates)
    Vector3 specialMoveStart;
    Vector3 specialMoveTarget;
    uint8_t specialMovingLeg = 0; // the leg that is currently moving

    uint32_t previousStepMillis = 0;

    const float maxStepWidth;
    float walk_x = 0;
    float walk_y = 0;
    float rotate_Body = 0;

    uint8_t currentPhase = 0;

    BodyPose m_pose;

    // Factor for how far the body center is shifted toward the opposite leg
    // while walking (fraction of BODY_RADIUS). 0 = off. Only effective with 4 legs.
    float bodyShiftFactor = 0.0f;

public:
    /**
     * Constructor
     */
    RobotWithKinematics(
        uint8_t numberOfLegs,
        float maxStepWidth,
        uint16_t walkingStepCount,
        uint16_t mainLoopDelay,
        RobotLeg *legs) : NUM_LEGS(numberOfLegs),
                          WALKING_STEP_COUNT(walkingStepCount),
                          MAIN_LOOP_DELAY(mainLoopDelay),
                          legs(legs),
                          maxStepWidth(maxStepWidth)
    {
        // With fewer than 5 legs only one leg in the air at a time makes sense
        // (otherwise the robot tips). From 5 legs on, half of them can move at once.
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
     * true if the given leg currently belongs to the set of moving legs.
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
     * Prepares the movement sequences
     */
    void mainLoop()
    {
        // State transitions: react to input changes
        switch (walkState)
        {
        case Walk_Idle:
            // While idle the cycle rests at step=0, phase=0. Start when input arrives.
            if (hasWalkInput())
            {
                walkState = Walk_Active;
                previousStepMillis = millis(); // clean start
            }
            else
            {
                // do nothing, clock rests
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
            // Input comes back before the cycle finishes -> back to Active
            if (hasWalkInput())
            {
                walkState = Walk_Active;
            }
            break;
        }

        // Time tick
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

            // Rotate all currently moving legs by one (modulo NUM_LEGS).
            // Since all entries rotate in sync, the originally chosen spacing
            // between the moving legs is preserved.
            for (uint8_t i = 0; i < NUM_OF_MOVABLE_LEGS; i++)
            {
                currentMovingLegs[i] += 1;
                if (currentMovingLegs[i] >= NUM_LEGS)
                {
                    currentMovingLegs[i] -= NUM_LEGS;
                }
            }

            // The cycle is at a clean zero point.
            // If we are currently stopping -> switch to Idle now.
            if (walkState == Walk_Stopping && currentPhase == 0)
            {
                walkState = Walk_Idle;
            }
        }
    }

    /**
     * Prepare the target coordinates before the walk loop can start
     */
    void prepareTargetPositions()
    {
        bool walking = hasWalkInput();

        // Shift the body center toward the opposite leg (4-leg robots only).
        updateBodyShift();

        for (uint8_t legIndex = 0; legIndex < NUM_LEGS; legIndex++)
        {
            // Do NOT overwrite legs that are part of the special pose.
            // specialPoseLoop() writes the target position for those legs.
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
     * Sets the base distance between the robot's center and the feet
     */
    void setBaseFootExtend(float newValue)
    {
        for (uint8_t i = 0; i < NUM_LEGS; i++)
        {
            float footRadius = legs[i].BODY_RADIUS + newValue; // distance from the base servo
            // Home position at the REAL mounting angle of the leg (not evenly
            // i*360/N), otherwise the foot and hip angles do not match for
            // unevenly distributed legs and the coxa swings sideways.
            float angle = legs[i].baseAngle;

            legs[i].baseFootPosition = Vector3(
                cosf(angle) * footRadius,
                0.0, // ground
                sinf(angle) * footRadius);

            legs[i].baseFootExtend = newValue;
        }
    }

    /**
     * Sets the body pose with individual parameters (angles in degrees)
     */
    void setPose(float height,
                 float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        m_pose = BodyPose::fromDegrees(height, tiltXDeg, tiltZDeg, rotYDeg);
    }

    /**
     * Sets the coordinate offset the robot should move toward
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
     * Sets walk direction, foot extension and body pose in a single call.
     * The parameters are only latched at a clean cycle zero point
     * (currentPhase == 0 && walkingStep == 0) so that targets do not
     * change mid-step.
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
     * Sets the factor for the body center-of-mass shift while walking.
     * Fraction of BODY_RADIUS (e.g. 0.3). 0 = off. Only effective on 4-legged robots.
     */
    void setBodyShiftFactor(float factor)
    {
        bodyShiftFactor = factor;
    }

    /**
     * While walking, shifts the body center in a circular motion toward the leg
     * opposite the currently lifted leg. This moves the center of mass (battery
     * in the middle) into the support triangle of the 3 standing legs, so that
     * the robot does not tip toward a leg while lifting it.
     *
     * Only for 4-legged robots (single-leg gait). With more legs nothing happens.
     */
    void updateBodyShift()
    {
        if (NUM_LEGS != 4 || bodyShiftFactor == 0.0f || !hasWalkInput())
        {
            m_pose.bodyShiftX = 0.0f;
            m_pose.bodyShiftZ = 0.0f;
            return;
        }

        const uint8_t m = currentMovingLegs[0]; // currently lifted leg
        const uint8_t prevLeg = (m + NUM_LEGS - 1) % NUM_LEGS;
        const uint8_t nextLeg = (m + 1) % NUM_LEGS;

        // Direction toward the opposite leg in each case (real baseAngle, since the
        // legs may be unevenly distributed).
        const float oppPrev = legs[(prevLeg + 2) % NUM_LEGS].baseAngle;
        const float oppCurr = legs[(m + 2) % NUM_LEGS].baseAngle;
        const float oppNext = legs[(nextLeg + 2) % NUM_LEGS].baseAngle;

        // Progress within the current cycle (0..1). In the middle (t=0.5) the leg
        // is lifted the most -> there the body should point exactly at the opposite leg.
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
     * Interpolates between two angles (radians) along the shortest path.
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
     * Computes the joint angles for all 5 legs
     *
     * @return array with 5 LegAngles structs
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
     * Checks whether all legs can reach their target position
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
     * Activates a specific special pose. Must be called every loop as long as
     * the pose should stay active. Once it is no longer called, the legs move
     * back to their home position one after another.
     */
    void doSpecialPose(uint8_t pose)
    {
        specialPoseRequested = true;

        // New pose requested while idle -> start the sequence
        if (specialPoseState == SP_Idle)
        {
            activeSpecialPose = pose;
            configureSpecialPoseLegs(pose);
            startSpecialMove(specialPoseLegA, targetForSpecialPose(specialPoseLegA, pose));
            specialPoseState = SP_MovingInA;
        }
    }

    /**
     * Tick for the special-pose state machine. Must be called once per loop,
     * after mainLoop() and before calculateAllLegAngles() computes the angles.
     * Writes the interpolated target positions into targetPosition[] for the
     * legs involved.
     */
    void specialPoseLoop()
    {
        if (specialPoseState == SP_Idle)
        {
            specialPoseRequested = false;
            return;
        }

        // Time-based interpolation, same cadence as mainLoop (4ms/step)
        if (millis() - previousSpecialStepMillis >= 4)
        {
            previousSpecialStepMillis = millis();

            // During the hold phase do not interpolate further, just wait for release
            if (specialPoseState == SP_Holding)
            {
                if (!specialPoseRequested)
                {
                    // Released -> start the return path with leg A
                    startSpecialMove(specialPoseLegA, legs[specialPoseLegA].baseFootPosition);
                    specialPoseState = SP_MovingOutA;
                }
            }
            else
            {
                // Movement phases: increment the step
                specialPoseStep++;

                if (specialPoseStep >= WALKING_STEP_COUNT)
                {
                    // Sub-movement finished -> next phase
                    specialPoseStep = 0;
                    advanceSpecialPoseState();
                }
            }
        }

        // Write the target position(s) for the involved legs into targetPosition
        applySpecialPoseTargets();

        // Reset the flag for the next loop; doSpecialPose() must set it again
        specialPoseRequested = false;
    }

private:
    /**
     * true if there is currently input that should actually trigger movement.
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
     * true if the given leg is currently part of a special pose.
     */
    bool legIsInSpecialPose(uint8_t legIndex) const
    {
        if (specialPoseState == SP_Idle)
            return false;
        return legIndex == specialPoseLegA || legIndex == specialPoseLegB;
    }

    void configureSpecialPoseLegs(uint8_t pose)
    {
        // Which leg pair for which pose. With 5 legs, opposite = +2 or +3.
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
     * Starts a sub-movement of a leg: remembers the start (current target position),
     * remembers the target and resets the interpolation step counter.
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
     * Returns the target coordinate (world coordinates) for a leg in a special pose.
     *
     * The coordinates are defined relative to the baseFootPosition of the
     * respective leg, so the pose automatically rotates along with the 72°
     * distribution of the legs. That way you only need to write the pose once
     * "thinking from the first leg" and it fits every other leg too.
     *
     * offset.x = radially outward (+) / toward the body (-)  [from the leg's view]
     * offset.y = upward (+) / downward (-)
     * offset.z = tangential clockwise (+) / counter-clockwise (-)
     */
    Vector3 targetForSpecialPose(uint8_t legIndex, uint8_t pose) const
    {
        Vector3 offset = offsetForSpecialPose(legIndex, pose);

        // Rotate the offset into the radial/tangential coordinate system of the leg
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
     * Here the coordinate offsets of the special pose are defined,
     * measured relative to a leg's baseFootPosition.
     *
     * x = radial (+ = outward, - = toward body), y = up/down, z = sideways
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
     * Writes the interpolated special-pose position into targetPosition[]
     * for the currently moving leg and holds already moved legs at their end position.
     */
    void applySpecialPoseTargets()
    {
        switch (specialPoseState)
        {
        case SP_MovingInA:
        {
            // Leg A moves toward the special-pose target
            legs[specialMovingLeg].targetPosition = legs[specialMovingLeg].interpolateSin(WALKING_STEP_COUNT,
                                                                                          specialMoveStart,
                                                                                          specialMoveTarget,
                                                                                          specialPoseStep,
                                                                                          1.0f);
            break;
        }
        case SP_MovingInB:
        {
            // Leg A stays on its target, leg B moves
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
            // Both legs hold their special position
            legs[specialPoseLegA].targetPosition = targetForSpecialPose(specialPoseLegA, activeSpecialPose);
            legs[specialPoseLegB].targetPosition = targetForSpecialPose(specialPoseLegB, activeSpecialPose);
            break;
        }
        case SP_MovingOutA:
        {
            // Leg B stays up, leg A goes back
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
            // Leg A is already down, leg B now goes back
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
     * Called when a sub-movement has completed its WALKING_STEP_COUNT steps
     * -> transition to the next phase.
     */
    void advanceSpecialPoseState()
    {
        switch (specialPoseState)
        {
        case SP_MovingInA:
            // Leg A has arrived -> start leg B
            startSpecialMove(specialPoseLegB, targetForSpecialPose(specialPoseLegB, activeSpecialPose));
            specialPoseState = SP_MovingInB;
            break;

        case SP_MovingInB:
            // Both legs in position -> hold
            specialPoseState = SP_Holding;
            break;

        case SP_MovingOutA:
            // Leg A is back -> move leg B back
            startSpecialMove(specialPoseLegB, legs[specialPoseLegB].baseFootPosition);
            specialPoseState = SP_MovingOutB;
            break;

        case SP_MovingOutB:
            // Everything back -> Idle
            specialPoseState = SP_Idle;
            // Synchronize lastTargetPosition so the walk mechanics continue cleanly
            legs[specialPoseLegA].lastTargetPosition = legs[specialPoseLegA].baseFootPosition;
            legs[specialPoseLegB].lastTargetPosition = legs[specialPoseLegB].baseFootPosition;
            break;

        default:
            break;
        }
    }
};

#endif // RobotWithKinematics_H