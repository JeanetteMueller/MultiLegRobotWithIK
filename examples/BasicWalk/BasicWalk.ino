/**
 * BasicWalk.ino
 *
 * Minimal, hardware-independent example for the MultiLegRobotWithIK library.
 *
 * It builds a 5-legged robot, feeds it a constant "walk forward" command and
 * prints the resulting joint angles for every leg to the serial monitor.
 *
 * There is no servo or remote-control code here on purpose: the library only
 * computes the angles. Mapping LegAngles -> your servos is up to you (see the
 * README). Swap the printing in loop() for your own servo writes.
 *
 * Board: any ESP32. Serial monitor @ 115200 baud.
 */

// 1) The library REQUIRES the leg count to be defined before it is included.
#define NUMBER_OF_LEGS 5
#include <MultiLegRobotWithIK.h>

// ---- Robot geometry (millimetres) -----------------------------------------
const float BODY_RADIUS   = 104.175f; // centre of body to a leg's first servo
const float COXA_LENGTH   = 54.0f;    // hip segment
const float FEMUR_LENGTH  = 120.0f;   // thigh segment
const float TIBIA_LENGTH  = 217.0f + 5.5f; // shin + rubber foot pad
const float HEIGHT_OFFSET = 0.0f;     // per-leg vertical offset of the hip
const float FOOT_EXTEND   = 170.0f;   // how far the foot sits out from the body

// ---- Gait / motion tuning --------------------------------------------------
const float    MAX_STEP_WIDTH   = 230.0f; // mm
const uint16_t WALKING_STEPS    = 70;     // interpolation steps per phase
const uint16_t MAIN_LOOP_DELAY  = 5;      // ms between gait ticks
const float    BODY_HEIGHT      = 200.0f; // mm above the ground

// One RobotLeg per leg. Each leg is independent: here we place 5 legs evenly
// at 72 degree intervals, but the angles (and any other parameter) can differ
// per leg for an asymmetric robot.
RobotLeg legs[NUMBER_OF_LEGS] = {
    RobotLeg(BODY_RADIUS, COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, HEIGHT_OFFSET, FOOT_EXTEND,   0),
    RobotLeg(BODY_RADIUS, COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, HEIGHT_OFFSET, FOOT_EXTEND,  72),
    RobotLeg(BODY_RADIUS, COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, HEIGHT_OFFSET, FOOT_EXTEND, 144),
    RobotLeg(BODY_RADIUS, COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, HEIGHT_OFFSET, FOOT_EXTEND, 216),
    RobotLeg(BODY_RADIUS, COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, HEIGHT_OFFSET, FOOT_EXTEND, 288),
};

RobotWithKinematics *robot = nullptr;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    robot = new RobotWithKinematics(
        NUMBER_OF_LEGS,  // number of legs (must match the macro)
        MAX_STEP_WIDTH,  // max step width in mm
        WALKING_STEPS,   // interpolation steps per gait phase
        MAIN_LOOP_DELAY, // ms between gait ticks
        legs             // the pre-configured legs
    );

    // Stand still at the start.
    robot->setPose(BODY_HEIGHT, 0.0f, 0.0f, 0.0f);
    robot->setBaseFootExtend(FOOT_EXTEND);
}

void loop()
{
    // 2) Feed the control inputs. Normally these come from a joystick / RC.
    //    Here we just walk forward at half the max step width.
    float walkX   = 0.0f;                 // strafe left/right (mm)
    float walkY   = MAX_STEP_WIDTH * 0.5f;// forward/backward (mm)
    float rotate  = 0.0f;                 // turn on the spot (deg)

    robot->applyControls(
        walkX, walkY, rotate, // walk direction + yaw rate
        FOOT_EXTEND,          // foot extension (mm)
        BODY_HEIGHT,          // body height (mm)
        0.0f, 0.0f, 0.0f);    // body tilt X, tilt Z, yaw (deg)

    // 3) Advance the state machines, in this order, once per loop.
    robot->mainLoop();
    robot->prepareTargetPositions();
    robot->specialPoseLoop();

    // 4) Solve inverse kinematics for every leg.
    std::array<LegAngles, NUMBER_OF_LEGS> angles = robot->calculateAllLegAngles();

    // 5) Use the angles. Replace this with your own servo writes.
    for (uint8_t i = 0; i < NUMBER_OF_LEGS; i++)
    {
        if (angles[i].valid)
        {
            // angles[i].coxaDeg(), femurDeg(), tibiaDeg() are degrees.
            // Map them to your servo's units and write them here.
        }
        printLegAngles(angles[i], i);
        Serial.println();
    }
    Serial.println("----");

    delay(MAIN_LOOP_DELAY);
}
