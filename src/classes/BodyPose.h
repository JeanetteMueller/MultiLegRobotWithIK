/**
 * BodyPose.h
 *
 * C++ struct to define the target values of the robot.
 *
 * Author: Jeanette Müller
 * Date: 2025
 */

#ifndef BodyPose_H
#define BodyPose_H

#include "BodyPose.h"

// Struct for body parameters
struct BodyPose
{
    float height; // height of the body above the ground (mm)
    float tiltX;  // tilt around the X axis / pitch (radians)
    float tiltZ;  // tilt around the Z axis / roll (radians)
    float rotY;   // rotation around the Y axis / yaw (radians)

    float bodyShiftX = 0.0f; // horizontal shift of the body center in X (mm)
    float bodyShiftZ = 0.0f; // horizontal shift of the body center in Z (mm)

    // Constructor with default values
    BodyPose() : height(200.0), tiltX(0.0), tiltZ(0.0), rotY(0.0) {}

    // Constructor with angles in degrees
    static BodyPose fromDegrees(float height,
                                float tiltXDeg, float tiltZDeg, float rotYDeg)
    {
        BodyPose pose;
        pose.height = height;
        pose.tiltX = tiltXDeg * M_PI / 180.0f;
        pose.tiltZ = tiltZDeg * M_PI / 180.0f;
        pose.rotY = rotYDeg * M_PI / 180.0f;
        return pose;
    }
};

#endif