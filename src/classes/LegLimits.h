/**
 * LegLimits.h
 *
 * C++ structs for the joint limits of a leg (in degrees).
 *
 * Previously the limits were hard-coded in LegAngles.h and therefore only
 * suited Rocky. Via these structs they are configured per robot in
 * definitions.h and passed to each RobotLeg.
 *
 * Author: Claude.ai & Jeanette Müller
 * Date: 2025
 */

#ifndef LegLimits_H
#define LegLimits_H

// Allowed value range of a single joint (in degrees).
// The default is deliberately wide (±180°) so that an unconfigured leg does not
// accidentally block all poses.
struct Limits
{
    float min = -180.0f;
    float max = 180.0f;

    // true if the angle lies within (inclusive of) the bounds
    bool contains(float value) const
    {
        return value >= min && value <= max;
    }
};

// Limits for all 3 joints of a leg.
struct LegLimits
{
    Limits coxa;  // swing around the vertical axis
    Limits femur; // lift
    Limits tibia; // knee
};

#endif // LegLimits_H
