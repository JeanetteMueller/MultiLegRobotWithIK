/**
 * MultiLegRobotWithIK.h
 *
 * Inverse kinematics and gait control for a walking robot with a configurable
 * number of 3-DOF legs (4 or more). This is the single header you include.
 *
 * You MUST define NUMBER_OF_LEGS before including this header, e.g.:
 *
 *     #define NUMBER_OF_LEGS 5
 *     #include <MultiLegRobotWithIK.h>
 *
 * Author: Jeanette Müller, 2025
 * https://github.com/JeanetteMueller/MultiLegRobotWithIK
 */

#ifndef MULTI_LEG_ROBOT_WITH_IK_H
#define MULTI_LEG_ROBOT_WITH_IK_H

#include <Arduino.h>

#ifndef NUMBER_OF_LEGS
#error "MultiLegRobotWithIK: please #define NUMBER_OF_LEGS (>= 4) before including this library."
#endif

#include "classes/RobotWithKinematics.h"
#include "basicFunctions.h"

#endif // MULTI_LEG_ROBOT_WITH_IK_H
