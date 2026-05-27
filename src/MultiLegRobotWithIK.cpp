/**
 * MultiLegRobotWithIK.h
 *
 * Arduino IDE kompatible Projektdatei um dieses Projekt auf einen ESP32 zu laden.
 *
 * Autor: Jeanette Müller
 * Datum: 2025
 */

#define NUMBER_OF_LEGS 5
static_assert(NUMBER_OF_LEGS >= 4, "MultiLegRobotWithIK requires at least 4 legs");

#include "system.h"

// #include "src/system_test.h"