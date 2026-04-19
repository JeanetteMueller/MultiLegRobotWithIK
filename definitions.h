/**
 * definitions.h
 *
 * Basis Variablen
 *
 * Autor: Jeanette Müller
 * Datum: 2025
 */

bool debug = false;

#define LED_PIN 23
#define NUM_LEDS 21
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

SMS_STS st;

#define NUMBER_OF_LEGS 5
#define SERVO_NUM (NUMBER_OF_LEGS * 3) // Number of servos
// u8 servoIds[SERVO_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
u8 servoIds[SERVO_NUM] = {5, 14, 15, 7, 8, 9, 4, 2, 3, 10, 11, 6, 13, 1, 12};
s16 newPosition[SERVO_NUM];
u16 newSpeed[SERVO_NUM];
u8 newAcc[SERVO_NUM];

u16 speed = 4000;
u8 acc = 0;

bool calibrated = false;

RobotWithKinematics *robot;

#include "../extraCalibrations.h"

float bodyCenterToLegsCircleRadius = 104.175; // mm
float coxaLength = 54.0;                      // mm
float thighLength = 120.0;                    // mm
float shinLength = 217.0 + 5.5;               // mm // shin + rubber pad
float startBodyHeightOverGround = 160.0;      // mm
float startLegExtend = 150.0;                 // mm

float maxTilt = 20.0;
float maxRotation = 25.0;
float maxStepWidth = 220.0;

float minHeight = 170.0; // mm
float maxHeight = 300.0; // mm

float maxRotationBodyOnPoint = 200.0; // mm


float waveLegA = 0;
bool waveLegADirection = true;