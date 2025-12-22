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
#define NUM_LEDS 2
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

SMS_STS st;

#define NUMBER_OF_LEGS 5
#define SERVO_NUM (NUMBER_OF_LEGS * 3) // Number of servos
u8 servoIds[SERVO_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
s16 newPosition[SERVO_NUM];
u16 newSpeed[SERVO_NUM];
u8 newAcc[SERVO_NUM];

u16 speed = 4000;
u8 acc = 0;

bool calibrated = false;

PentapodKinematics *robot;

std::array<LegAngles, NUMBER_OF_LEGS> extraCalibrations;

float maxTilt = 35.0;
float maxRotation = 25.0;
float maxStepWidth = 80.0;