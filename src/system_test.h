#include <Arduino.h>
#include <random>

#include <SCServo.h>
#include "classes/LegAngles.h"

#include "basicFunctions.h"

bool debug = false;
SMS_STS st;

#define NUMBER_OF_LEGS 5
#define SERVO_NUM (NUMBER_OF_LEGS * 3) // Number of servos
u8 servoIds[SERVO_NUM] = {5, 14, 15, 7, 8, 9, 4, 2, 3, 10, 11, 6, 13, 1, 12};
s16 newPosition[SERVO_NUM];
u16 newSpeed[SERVO_NUM];
u8 newAcc[SERVO_NUM];

u16 speed = 4000;
u8 acc = 0;

bool calibrated = false;

#include "servoFunctions.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("SETUP");
    Serial1.begin(1000000, SERIAL_8N1, 18, 19);

    st.pSerial = &Serial1;
    while (!Serial1)
    {
        delay(500);
    }
}

uint8_t servoNumber = 1;
// float value = 45;

void loop()
{
    Serial.println("LOOP");

    // unbekannt
    //  2 (evtl bein 2 2ter servo)
    //  7

    // Beine front
    // 0 = 5 14 15
    // 1 = 7 8 9
    // 2 = 4 2 3
    // 3 = 10 11 6
    // 4 = 13 1 12

    if (calibrated == false)
    {

        calibrate();

        calibrated = true;

        initServoPositions();

        finalizeServoPositions();

        delay(1000);
    }

    // newPosition[servoNumber - 1] = calcSerialServoDegree(180 + value);

    finalizeServoPositions();

    delay(1000);

    // servoNumber++;
    // if (servoNumber > 15) {
    //     servoNumber = 1;

    //     value = -value;
    // }
}