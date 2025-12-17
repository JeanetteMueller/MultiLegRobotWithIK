#include <Arduino.h>
#include <random>

#include <SCServo.h>
#include "classes/CircularRobot.h"
#include <FastLED.h>

#include "../definitions.h"
#include "basicFunctions.h"
#include "servoFunctions.h"
#include "input.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("SETUP");

    Serial1.begin(1000000, SERIAL_8N1, 18, 19);

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(50);

    st.pSerial = &Serial1;
    while (!Serial1)
    {
        delay(500);
    }

    for (int i = 0; i < NUMBER_OF_LEGS; i++)
    {
        extraCalibrations.push_back(JointAngles());
    }

    extraCalibrations[2].theta0 = -2;
    extraCalibrations[4].theta0 = -2;

    robot = new CircularRobot(NUMBER_OF_LEGS, // Number of Legs
                              157.982,        // body radius in mm
                              120.0,          // thigh length in mm
                              175.0,          // shin length in mm
                              175.0 + 25.0);         // base position over ground in mm

    setupInput();

    initServoPositions();
}

void loop()
{
    loopInput();

    if (calibrated == false)
    {
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Red;
        FastLED.show();

        calibrate();
        calibrated = true;

        leds[0] = CRGB::Green;
        FastLED.show();
    }

    float maxRotation = 70.0;
    float walkingRange = 50.0;

    // float random_10 = dis10(gen);
    // robot->setBodyHeight(5.0);
    // robot->setBodyHeight(fmap(random_10, -10.0, 10.0, 30.0, 250.0));
    robot->setBodyHeight(fmap(input->leftPoit, 0.0, 1000.0, 30.0, 290.0));

    robot->setTilt(fmap(input->leftStickVertical, -100.0, 100.0, -10.0, 10.0),
                   fmap(input->rightPoti, 0.0, 1000.0, -maxRotation, maxRotation),
                   fmap(input->leftStickHorizontal, -100.0, 100.0, -10.0, 10.0));

    robot->setWalkDirection(fmap(input->rightStickHorizontal, -100.0, 100.0, -walkingRange, walkingRange),
                            fmap(input->rightStickVertical, -100.0, 100.0, -walkingRange, walkingRange));

    // float random_x = dis5(gen);
    // float random_y = dis5(gen);

    // robot->setWalkDirection(random_x, random_y);
    // robot->setWalkDirection(20, 0);

    robot->mainLoop();

    std::vector<JointAngles> angles = robot->calculateAllLegsIK();

    for (int legIndex = 0; legIndex < NUMBER_OF_LEGS; legIndex++)
        {
robot->printLegAngles(legIndex, angles[legIndex]);
        }
    int allInReach = robot->allLegAnglesAreReachable(angles);

    if (allInReach == -1)
    {

        leds[1] = CRGB::Blue;

        Serial.println("leg angles good");

        for (int legIndex = 0; legIndex < NUMBER_OF_LEGS; legIndex++)
        {
            // if (legIndex == 1)
            // {
                // robot->printLegAngles(legIndex, angles[legIndex]);
            // }

            JointAngles calibration = extraCalibrations[legIndex];

            // lift
            float degree0 = angles[legIndex].theta0;
            setDegreeForLegAndServo(legIndex, 0, calibration.theta0 + degree0, speed, acc);

            // rotate
            float degree1 = angles[legIndex].theta1;
            setDegreeForLegAndServo(legIndex, 1, calibration.theta1 + degree1, speed, acc);

            // knie
            float degree2 = angles[legIndex].theta2;
            setDegreeForLegAndServo(legIndex, 2, calibration.theta2 + 180 - degree2, speed, acc);
        }

        finalizeServoPositions();
    }
    else
    {
        leds[1] = CRGB::Red;

        JointAngles notInReach = angles[allInReach];
        Vector3 target = robot->targetPosition[allInReach];

        Serial.print("    Leg target ");
        Serial.print(allInReach);
        Serial.print(":  ");
        Serial.print(target.x);
        Serial.print(" ");
        Serial.print(target.y);
        Serial.print(" ");
        Serial.print(target.z);


        Serial.println(" -> leg angles not reachable!!! ");
    }

    FastLED.show();

    delay(100);
}