/**
 * system.h
 *
 * Zentrale Logic für Setup und Mainloop des ESP32.
 *
 * Autor: Jeanette Müller
 * Datum: 2025
 */

#include <Arduino.h>
#include <random>

#include <SCServo.h>
#include "classes/Vector3.h"

#include "classes/RobotWithKinematics.h"
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

    for (uint8_t i = 0; i < NUMBER_OF_LEGS; i++)
    {
        extraCalibrations[i] = LegAngles();
    }

    // extraCalibrations[2].coxa = -3 * M_PI / 180.0;
    // extraCalibrations[2].swing = 2 * M_PI / 180.0;

    // extraCalibrations[4].lift = -2 * M_PI / 180.0;
    // extraCalibrations[4].swing = -1 * M_PI / 180.0;

    robot = new RobotWithKinematics(bodyCenterToLegsCircleRadius, // body radius in mm
                                    NUMBER_OF_LEGS,               // Number of Legs
                                    coxaLength,                   // coxa length in mm
                                    thighLength,                  // thigh length in mm
                                    shinLength,                   // shin length in mm
                                    startLegExtend);              // base food extend in mm

    robot->setPose(startBodyHeightOverGround, 0.0, 0.0, 0.0);

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

    float legExtend = 0;

    if (input->switchRightInside == 0)
    {
        legExtend = 50;
    }
    else if (input->switchRightInside == 1)
    {
        legExtend = 110;
    }
    else if (input->switchRightInside == 2)
    {
        legExtend = 190;
    }

    // float legExtend = fmap(input->rightPoti, 0.0, 1000.0, minLegExtend, maxLegExtend);
    robot->setBaseFootExtend(legExtend);

    float height = fmap(input->leftPoit, 0.0, 1000.0, minHeight, maxHeight);
    float tiltX = fmap(input->leftStickVertical, -100.0, 100.0, -maxTilt, maxTilt);
    float tiltY = fmap(input->leftStickHorizontal, -100.0, 100.0, -maxTilt, maxTilt);
    float rotate = fmap(input->rightPoti, 0.0, 1000.0, -maxRotation, maxRotation);

    robot->setPose(height, tiltX, tiltY, rotate);

    if (scorpionLeg > -1)
    {
        robot->setScorpionLeg(scorpionLeg);
        // robot->setTiltTowardsLeg(scorpionLeg, 15.0);
    }

    robot->mainLoop();

    double walkX = fmap(input->rightStickVertical, -100, 100, -maxStepWidth, maxStepWidth);
    double walkY = fmap(input->rightStickHorizontal, -100, 100, -maxStepWidth, maxStepWidth);

    robot->setWalkDirection(walkX, walkY);

    robot->prepareTargetPositions();

    std::array<LegAngles, RobotWithKinematics::MAX_NUM_LEGS> allAngles = robot->calculateAllLegAngles();

    if (robot->isValidPose())
    {
        // Serial.println("leg angles good");
        leds[1] = CRGB::Blue;

        moveAllLegs(allAngles);
    }
    else
    {
        Serial.println("leg angles not reachable!!! ");
        leds[1] = CRGB::Red;

        robot->setWalkDirection(0, 0);

        robot->resetTargetPositions();

        allAngles = robot->calculateAllLegAngles();

        if (robot->isValidPose())
        {
            moveAllLegs(allAngles);
        }
    }
    FastLED.show();
    // delay(1);
}
