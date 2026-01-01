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
#include "webservice.h"

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

    setupWebServer();

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
    webSocket.loop();
    server.handleClient();

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

    // float height = fmap(input->leftPoit, 0.0, 1000.0, 50.0, 290.0);
    // float tiltX = fmap(input->leftStickVertical, -100.0, 100.0, -28.0, 28.0);
    // float tiltY = fmap(input->leftStickHorizontal, -100.0, 100.0, -28.0, 28.0);
    // float rotate = fmap(input->rightPoti, 0.0, 1000.0, -20.0, 20.0);

    float legExtend = fmap(robotControl.legextend, 0.0, 1000.0, minLegExtend, maxLegExtend);
    robot->setBaseFootExtend(legExtend);

    float height = fmap(robotControl.height, 0.0, 1000.0, minHeight, maxHeight);
    float tiltX = fmap(robotControl.roll, -100.0, 100.0, -maxTilt, maxTilt);
    float tiltY = fmap(robotControl.pitch, -100.0, 100.0, -maxTilt, maxTilt);
    float rotate = fmap(robotControl.yaw, -100.0, 100.0, -maxRotation, maxRotation);

    robot->setPose(height, tiltX, tiltY, rotate);

    if (scorpionLeg > -1)
    {
        robot->setScorpionLeg(scorpionLeg);
        robot->setTiltTowardsLeg(scorpionLeg, 15.0);
    }

    robot->mainLoop();

    double walkX = fmap(robotControl.joystickY, -100, 100, -maxStepWidth, maxStepWidth);
    double walkY = fmap(robotControl.joystickX, -100, 100, -maxStepWidth, maxStepWidth);
    robot->setWalkDirection(walkX, walkY);

    robot->prepareTargetPositions();

    std::array<LegAngles, RobotWithKinematics::MAX_NUM_LEGS> allAngles = robot->calculateAllLegAngles();

    if (robot->isValidPose() == false)
    {
        robot->setWalkDirection(walkX * 0.6, walkY * 0.6);

        robot->prepareTargetPositions();

        allAngles = robot->calculateAllLegAngles();
    }

    if (robot->isValidPose())
    {
        // Serial.println("leg angles good");
        leds[1] = CRGB::Blue;

        for (uint8_t legIndex = 0; legIndex < robot->NUM_LEGS; legIndex++)
        {
            LegAngles calibration = extraCalibrations[legIndex];

            if (robot->currentScorpionLeg == legIndex)
            {
                setDegreeForLegAndServo(legIndex, 0, calibration.coxaDeg() + 0, speed, acc);
                setDegreeForLegAndServo(legIndex, 1, calibration.femurDeg() + 45.0, speed, acc);
                setDegreeForLegAndServo(legIndex, 2, calibration.tibiaDeg() + 45.0, speed, acc);
            }
            else
            {

                LegAngles angles = allAngles[legIndex];

                setDegreeForLegAndServo(legIndex, 0, calibration.coxaDeg() + angles.coxaDeg(), speed, acc);
                setDegreeForLegAndServo(legIndex, 1, calibration.femurDeg() + angles.femurDeg(), speed, acc);
                setDegreeForLegAndServo(legIndex, 2, calibration.tibiaDeg() + -angles.tibiaDeg(), speed, acc);
            }
        }
        finalizeServoPositions();
    }
    else
    {
        Serial.println("leg angles not reachable!!! ");
        leds[1] = CRGB::Red;
    }

    FastLED.show();

    delay(1);
}