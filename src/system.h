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

#include "classes/PentapodKinematics.h"
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

    extraCalibrations[2].lift = -3 * M_PI / 180.0;
    extraCalibrations[2].swing = 2 * M_PI / 180.0;

    extraCalibrations[4].lift = -2 * M_PI / 180.0;
    extraCalibrations[4].swing = -1 * M_PI / 180.0;

    robot = new PentapodKinematics(157.982, // body radius in mm
                                   120.0,   // thigh length in mm
                                   175.0);  // shin length in mm

    robot->setPose(175.0 - 75.0, 0.0, 0.0, 0.0);

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

    robot->setBaseFootExtend(robotControl.legextend / 10);

    robot->mainLoop();

    float height = robotControl.height / 10;

    float tiltX = fmap(robotControl.roll, -100.0, 100.0, -maxTilt, maxTilt);
    float tiltY = fmap(robotControl.pitch, -100.0, 100.0, -maxTilt, maxTilt);
    float rotate = fmap(robotControl.yaw, -100.0, 100.0, -maxRotation, maxRotation);

    robot->setPose(height, tiltX, tiltY, rotate);

    double walkX = fmap(robotControl.joystickY, -100, 100, -maxStepWidth, maxStepWidth);
    double walkY = fmap(robotControl.joystickX, -100, 100, -maxStepWidth, maxStepWidth);
    robot->setWalkDirection(walkX, walkY);

    robot->prepareTargetPositions();

    std::array<LegAngles, PentapodKinematics::NUM_LEGS> allAngles = robot->calculateAllLegAngles();

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

        for (uint8_t legIndex = 0; legIndex < PentapodKinematics::NUM_LEGS; legIndex++)
        {
            LegAngles calibration = extraCalibrations[legIndex];

            LegAngles angles = allAngles[legIndex];

            setDegreeForLegAndServo(legIndex, 0, calibration.liftDeg() + angles.liftDeg(), speed, acc);
            setDegreeForLegAndServo(legIndex, 1, calibration.swingDeg() + angles.swingDeg(), speed, acc);
            setDegreeForLegAndServo(legIndex, 2, calibration.kneeDeg() + -angles.kneeDeg(), speed, acc);
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