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


#include "basicFunctions.h"

#include "../definitions.h"

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

    setupExtraCalibrations();

    robot = new RobotWithKinematics(bodyCenterToLegsCircleRadius, // body radius in mm
                                    NUMBER_OF_LEGS,               // Number of Legs
                                    coxaLength,                   // coxa length in mm
                                    thighLength,                  // thigh length in mm
                                    shinLength,                   // shin length in mm
                                    startLegExtend);              // base food extend in mm

    robot->setPose(startBodyHeightOverGround, 0.0, 0.0, 0.0);

    setupInput();

    initServoPositions();

    delay(200);
    loopInput();
    delay(500);
}
uint32_t previousStepMillis = 0;
int8_t standByLeg = 0;
bool standby = false;
uint8_t preOperationalCount = 0;

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

    if (input->switchLeftInside == Top)
    {
        Serial.println("standby");

        standby = true;
        preOperationalCount = 0;

        if (millis() - previousStepMillis >= 700)
        {
            previousStepMillis = millis();

            std::array<LegAngles, RobotWithKinematics::MAX_NUM_LEGS> standByAngles = robot->standBy();

            LegAngles angles = standByAngles[standByLeg];
            moveOneLeg(standByLeg, angles, true);

            standByLeg++;

            if (standByLeg > robot->NUM_LEGS - 1)
            {
                standByLeg = 0;
            }
        }
    }
    else if (standby == true)
    {
        Serial.println("standby is ending soon");

        if (millis() - previousStepMillis >= 700)
        {
            previousStepMillis = millis();

            if (preOperationalCount == 0)
            {
                standByLeg = robot->NUM_LEGS - 1;
            }

            if (standByLeg >= 0)
            {
                std::array<LegAngles, RobotWithKinematics::MAX_NUM_LEGS> standByAngles = robot->preOperationPositions();

                LegAngles angles = standByAngles[standByLeg];
                moveOneLeg(standByLeg, angles, true);

                preOperationalCount++;
            }
            standByLeg--;

            if (standByLeg < -2)
            {
                standby = false;
            }
        }
    }
    else
    {
        previousStepMillis = 0;
        float legExtend = 0;

        if (input->switchRightInside == 0)
        {
            legExtend = 190;
        }
        else if (input->switchRightInside == 1)
        {
            legExtend = 120;
        }
        else if (input->switchRightInside == 2)
        {
            legExtend = 70;
        }

        // float legExtend = fmap(input->rightPoti, 0.0, 1000.0, minLegExtend, maxLegExtend);
        robot->setBaseFootExtend(legExtend);

        float height = fmap(input->leftPoit, 0.0, 1000.0, minHeight, maxHeight);
        float tiltX = fmap(input->leftStickVertical, -100.0, 100.0, -maxTilt, maxTilt);
        float tiltY = fmap(input->leftStickHorizontal, -100.0, 100.0, -maxTilt, maxTilt);
        float rotateTorso = fmap(input->rightPoti, 0.0, 1000.0, -maxRotation, maxRotation);

        robot->setPose(height, tiltX, tiltY, rotateTorso);

        robot->mainLoop();

        float walkX = fmap(input->rightStickVertical, -100, 100, -maxStepWidth, maxStepWidth);
        float walkY = fmap(input->rightStickHorizontal, -100, 100, -maxStepWidth, maxStepWidth);

        float rotateBody = 0;
        if (input->switchLeftOutside == Top) {
            rotateBody = maxRotationBodyOnPoint;
        }else if (input->switchLeftOutside == Bottom) {
            rotateBody = -maxRotationBodyOnPoint;
        }

        robot->setWalkDirection(walkX, walkY, rotateBody);

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
            // Serial.println("leg angles not reachable!!! ");
            // leds[1] = CRGB::Red;

            // // robot->setWalkDirection(0, 0);

            // robot->resetTargetPositions();

            // allAngles = robot->calculateAllLegAngles();

            // if (robot->isValidPose())
            // {
            //     moveAllLegs(allAngles);
            // }
        }
    }
    FastLED.show();
    // delay(1);
}
