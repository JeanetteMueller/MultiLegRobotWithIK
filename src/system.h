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
#include "leds.h"

uint32_t previousStepMillis = 0;
int8_t standByLeg = 0;
bool standby = false;
uint8_t preOperationalCount = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("SETUP");

    Serial1.begin(1000000, SERIAL_8N1, 18, 19);

    setup_leds();

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

void legsInStandby()
{
    updateAllLed(CRGB::Red);

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

void extendLegsFromStandby()
{
    updateAllLed(CRGB::Yellow);

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

void waveWithLeg()
{
    Serial.println("I CAN USE MY HANDY");

    if (input->switchRightOutside == Top || input->switchRightOutside == Bottom)
    {

        LegAngles la;
        la.valid = true;
        la.coxa = degToRad(waveLegA);
        la.femur = degToRad(50);
        la.tibia = degToRad(50);

        if (waveLegADirection)
        {
            waveLegA += 0.4;
        }
        else
        {
            waveLegA -= 0.4;
        }

        if (waveLegA > 60 || waveLegA < -60)
        {
            waveLegADirection = !waveLegADirection;
        }

        if (input->switchRightOutside == Top)
        {
            moveOneLeg(2, la, false);
        }
        else if (input->switchRightOutside == Bottom)
        {
            moveOneLeg(3, la, false);
        }
    }
}

void loop()
{
    loopInput();

    if (calibrated == false)
    {
        updateAllLed(CRGB::Red);
        FastLED.show();

        calibrate();
        calibrated = true;

        updateAllLed(CRGB::Green);
        FastLED.show();
    }

    if (input->switchLeftInside == Top)
    {
        Serial.println("standby");
        legsInStandby();
    }
    else if (standby == true)
    {
        Serial.println("standby is ending soon");
        extendLegsFromStandby();
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
        float tiltY = fmap(input->leftStickVertical, 100.0, -100.0, maxTilt, -maxTilt);
        float tiltX = fmap(input->leftStickHorizontal, -100.0, 100.0, maxTilt, -maxTilt);
        float rotateTorso = fmap(input->rightPoti, 0.0, 1000.0, -maxRotation, maxRotation);

        robot->setPose(height, tiltX, tiltY, rotateTorso);

        robot->mainLoop();

        float walkX = fmap(input->rightStickVertical, 100, -100, -maxStepWidth, maxStepWidth);
        float walkY = fmap(input->rightStickHorizontal, 100, -100, -maxStepWidth, maxStepWidth);

        float rotateBody = 0;
        if (input->switchLeftOutside == Top)
        {
            rotateBody = maxRotationBodyOnPoint;
        }
        else if (input->switchLeftOutside == Bottom)
        {
            rotateBody = -maxRotationBodyOnPoint;
        }

        robot->setWalkDirection(walkX, walkY, rotateBody);

        robot->prepareTargetPositions();

        std::array<LegAngles, RobotWithKinematics::MAX_NUM_LEGS> allAngles = robot->calculateAllLegAngles();

        if (robot->isValidPose())
        {
            // Serial.println("leg angles good");

            loop_leds();

            moveAllLegs(allAngles);

            if (walkX < 1.0 && walkX > -1.0 && walkY < 1.0 && walkY > -1.0 && rotateBody == 0 && height > (minHeight + (maxHeight - minHeight) / 2))
            {
                waveWithLeg();
            }

            finalizeServoPositions();
        }
        else
        {
            // Serial.println("leg angles not reachable!!! ");

            updateAllLed(CRGB::Red);
        }
    }

    FastLED.show();
    // delay(1);
}
