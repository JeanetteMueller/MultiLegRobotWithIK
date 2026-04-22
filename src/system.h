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

#include "basicFunctions.h"
#include "../definitions.h"
#include "servoFunctions.h"
#include "input.h"
#include "footSensors.h"

uint32_t previousStepMillis = 0;

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

    setupExtraCalibrations();

    robot = new RobotWithKinematics(bodyCenterToLegsCircleRadius, // body radius in mm
                                    NUMBER_OF_LEGS,               // Number of Legs
                                    coxaLength,                   // coxa length in mm
                                    thighLength,                  // thigh length in mm
                                    shinLength,                   // shin length in mm
                                    startLegExtend,               // base food extend in mm
                                    maxStepWidth                  // max step in mm
    );

    robot->setPose(startBodyHeightOverGround, 0.0, 0.0, 0.0);

    setupInput();

    setupFootSensors();

    initServoPositions();

    delay(200);
    loopInput();
    delay(500);
}

void waveWithLeg()
{
    // Serial.println("I CAN USE MY HANDY");

    if (input->switchRightOutside == Top || input->switchRightOutside == Bottom)
    {

        float maxRotation = 28;
        float rotationStep = 0.3;

        LegAngles la;
        la.valid = true;
        la.coxa = degToRad(waveLegA);
        la.femur = degToRad(60);
        la.tibia = degToRad(-120);

        if (waveLegADirection)
        {
            waveLegA += rotationStep;
        }
        else
        {
            waveLegA -= rotationStep;
        }

        if (waveLegA > maxRotation || waveLegA < -maxRotation)
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

    loopFootSensors();

    if (calibrated == false)
    {
        calibrate();
        calibrated = true;
    }

    previousStepMillis = 0;
    float legExtend = 170;

    float height = fmap(input->leftPoit, 0.0, 1000.0, minHeight, maxHeight);
    float tiltY = fmap(input->leftStickVertical, -100.0, 100.0, maxTilt, -maxTilt);
    float tiltX = fmap(input->leftStickHorizontal, 100.0, -100.0, maxTilt, -maxTilt);
    float rotateTorso = fmap(input->rightPoti, 0.0, 1000.0, -maxRotation, maxRotation);

    bool vehicleSteering = input->switchLeftInside == Top;

    float walkX = 0;
    float walkY = 0;
    float rotateBody = 0;

    if (vehicleSteering)
    {
        // Fahrzeug-Modus: Y = Geschwindigkeit, X = Lenkung
        walkX = fmap(input->rightStickVertical, -100.0, 100.0, maxStepWidth, -maxStepWidth);
        rotateBody = fmap(input->rightStickHorizontal, -100.0, 100.0, -maxRotationBodyOnPoint, maxRotationBodyOnPoint);
    }
    else
    {
        // Strafing-Modus: direktionale Bewegung
        walkX = fmap(input->rightStickVertical, -100.0, 100.0, maxStepWidth, -maxStepWidth);
        walkY = fmap(input->rightStickHorizontal, -100.0, 100.0, maxStepWidth, -maxStepWidth);
    }

    if (input->switchLeftOutside == Top)
    {
        robot->doSpecialPose(0);
    }
    else if (input->switchLeftOutside == Bottom)
    {
        robot->doSpecialPose(1);
    }

    robot->setWalkDirection(walkX, walkY, rotateBody);
    robot->setBaseFootExtend(legExtend);
    robot->setPose(height, tiltX, tiltY, rotateTorso);
    robot->setStepSmoothness(1.0);

    robot->mainLoop();

    // Serial.print("height: ");
    // Serial.print(height);

    // Serial.println();

    robot->prepareTargetPositions();

    // Sonderpose-State-Machine: schreibt ggf. targetPosition[] für die beteiligten Beine
    robot->specialPoseLoop();

    std::array<LegAngles, RobotWithKinematics::MAX_NUM_LEGS> allAngles = robot->calculateAllLegAngles();

    if (robot->isValidPose())
    {
        // Serial.println("leg angles good");

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

        
    }

    // delay(1);
}