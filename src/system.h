
#include <SCServo.h>
#include "classes/CircularRobot.h"

#include "definitions.h"
#include "basicFunctions.h"
#include "servoFunctions.h"
#include "input.h"

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

    for (int i = 0; i < NUMBER_OF_LEGS; i++) {
        extraCalibrations.push_back(JointAngles()); 
    }

    extraCalibrations[2].theta1 = -2;
    extraCalibrations[4].theta1 = -2;

    robot = new CircularRobot(NUMBER_OF_LEGS, // Number of Legs
                              157.982,        // body radius in mm
                              120.0,          // thigh length in mm
                              175.0,          // shin length in mm
                              200.0);         // base position over ground in mm

    setupInput();

    initServoPositions();
}



void loop()
{
    loopInput();

    if (calibrated == false)
    {
        calibrate();
        calibrated = true;
    }

    u16 speed = 2000;
    u8 acc = 0;

    float maxRotation = 18.0;

    // robot->setBodyHeight(fmap(input->leftPoit, 0.0, 100.0, 130.0, 290.0));
    robot->setBodyHeight(fmap(input->leftPoit, 0.0, 100.0, 20.0, 290.0));

    robot->setTilt(fmap(input->leftStickVertical, -100.0, 100.0, -10.0, 10.0),
                   fmap(input->rightPoti, 0.0, 1000.0, -maxRotation, maxRotation),
                   fmap(input->leftStickHorizontal, -100.0, 100.0, -10.0, 10.0));

    robot->setWalkDirection(fmap(input->rightStickHorizontal, -100.0, 100.0, -50.0, 50.0),
                            fmap(input->rightStickVertical, -100.0, 100.0, -50.0, 50.0),
                            10);

    robot->mainLoop();

    std::vector<JointAngles> angles = robot->calculateAllLegsIK();

    if (robot->allLegAnglesAreReachable(angles))
    {
        for (int i = 0; i < NUMBER_OF_LEGS; i++)
        {
            if (i == 0)
            {
                robot->printLegAngles(i, angles[i]);
            }

            JointAngles calibration = extraCalibrations[i];

            // lift
            setDegreeForLegAndServo(i, 0, calibration.theta1 + (angles[i].theta1 * 180.0 / M_PI), speed, acc);

            // rotate
            float rValue = (angles[i].theta0 * 180.0 / M_PI);
            if (rValue < 0 && i > 1)
            {
                rValue = 360 + rValue;
            }
            rValue = rValue - (i * (360.0 / NUMBER_OF_LEGS));

            setDegreeForLegAndServo(i, 1, calibration.theta0 + rValue, speed, acc);

            // knie
            setDegreeForLegAndServo(i, 2, calibration.theta2 + 180 - (angles[i].theta2 * 180.0 / M_PI), speed, acc);
        }

        finalizeServoPositions();
    }else {
        Serial.println("leg angles not reachable!!!");
    }
}