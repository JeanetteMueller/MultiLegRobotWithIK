
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

    robot = new CircularRobot(NUMBER_OF_LEGS,       // Number of Legs
                              157.982, // body radius in mm
                              120.0,   // thigh length in mm
                              175.0,   // shin length in mm
                              200.0);  // base position over ground in mm

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

    robot->setBodyHeight(fmap(input->leftPoit, 0.0, 100.0, 130.0, 294.0));

    robot->setTilt(fmap(input->leftStickVertical, -100.0, 100.0, -10.0, 10.0),
                   fmap(input->rightPoti, 0.0, 1000.0, -5.5, 5.5),
                   fmap(input->leftStickHorizontal, -100.0, 100.0, -10.0, 10.0));

    std::vector<JointAngles> angles = robot->calculateAllLegsIK();

    if (robot->allLegAnglesAreReachable(angles))
    {
        for (int i = 0; i < NUMBER_OF_LEGS; i++)
        {
            //robot->printLegAngles(i, angles[i]);

            setDegreeForLegAndServo(i, 0, 180 + (angles[i].theta0 * 180.0 / M_PI) - (i * 72.0), speed, acc);
            setDegreeForLegAndServo(i, 1, -(angles[i].theta1 * 180.0 / M_PI), speed, acc);
            setDegreeForLegAndServo(i, 2, (angles[i].theta2 * 180.0 / M_PI) - 180, speed, acc);
        }
        finalizeServoPositions();
    }
}