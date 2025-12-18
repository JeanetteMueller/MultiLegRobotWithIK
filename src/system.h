#include <Arduino.h>
#include <random>

#include <SCServo.h>
#include "classes/Vector3.h"

// #include "classes/CircularRobot.h"
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
    extraCalibrations[4].lift = -2 * M_PI / 180.0;

    robot = new PentapodKinematics(157.982, // body radius in mm
                                   120.0,   // thigh length in mm
                                   175.0);  // shin length in mm

    robot->setPose(175.0 - 75.0, 120.0, 0.0, 0.0, 0.0);

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

    robot->mainLoop();

    float height = robotControl.height;
    float tiltX = fmap(robotControl.roll, -10.0, 10.0, -28.0, 28.0);
    float tiltY = fmap(robotControl.pitch, -10.0, 10.0, -28.0, 28.0);
    float rotate = fmap(robotControl.yaw, -10.0, 10.0, -20.0, 20.0);

    robot->setPose(height, 120.0, tiltX, tiltY, rotate);

    double walkX = fmap(robotControl.joystickY, -100, 100, -50, 50);
    double walkY = fmap(robotControl.joystickX, -100, 100, 50, -50);
    robot->setWalkDirection(walkX, walkY);
    // robot->setWalkDirection(60, 0);
    robot->prepareTargetPositions();

    std::array<LegAngles, PentapodKinematics::NUM_LEGS> allAngles = robot->calculateAllLegAngles();

    for (uint8_t legIndex = 0; legIndex < PentapodKinematics::NUM_LEGS; legIndex++)
    {
        // printLegAngles(allAngles[legIndex], legIndex);
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

    // int allInReach = robot->allLegAnglesAreReachable(allAngles);

    // if (allInReach == -1)
    // {

    //     leds[1] = CRGB::Blue;

    //     Serial.println("leg angles good");

    //     for (uint8_t legIndex = 0; legIndex < NUMBER_OF_LEGS; legIndex++)
    //     {
    //         // if (legIndex == 1)
    //         // {
    //             // robot->printLegAngles(legIndex, angles[legIndex]);
    //         // }

    //         JointAngles calibration = extraCalibrations[legIndex];

    //         // lift
    //         float degree0 = angles[legIndex].theta0;
    //         setDegreeForLegAndServo(legIndex, 0, calibration.theta0 + degree0, speed, acc);

    //         // rotate
    //         float degree1 = angles[legIndex].theta1;
    //         setDegreeForLegAndServo(legIndex, 1, calibration.theta1 + degree1, speed, acc);

    //         // knie
    //         float degree2 = angles[legIndex].theta2;
    //         setDegreeForLegAndServo(legIndex, 2, calibration.theta2 - degree2, speed, acc);
    //     }

    //     finalizeServoPositions();
    // }
    // else
    // {
    //     leds[1] = CRGB::Red;

    //     JointAngles notInReach = angles[allInReach];
    //     Vector3 target = robot->targetPosition[allInReach];

    //     Serial.print("    Leg target ");
    //     Serial.print(allInReach);
    //     Serial.print(":  ");
    //     Serial.print(target.x);
    //     Serial.print(" ");
    //     Serial.print(target.y);
    //     Serial.print(" ");
    //     Serial.print(target.z);

    //     Serial.println(" -> leg angles not reachable!!! ");
    // }

    FastLED.show();

    delay(10);
}