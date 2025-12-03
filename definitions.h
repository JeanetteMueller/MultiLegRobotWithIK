
bool debug = false;

SMS_STS st;

#define NUMBER_OF_LEGS 5
#define SERVO_NUM (NUMBER_OF_LEGS * 3) // Number of servos
u8 servoIds[SERVO_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
s16 newPosition[SERVO_NUM];
u16 newSpeed[SERVO_NUM];
u8 newAcc[SERVO_NUM];

bool calibrated = false;

CircularRobot *robot;
