#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class PIDAngle
{
private:
    // PID variables for angle tracking
    float ANGLE = 90;
    volatile float previousError = 0.0;
    volatile float leftIntegral = 0.0;
    volatile float rightIntegral = 0.0;
    const float KP_left = 7.0;
    const float KI_left = 0.05;
    const float KD_left = 0;
    const float KP_right = 7.0;
    const float KI_right = 0.05;
    const float KD_right = 0;

    Adafruit_BNO055 bno = Adafruit_BNO055(55);

public:
    double constrainAngle(double x);
    void runPIDAngle();

    int NORMAL_SPEED;
    int MAX_MOTOR_SPEED;
    volatile int *leftSpeed;
    volatile int *rightSpeed;

    PIDAngle(int normal_speed, int max_motor_speed, volatile int *left_speed, volatile int *right_speed)
    { // Constructor with parameters
        NORMAL_SPEED = normal_speed;
        MAX_MOTOR_SPEED = max_motor_speed;
        leftSpeed = left_speed;
        rightSpeed = right_speed;
    }

    void init();
};