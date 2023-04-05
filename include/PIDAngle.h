#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ANGLE_ERRORS_COUNT 40

class PIDAngle
{
private:
    // PID variables for angle tracking
    float ANGLE = 0;
    volatile float previousError = 0.0;
    volatile float leftIntegral = 0.0;
    volatile float rightIntegral = 0.0;
    const float KP_left = 3.5;
    const float KI_left = 0.05;
    const float KD_left = 0.03;
    const float KP_right = 3.5;
    const float KI_right = 0.05;
    const float KD_right = 0.03;
    bool is_enabled = false;

    const float angle_error_confidence_threshold = 10.0;

    float errors[ANGLE_ERRORS_COUNT];
    int error_index = 0;

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

    void enable(float angle);
    void disable();

    void set_normal_speed(float speed);

    bool get_confidence();
};