#include "Arduino.h"
#include "PID_v1.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define PID_ANGLE_KP_Left 80.0
#define PID_ANGLE_KI_Left 50.0
#define PID_ANGLE_KD_Left 20.0
#define PID_ANGLE_KP_Right 80.0
#define PID_ANGLE_KI_Right 50.0
#define PID_ANGLE_KD_Right 20.0

class PIDAngle
{
private:
    Adafruit_BNO055 bno = Adafruit_BNO055(55);

    double remappedYawAngle;
    double yawAngle;
    double SETPOINT = -90;
    double angleTolerance = 1;
    double setpointLowerLimit = SETPOINT - angleTolerance;
    double setpointUpperLimit = SETPOINT + angleTolerance;

    double leftDeltaSpeed = 0;
    double rightDeltaSpeed = 0;

    double NORMAL_SPEED = 0;

    PID rightPID;
    PID leftPID;

    bool calibrated = false;
    void get_is_calibrated();

public:
    volatile int *leftSpeed;
    volatile int *rightSpeed;

    PIDAngle(volatile int *left_speed, volatile int *right_speed) : rightPID(&remappedYawAngle, &rightDeltaSpeed, &SETPOINT, PID_ANGLE_KP_Right, PID_ANGLE_KI_Right, PID_ANGLE_KD_Right, DIRECT),
                                                                    leftPID(&remappedYawAngle, &leftDeltaSpeed, &SETPOINT, PID_ANGLE_KP_Left, PID_ANGLE_KI_Left, PID_ANGLE_KD_Left, DIRECT)
    {
        leftSpeed = left_speed;
        rightSpeed = right_speed;
    }

    void init();
    double remapAngle(double angle);
    void compute();

    bool get_confidence();

    void set_setpoint(double angle);

    void set_normal_speed(double speed);
    void reset_normal_speed();

    void enable()
    {
        rightPID.SetMode(AUTOMATIC);
        leftPID.SetMode(AUTOMATIC);
    }

    void disable()
    {
        rightPID.SetMode(MANUAL);
        leftPID.SetMode(MANUAL);
    }
};
