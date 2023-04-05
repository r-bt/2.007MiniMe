#include "PIDAngle.h"

void PIDAngle::init()
{
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    bno.setExtCrystalUse(true);
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    yawAngle = event.orientation.x; // deg
    remappedYawAngle = remapAngle(yawAngle);

    // SETUP PID
    rightPID.SetSampleTime(25); // Set PID sampling frequency is 100ms
    rightPID.SetOutputLimits(-255, 255);

    leftPID.SetSampleTime(25); // Set PID sampling frequency is 100ms
    leftPID.SetOutputLimits(-255, 255);

    if (!calibrated)
    {
        get_is_calibrated();
        return;
    }
}

double PIDAngle::remapAngle(double angle)
{
    double xyCoordinate[2];

    angle = angle * DEG_TO_RAD;
    xyCoordinate[0] = cos(angle);
    xyCoordinate[1] = -sin(angle);

    return RAD_TO_DEG * (atan2(xyCoordinate[1], xyCoordinate[0]));
}

void PIDAngle::compute()
{
    if (!calibrated)
    {
        get_is_calibrated();
        return;
    }

    sensors_event_t event;
    bno.getEvent(&event);

    // get the yaw angle
    yawAngle = event.orientation.x; // deg
    remappedYawAngle = remapAngle(yawAngle);

    rightPID.Compute();
    leftPID.Compute();

    *leftSpeed = NORMAL_SPEED - leftDeltaSpeed;
    *rightSpeed = NORMAL_SPEED + rightDeltaSpeed;
}

void PIDAngle::set_setpoint(double angle)
{
    SETPOINT = angle;
    setpointLowerLimit = SETPOINT - angleTolerance;
    setpointUpperLimit = SETPOINT + angleTolerance;
}

bool PIDAngle::get_confidence()
{
    return ((remappedYawAngle >= setpointLowerLimit) && (remappedYawAngle <= setpointUpperLimit));
}

void PIDAngle::reset_normal_speed()
{
    NORMAL_SPEED = 0;
}

void PIDAngle::set_normal_speed(double speed)
{
    NORMAL_SPEED = speed;
}

void PIDAngle::get_is_calibrated()
{
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    calibrated = gyro == 3;
}