#include "PIDAngle.h"

double PIDAngle::constrainAngle(double x)
{
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

void PIDAngle::init()
{
    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
}

void PIDAngle::runPIDAngle()
{

    // Allow wire communication inside call
    sei();

    uint8_t system, gyro, accel, mag;
    gyro = 0;

    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    if (gyro == 0)
    {
        return;
    }

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    // // get the yaw angle
    float yawAngle = event.orientation.x;

    // Serial.print("ANGLE: ");
    // Serial.print(yawAngle);
    // Serial.print(",error=");

    // compute the error
    float error = constrainAngle(int(ANGLE) - int(yawAngle));

    // Serial.println(error);

    // convert time from milliseconds to seconds
    float delta_time = 0.01;

    // PID computation for left wheel
    leftIntegral = leftIntegral + error * delta_time;
    float angle_leftDerivative = (error - previousError) / delta_time;
    float leftDeltaSpeed = int(KP_left * error + KI_left * leftIntegral + KD_left * angle_leftDerivative);
    *leftSpeed = NORMAL_SPEED + leftDeltaSpeed;

    // contrain the wheel speed to lie between -maxMotorSpeed and maxMotorSpeed
    if (*leftSpeed > MAX_MOTOR_SPEED)
    {
        *leftSpeed = MAX_MOTOR_SPEED;
    }
    else if (*leftSpeed < -MAX_MOTOR_SPEED)
    {
        *leftSpeed = -MAX_MOTOR_SPEED;
    }

    // PID computation for right wheel
    rightIntegral = rightIntegral + error * delta_time;
    float rightDerivative = (error - previousError) / delta_time;
    float rightDeltaSpeed = int(KP_right * error + KI_right * rightIntegral + KD_right * rightDerivative);
    *rightSpeed = NORMAL_SPEED - rightDeltaSpeed;

    // contrain the wheel speed to lie between -maxMotorSpeed and maxMotorSpeed
    if (*rightSpeed > MAX_MOTOR_SPEED)
    {
        *rightSpeed = MAX_MOTOR_SPEED;
    }
    else if (*rightSpeed < -MAX_MOTOR_SPEED)
    {
        *rightSpeed = -MAX_MOTOR_SPEED;
    }
    previousError = error;
}
