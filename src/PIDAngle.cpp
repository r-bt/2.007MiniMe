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

    // Setup Intersection Array
    for (int i = 0; i < ANGLE_ERRORS_COUNT; i++)
    {
        errors[i] = 180;
    }
}

void PIDAngle::runPIDAngle()
{

    if (!is_enabled)
    {
        return;
    }

    // Allow wire communication inside call
    sei();

    digitalWrite(8, HIGH);

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

    // Track the error
    if (error_index == ANGLE_ERRORS_COUNT)
    {
        error_index = 0;
    }
    errors[error_index] = error;
    error_index += 1;
}

void PIDAngle::enable()
{
    is_enabled = true;
}

void PIDAngle::disable()
{
    is_enabled = false;
    previousError = 0.0;
    leftIntegral = 0.0;
    rightIntegral = 0.0;
}

void PIDAngle::set_angle(float angle)
{
    ANGLE = angle;
    // Setup Intersection Array
    for (int i = 0; i < ANGLE_ERRORS_COUNT; i++)
    {
        errors[i] = 180;
    }
}

bool PIDAngle::get_confidence()
{
    float sum = 0;
    for (int k = 0; k < ANGLE_ERRORS_COUNT; k++)
    {
        sum += errors[k];
    }

    return (sum / ANGLE_ERRORS_COUNT) < angle_error_confidence_threshold;
}