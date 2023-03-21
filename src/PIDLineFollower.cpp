#include "PIDLineFollower.h"

// compute the normalized sensor value
float computeNormVal(float sensorVal, float minVal, float maxVal)
{
    // constrain the normal to lie between 0 and 1
    return constrain((sensorVal - minVal) / (maxVal - minVal), 0, 1);
}

void PIDLineFollower::init()
{
    // Setup Sensors
    pinMode(IR1_PIN, INPUT);
    pinMode(IR2_PIN, INPUT);
    pinMode(IR3_PIN, INPUT);
    pinMode(IR4_PIN, INPUT);
    pinMode(IR5_PIN, INPUT);

    // Setup Intersection Array
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < NORMED_VALUES_BUFFER_SIZE; j++)
        {
            normedValues[i][j] = 1;
        }
    }
}

void PIDLineFollower::PIDLineFollowing()
{
    if (!is_enabled)
    {
        return;
    }

    // read the line tracking sensor channels
    int IR1Val = analogRead(IR1_PIN);
    int IR2Val = analogRead(IR2_PIN);
    int IR3Val = analogRead(IR3_PIN);
    int IR4Val = analogRead(IR4_PIN);
    int IR5Val = analogRead(IR5_PIN);

    // compute the normalized channel values
    float IR1Norm = computeNormVal(IR1Val, IR1Min, IR1Max);
    float IR2Norm = computeNormVal(IR2Val, IR2Min, IR2Max);
    float IR3Norm = computeNormVal(IR3Val, IR3Min, IR3Max);
    float IR4Norm = computeNormVal(IR4Val, IR4Min, IR4Max);
    float IR5Norm = computeNormVal(IR5Val, IR5Min, IR5Max);

    // determine which channel lies over the line
    // note: to prevent NAN, you can change to (1 - IR1Norm) to (1.001 - IR1Norm), etc.
    float numerator = ((1 - IR1Norm) * 1 + (1 - IR2Norm) * 2 + (1 - IR3Norm) * 3 + (1 - IR4Norm) * 4 + (1 - IR5Norm) * 5);
    float denominator = (1 - IR1Norm) + (1 - IR2Norm) + (1 - IR3Norm) + (1 - IR4Norm) + (1 - IR5Norm);
    float actualValue = numerator / denominator;

    if (isnan(actualValue))
    {
        actualValue = 3.0;
    }

    // compute the error
    error = SETPOINT - actualValue;

    // convert time from milliseconds to seconds
    float delta_time = 0.001;

    // PID computation for left wheel
    leftIntegral = leftIntegral + error * delta_time;
    float leftDerivative = (error - previousError) / delta_time;

    float leftDeltaSpeed = int(KP_Left * error + KI_Left * leftIntegral + KD_Left * leftDerivative);
    *leftSpeed = NORMAL_SPEED - leftDeltaSpeed;

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
    float rightDeltaSpeed = int(KP_Right * error + KI_Right * rightIntegral + KD_Right * rightDerivative);
    *rightSpeed = NORMAL_SPEED + rightDeltaSpeed;

    // contrain the wheel speed to lie between -maxMotorSpeed and maxMotorSpeed
    if (*rightSpeed > MAX_MOTOR_SPEED)
    {
        *rightSpeed = MAX_MOTOR_SPEED;
    }
    else if (*rightSpeed < -MAX_MOTOR_SPEED)
    {
        *rightSpeed = -MAX_MOTOR_SPEED;
    }

    float latestNormedValues[] = {IR1Norm, IR2Norm, IR3Norm, IR4Norm, IR5Norm};
    onIntersection = isIntersection(latestNormedValues);

    previousError = error;
}

bool PIDLineFollower::isIntersection(float latestNormedValues[5])
{
    if (currentIndex == NORMED_VALUES_BUFFER_SIZE)
    {
        currentIndex = 0;
    }

    int indexes[] = {0, 4};
    for (int k = 0; k < 2; k++)
    {
        int i = indexes[k];
        normedValues[i][currentIndex] = latestNormedValues[i];
        float sum = 0.0;
        for (int j = 0; j < NORMED_VALUES_BUFFER_SIZE; j++)
        {
            sum += normedValues[i][j];
        }
        float average = sum / NORMED_VALUES_BUFFER_SIZE;
        if (average > intersectionThreshold)
        {
            currentIndex += 1;
            return false;
        }
    }

    currentIndex += 1;
    return true;
}

void PIDLineFollower::enable()
{
    is_enabled = true;
}

void PIDLineFollower::disable()
{
    is_enabled = false;
    previousError = 0.0;
    leftIntegral = 0.0;
    rightIntegral = 0.0;
}