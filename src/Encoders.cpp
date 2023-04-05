#include "Encoders.h"

void Encoders::init()
{
    // Attach to instance
    instance0_ = this;

    // setup encoder pins
    pinMode(leftChannelA, INPUT);
    pinMode(leftChannelB, INPUT);
    pinMode(rightChannelA, INPUT);
    pinMode(rightChannelB, INPUT);

    // read initial states of encoder A channels
    prevLeftAVal = digitalRead(leftChannelA);
    prevRightAVal = digitalRead(rightChannelA); // attach channel A pins to ISRs

    attachInterrupt(digitalPinToInterrupt(leftChannelA), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightChannelA), rightEncoderISR, CHANGE);
}

void Encoders::leftEncoderISR()
{
    instance0_->handleLeftEncoder();
}

void Encoders::handleLeftEncoder()
{
    currentLeftAVal = digitalRead(leftChannelA);

    if (currentLeftAVal != prevLeftAVal)
    {

        if (currentLeftAVal != digitalRead(leftChannelB))
        {
            // swap these for counting in opposite direction
            // leftEncoderCount++;
            leftEncoderCount--;
        }
        else
        {
            // swap these for counting in opposite direction
            // leftEncoderCount--;
            leftEncoderCount++;
        }
    }

    prevLeftAVal = currentLeftAVal;
}

void Encoders::rightEncoderISR()
{
    instance0_->handleRightEncoder();
}

void Encoders::handleRightEncoder()
{
    currentRightAVal = digitalRead(rightChannelA);

    if (currentRightAVal != prevRightAVal)
    {

        if (currentRightAVal != digitalRead(rightChannelB))
        {
            // swap these for counting in opposite direction
            // rightEncoderCount--;
            rightEncoderCount++;
        }
        else
        {
            // swap these for counting in opposite direction
            // rightEncoderCount++;
            rightEncoderCount--;
        }
    }

    prevRightAVal = currentRightAVal;
}

void Encoders::reset_counts()
{
    leftEncoderCount = 0;
    rightEncoderCount = 0;
}

void Encoders::get_distance()
{
    float leftDistance = (leftEncoderCount / ENC_COUNT_REV) * PI * WHEEL_DIAMETER;
    float rightDistance = (rightEncoderCount / ENC_COUNT_REV) * PI * WHEEL_DIAMETER;

    return (leftDistance + rightDistance) / 2;
}