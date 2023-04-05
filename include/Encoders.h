#include <Arduino.h>

#define ENC_COUNT_REV 1920.0
#define WHEEL_DIAMETER 6.45 // cm

class Encoders
{
private:
    // define encoder pins
    const byte leftChannelA = 3;
    const byte leftChannelB = 32;
    const byte rightChannelA = 2;
    const byte rightChannelB = 34;

    static Encoders *instance0_;

    static void leftEncoderISR();
    static void rightEncoderISR();

    void handleLeftEncoder();
    void handleRightEncoder();

public:
    // define encoder count variables
    volatile long leftEncoderCount = 0;
    volatile long rightEncoderCount = 0;

    // define channel A previous and current variables
    volatile byte prevLeftAVal = 0;
    volatile byte currentLeftAVal = 0;
    volatile byte prevRightAVal = 0;
    volatile byte currentRightAVal = 0;

    void init();

    float get_distance();
    void reset_counts();
};