#include <Arduino.h>

// IR Sensor Pins
#define IR1_PIN A11
#define IR2_PIN A12
#define IR3_PIN A13
#define IR4_PIN A14
#define IR5_PIN A15

// Minimum IR Values
#define IR1Min 815
#define IR2Min 786
#define IR3Min 796
#define IR4Min 663
#define IR5Min 812

// Maximum IR Values
#define IR1Max 984
#define IR2Max 985
#define IR3Max 984
#define IR4Max 983
#define IR5Max 984

// Define our PID Values

// Intersection code
#define NORMED_VALUES_BUFFER_SIZE 10

class PIDLineFollower
{
private:
    float SETPOINT = 3.0;
    const float DELTA_TIME = 12; // milliseconds
    float error = 0.0;
    float previousError = 0.0;
    float leftIntegral = 0.0;
    float rightIntegral = 0.0;
    float normedValues[5][NORMED_VALUES_BUFFER_SIZE];
    int currentIndex = 0;
    float intersectionThreshold = 0.95;

    // Define our PID values
    float KP_Left = 200.0;
    float KI_Left = 10.0;
    float KD_Left = 11.0;
    float KP_Right = 200.0;
    float KI_Right = 10.0;
    float KD_Right = 11.0;

    bool isIntersection(float latestNormedValues[5]);

    bool is_enabled = false;

public:
    int NORMAL_SPEED;
    int MAX_MOTOR_SPEED;
    volatile int *leftSpeed;
    volatile int *rightSpeed;
    bool onIntersection = false;

    PIDLineFollower(int normal_speed, int max_motor_speed, volatile int *left_speed, volatile int *right_speed)
    { // Constructor with parameters
        NORMAL_SPEED = normal_speed;
        MAX_MOTOR_SPEED = max_motor_speed;
        leftSpeed = left_speed;
        rightSpeed = right_speed;
    }

    void init();
    void PIDLineFollowing();

    void enable();
    void disable();
};

float computeNormVal(float sensorVal, float minVal, float maxVal);