#include <Arduino.h>
// #include <Servo.h> // use the servo library
#include "DFRobotMotorShield.h"
DFRobotMotorShield motors;

// SETUP PID Timer
#include "PIDTimer.h"
PIDTimer pid_timer;

/*
2.S007 Mini Me Physical Homework Three

Objective: Follow a line on the gameboard

Based on 16.632 Line Following Code
*/

// distance sensor servo
#define DISTANCE_SERVO_PIN 46 // distance sensor servo pin
// Servo myservo;
int servoPosition = 90;

// line tracker variables and constants
#define IR1_PIN A11
#define IR2_PIN A12
#define IR3_PIN A13
#define IR4_PIN A14
#define IR5_PIN A15
volatile int IR1Val, IR2Val, IR3Val, IR4Val, IR5Val;
int IR1Min, IR2Min, IR3Min, IR4Min, IR5Min;
int IR1Max, IR2Max, IR3Max, IR4Max, IR5Max;
bool IR1Bool, IR2Bool, IR3Bool, IR4Bool, IR5Bool;
volatile float IR1Norm, IR2Norm, IR3Norm, IR4Norm, IR5Norm;
volatile float numerator, denominator;
float SETPOINT = 3.0;            // sensor set point
volatile float weightedLocation; // sensor actual location

#define NORMED_VALUES_BUFFER_SIZE 10
float normedValues[5][NORMED_VALUES_BUFFER_SIZE];
int currentIndex = 0;

// wheel speeds
volatile int leftSpeed, rightSpeed;
volatile float leftDeltaSpeed, rightDeltaSpeed;
const int NORMAL_SPEED = 255;
const int maxMotorSpeed = 255;

// PID constants
const float KP_Left = 200.0;
const float KI_Left = 10.0;
const float KD_Left = 11.0;
const float KP_Right = 200.0;
const float KI_Right = 10.0;
const float KD_Right = 11.0;
const float DELTA_TIME = 12; // milliseconds

// PID variables
volatile float error = 0.0;
volatile float previousError = 0.0;
volatile float leftIntegral = 0.0;
volatile float leftDerivative = 0.0;
volatile float rightIntegral = 0.0;
volatile float rightDerivative = 0.0;

// define time variables/conversions
long currentMillis = 0;
long previousMillis = 0;
#define MILLISEC_TO_SEC 1 / 1000

// define intersection variables
float intersectionThreshold = 0.95;
bool onIntersection = false;
int crossedIntersections = 0;

bool fullStop = false;

float computeNormVal(float sensorVal, float minVal, float maxVal);
bool isIntersection(float latestNormedValues[5]);
void PIDLineTracker();

void setup()
{
  Serial.begin(115200);

  // myservo.attach(DISTANCE_SERVO_PIN); // attach the servo to the servo pin
  // myservo.write(servoPosition);

  pinMode(IR1_PIN, INPUT_PULLUP);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);
  pinMode(IR5_PIN, INPUT);

  pinMode(8, OUTPUT);

  // minimum and maximum sensor channel readings
  IR1Min = 815;
  IR2Min = 786;
  IR3Min = 796;
  IR4Min = 663;
  IR5Min = 812;

  IR1Max = 984;
  IR2Max = 985;
  IR3Max = 984;
  IR4Max = 983;
  IR5Max = 984;

  // Setup our intersection array
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < NORMED_VALUES_BUFFER_SIZE; j++)
    {
      normedValues[i][j] = 1;
    }
  }

  // ISR_Timer2.setInterval(1L, PIDLineTracker);
  pid_timer.init();
  pid_timer.attachPID(1L, PIDLineTracker);
}

void loop()
{

  // currentMillis = millis();

  float latestNormedValues[] = {IR1Norm, IR2Norm, IR3Norm, IR4Norm, IR5Norm};
  if (isIntersection(latestNormedValues))
  {
    if (!onIntersection)
    {
      onIntersection = true;
      crossedIntersections += 1;
      if (crossedIntersections == 13)
      {
        fullStop = true;
      }
      // digitalWrite(8, HIGH);
    }

    numerator = ((1 - IR2Norm) * 2 + (1 - IR3Norm) * 3 + (1 - IR4Norm) * 4);
    denominator = (1 - IR2Norm) + (1 - IR3Norm) + (1 - IR4Norm);
    weightedLocation = numerator / denominator;
  }
  else
  {
    if (onIntersection)
    {
      Serial.println("Off intersection!");
      // digitalWrite(8, LOW);
    }
    onIntersection = false;
  }

  if (!fullStop)
  {
    motors.setM1Speed(rightSpeed);
    motors.setM2Speed(leftSpeed);
  }
  else
  {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
  }
}

void PIDLineTracker()
{
  digitalWrite(8, HIGH);
  // read the line tracking sensor channels
  IR1Val = analogRead(IR1_PIN);
  IR2Val = analogRead(IR2_PIN);
  IR3Val = analogRead(IR3_PIN);
  IR4Val = analogRead(IR4_PIN);
  IR5Val = analogRead(IR5_PIN);

  // compute the normalized channel values
  IR1Norm = computeNormVal(IR1Val, IR1Min, IR1Max);
  IR2Norm = computeNormVal(IR2Val, IR2Min, IR2Max);
  IR3Norm = computeNormVal(IR3Val, IR3Min, IR3Max);
  IR4Norm = computeNormVal(IR4Val, IR4Min, IR4Max);
  IR5Norm = computeNormVal(IR5Val, IR5Min, IR5Max);

  // determine which channel lies over the line
  // note: to prevent NAN, you can change to (1 - IR1Norm) to (1.001 - IR1Norm), etc.
  numerator = ((1 - IR1Norm) * 1 + (1 - IR2Norm) * 2 + (1 - IR3Norm) * 3 + (1 - IR4Norm) * 4 + (1 - IR5Norm) * 5);
  denominator = (1 - IR1Norm) + (1 - IR2Norm) + (1 - IR3Norm) + (1 - IR4Norm) + (1 - IR5Norm);
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
  leftDerivative = (error - previousError) / delta_time;
  leftDeltaSpeed = int(KP_Left * error + KI_Left * leftIntegral + KD_Left * leftDerivative);
  leftSpeed = NORMAL_SPEED - leftDeltaSpeed;

  // contrain the wheel speed to lie between -maxMotorSpeed and maxMotorSpeed
  if (leftSpeed > maxMotorSpeed)
  {
    leftSpeed = maxMotorSpeed;
  }
  else if (leftSpeed < -maxMotorSpeed)
  {
    leftSpeed = -maxMotorSpeed;
  }

  // PID computation for right wheel
  rightIntegral = rightIntegral + error * delta_time;
  rightDerivative = (error - previousError) / delta_time;
  rightDeltaSpeed = int(KP_Right * error + KI_Right * rightIntegral + KD_Right * rightDerivative);
  rightSpeed = NORMAL_SPEED + rightDeltaSpeed;

  // contrain the wheel speed to lie between -maxMotorSpeed and maxMotorSpeed
  if (rightSpeed > maxMotorSpeed)
  {
    rightSpeed = maxMotorSpeed;
  }
  else if (rightSpeed < -maxMotorSpeed)
  {
    rightSpeed = -maxMotorSpeed;
  }
  previousError = error;
}

bool isIntersection(float latestNormedValues[5])
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
      Serial.println(average);
      currentIndex += 1;
      return false;
    }
  }

  currentIndex += 1;
  return true;
}

// compute the normalized sensor value
float computeNormVal(float sensorVal, float minVal, float maxVal)
{
  // constrain the normal to lie between 0 and 1
  return constrain((sensorVal - minVal) / (maxVal - minVal), 0, 1);
}