/*
2.S007 Mini Me Physical Homework Three

Objective: Follow a line on the gameboard

Based on 16.632 Line Following Code
*/

#include <Arduino.h>
#include "DFRobotMotorShield.h"
DFRobotMotorShield motors;

// SETUP PID Timer
#include "PIDTimer.h"

// SETUP PID Line Following
#include "PIDLineFollower.h"

// SETUP PID Angle Following
#include "PIDAngle.h"

// wheel speeds
volatile int leftSpeed, rightSpeed;

const int NORMAL_SPEED = 255;
const int NORMAL_SPEED_ANGLE = 0;
const int maxMotorSpeed = 255;

PIDLineFollower pid_line_follower(NORMAL_SPEED, maxMotorSpeed, &leftSpeed, &rightSpeed);
PIDAngle pid_angle(NORMAL_SPEED_ANGLE, maxMotorSpeed, &leftSpeed, &rightSpeed);

PIDTimer pid_timer;

bool onIntersection = false;
int crossedIntersections = 0;
bool fullStop = false;

void PIDLineTracker();
void runPIDLineFollowingTimer();
void runPIDAngle();

void setup()
{
  Serial.begin(115200);

  pid_line_follower.init();
  pid_angle.init();

  pinMode(8, OUTPUT);

  pid_timer.init();
  // pid_timer.attachPID(1L, runPIDLineFollowingTimer);
  pid_timer.attachPID(10L, runPIDAngle);
}

void runPIDAngle()
{
  pid_angle.runPIDAngle();
}

void runPIDLineFollowingTimer()
{
  pid_line_follower.PIDLineFollowing();
}

void loop()
{

  if (pid_line_follower.onIntersection)
  {
    if (!onIntersection)
    {
      onIntersection = true;
      crossedIntersections += 1;
      if (crossedIntersections == 13)
      {
        fullStop = true;
      }
      digitalWrite(8, HIGH);
    }
  }
  else
  {
    if (onIntersection)
    {
      Serial.println("Off intersection!");
      digitalWrite(8, LOW);
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
