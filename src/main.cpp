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
int desiredIntersections = 0;

bool fullStop = false;

void PIDLineTracker();
void runPIDLineFollowingTimer();
void runPIDAngle();
void set_desired_intersections(int intersections);
bool count_intersections();

enum STATE
{
  STRAIGHT_1,
  TURN_90,
  STRAIGHT_2,
  TURN_90_2,
  STRAIGHT_3,
  TURN_90_3,
  STRAIGHT_4,
};

STATE current_state = STRAIGHT_1;

void setup()
{
  Serial.begin(115200);

  pid_line_follower.init();
  pid_angle.init();

  pinMode(8, OUTPUT);

  pid_timer.init();
  pid_timer.attachPID(1L, runPIDLineFollowingTimer);
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

void set_desired_intersections(int intersections)
{
  crossedIntersections = 0;
  desiredIntersections = intersections;
}

void full_stop()
{
  leftSpeed = 0;
  rightSpeed = 0;
}

bool count_intersections()
{
  if (pid_line_follower.onIntersection)
  {
    if (!onIntersection)
    {
      onIntersection = true;
      crossedIntersections += 1;
      if (crossedIntersections == desiredIntersections)
      {
        crossedIntersections = 0;
        desiredIntersections = 0;
        return true;
      }
    }
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
  return false;
}

void loop()
{
  if (current_state == STRAIGHT_1)
  {
    if (desiredIntersections == 0)
    {
      set_desired_intersections(13);
      pid_line_follower.enable();
    }
    if (count_intersections())
    {
      current_state = TURN_90;
      pid_line_follower.disable();
      full_stop();
    }
  }
  else if (current_state == TURN_90)
  {
    pid_angle.set_angle(90);
    pid_angle.enable();

    if (pid_angle.get_confidence())
    {
      pid_angle.disable();
      full_stop();
      current_state = STRAIGHT_2;
      digitalWrite(8, HIGH);
    }
  }
  else if (current_state == STRAIGHT_2)
  {
    digitalWrite(8, LOW);
    if (desiredIntersections == 0)
    {
      set_desired_intersections(13);
      pid_line_follower.enable();
    }
    if (count_intersections())
    {
      current_state = TURN_90_2;
      pid_line_follower.disable();
      full_stop();
    }
  }

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);
  // if (pid_line_follower.onIntersection)
  // {
  //   if (!onIntersection)
  //   {
  //     onIntersection = true;
  //     crossedIntersections += 1;
  //     if (crossedIntersections == 13)
  //     {
  //       fullStop = true;
  //     }
  //     digitalWrite(8, HIGH);
  //   }
  // }
  // else
  // {
  //   if (onIntersection)
  //   {
  //     Serial.println("Off intersection!");
  //     digitalWrite(8, LOW);
  //   }
  //   onIntersection = false;
  // }

  // if (!fullStop)
  // {
  //
  // }
  // else
  // {
  //   motors.setM1Speed(0);
  //   motors.setM2Speed(0);
  // }
}
