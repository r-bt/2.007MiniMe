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

// Encoders
#include "Encoders.h"

// wheel speeds
volatile int leftSpeed, rightSpeed;

const int NORMAL_SPEED = 230;
const int NORMAL_SPEED_ANGLE = 0;
const int maxMotorSpeed = 255;

int previousIntersectionMillis = 0;
const int intersectionMillisThreshold = 50;

PIDLineFollower pid_line_follower(NORMAL_SPEED, maxMotorSpeed, &leftSpeed, &rightSpeed);
PIDAngle pid_angle(&leftSpeed, &rightSpeed);
Encoders encoders;

PIDTimer pid_timer;

bool onIntersection = false;
int crossedIntersections = 0;
int desiredIntersections = 0;

bool fullStop = false;

void PIDLineTracker();
void runPIDLineFollowingTimer();
void set_desired_intersections(int intersections);
bool count_intersections();

enum STATE_TYPE
{
  LINE_FOLLOW,
  ANGLE,
  DISTANCE,
  NONE
};

struct State
{
  STATE_TYPE type;
  int value;
  int extra_value;
};

struct State states[] = {
    // {DISTANCE, 183, -45},
    // {DISTANCE, 183, -135}};
    {LINE_FOLLOW, 13},
    {ANGLE, -90},
    {LINE_FOLLOW, 13},
    {ANGLE, -180},
    {LINE_FOLLOW, 10},
    {ANGLE, -270},
    {LINE_FOLLOW, 13},
    {ANGLE, -180},
    {LINE_FOLLOW, 1},
    {NONE, 0}};

int current_state_index = 0;

void handle_angle_state(State current_state);
void handle_LINE_FOLLOW_state(State current_state);
void handle_distance_state(State current_state);

void setup()
{
  Serial.begin(115200);

  // pid_line_follower.init();
  pid_angle.init();
  encoders.init();

  pinMode(8, OUTPUT);

  pid_timer.init();
  pid_timer.attachPID(10L, runPIDLineFollowingTimer);
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
    if (!onIntersection && (millis() - previousIntersectionMillis > intersectionMillisThreshold))
    {
      digitalWrite(8, HIGH);
      onIntersection = true;
      crossedIntersections += 1;
      previousIntersectionMillis = millis();
      if (crossedIntersections == desiredIntersections)
      {
        crossedIntersections = 0;
        desiredIntersections = 0;
        return true;
      }
    }
    else
    {
      // digitalWrite(8, LOW);
    }
  }
  else
  {
    onIntersection = false;
  }
  return false;
}

void loop()
{
  struct State current_state = states[current_state_index];

  switch (current_state.type)
  {
  case NONE:
    return;
  case LINE_FOLLOW:
    handle_LINE_FOLLOW_state(current_state);
    break;
  case ANGLE:
    handle_angle_state(current_state);
    break;
  case DISTANCE:
    handle_distance_state(current_state);
    break;
  };

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);
}

void handle_LINE_FOLLOW_state(State current_state)
{

  if (desiredIntersections == 0)
  {
    set_desired_intersections(current_state.value);
    pid_line_follower.enable();
  }

  if (count_intersections())
  {
    current_state_index += 1;
    full_stop();
    pid_line_follower.disable();
    return;
  }
}

void handle_angle_state(State current_state)
{
  pid_angle.enable();
  pid_angle.set_setpoint(current_state.value);

  if (pid_angle.get_confidence())
  {
    full_stop();
    current_state_index += 1;
    pid_angle.disable();
    return;
  }

  pid_angle.compute();
}

bool init_distance = true;

void handle_distance_state(State current_state)
{
  if (init_distance)
  {
    pid_angle.enable();
    pid_angle.set_setpoint(current_state.extra_value);
    encoders.reset_counts();
    init_distance = false;
  }

  if (pid_angle.get_confidence())
  {
    int straight_speed = encoders.get_distance() > (current_state.value * 0.9) ? NORMAL_SPEED * 0.7 : NORMAL_SPEED;

    pid_angle.set_normal_speed(straight_speed);

    if (encoders.get_distance() > current_state.value)
    {
      full_stop();
      current_state_index += 1;
      pid_angle.disable();
      init_distance = true;
      return;
    }
  }

  pid_angle.compute();
}