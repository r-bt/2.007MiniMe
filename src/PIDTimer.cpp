/*
Author: Richard Beattie
Description: Timer Utility to run PID controller at 1ms (maximum is about 12ms with delay())
Date: 13/03/2023
*/

#include "Timers.h"
#include "PIDTimer.h"

#define TIMER_INTERRUPT_DEBUG 0
#define TIMER2_INTERVAL_MS 1L

// TimerInterrupt ITimer2;

void PIDTimer::TimerHandler2()
{
    ISR_Timer2.run();
}

void PIDTimer::init()
{
    // Timer stuff
    ITimer2.init();
    if (ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, TimerHandler2))
        Serial.println("Starting  ITimer2 OK, millis() = " + String(millis()));
    else
        Serial.println("Can't set ITimer2. Select another freq., duration or timer");
}

void PIDTimer::attachPID(unsigned long d, timer_callback f)
{
    ISR_Timer2.setInterval(d, f);
}