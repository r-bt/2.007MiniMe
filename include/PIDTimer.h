#include <TimerInterrupt.hpp> //https://github.com/khoih-prog/TimerInterrupt
#include <ISR_Timer.hpp>      //https://github.com/khoih-prog/TimerInterrupt

class PIDTimer
{
private:
    static void TimerHandler2();

public:
    inline static ISR_Timer ISR_Timer2;
    void init();
    void attachPID(unsigned long d, timer_callback f);
};