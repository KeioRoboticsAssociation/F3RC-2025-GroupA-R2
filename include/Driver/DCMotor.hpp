#pragma once
#include <mbed.h>

class DCMotor
{
public:
    PwmOut pwm;
    DigitalOut dir;

    DCMotor(PinName pwm_pin, PinName dir_pin, bool reverse_dir = false, chrono::milliseconds pwm_period = 20ms);
    void setDuty(float duty);
    void stop();
    float getDuty();

private:
    bool reverse_dir;
    float last_duty;
};
