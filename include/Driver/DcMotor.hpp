#pragma once
#include <mbed.h>

class DcMotor
{
public:
    DcMotor(PinName pwm_pin, PinName dir_pin, bool direction = 0, float pwm_freq = 16000 /* Hz */);
    void setDuty(float duty);
    float getDuty() const;
    void stop();

    PwmOut pwm;
    DigitalOut dir;

private:
    bool direction;
    float last_duty;
};
