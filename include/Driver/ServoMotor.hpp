#pragma once
#include <mbed.h>

class Servo
{
public:
    PwmOut pwm;
    chrono::microseconds min_pulse_width;
    chrono::microseconds max_pulse_width;

    Servo(PinName pwm_pin, chrono::microseconds min_pulse_width, chrono::microseconds max_pulse_width, chrono::milliseconds pwm_period = 20ms);
    void setPulseWidth(chrono::microseconds pulse_width);
    void setAngleDeg(float degrees);
};
