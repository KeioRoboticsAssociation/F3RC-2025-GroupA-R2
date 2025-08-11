#include "Driver/ServoMotor.hpp"
#include <cmath>

Servo::Servo(PinName pwm_pin, chrono::microseconds min_pulse_width, chrono::microseconds max_pulse_width, chrono::milliseconds pwm_period)
    : pwm(pwm_pin), min_pulse_width(min_pulse_width), max_pulse_width(max_pulse_width)
{
    pwm.period_ms(pwm_period.count());
}

// @param pulse_width chrono::microseconds 目標のパルス幅[us]。
void Servo::setPulseWidth(chrono::microseconds pulse_width)
{
    assert(min_pulse_width <= pulse_width && pulse_width <= max_pulse_width);
    pwm.pulsewidth_us(pulse_width.count());
}

// @param degrees float 目標の角度[deg]
void Servo::setAngleDeg(float degrees)
{
    assert(-90.f <= degrees && degrees <= 90.f);

    chrono::microseconds pulse_range = max_pulse_width - min_pulse_width;
    int offset_pulse_width = std::round((degrees + 90.f) * (float)pulse_range.count() / 180.f);
    chrono::microseconds pulse_width = min_pulse_width + chrono::microseconds(offset_pulse_width);

    setPulseWidth(pulse_width);
}
