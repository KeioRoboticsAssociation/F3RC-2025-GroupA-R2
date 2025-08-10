#include "Driver/DCMotor.hpp"

DCMotor::DCMotor(PinName pwm_pin, PinName dir_pin, bool reverse_dir, chrono::milliseconds pwm_period) : pwm(pwm_pin), dir(dir_pin)
{
    pwm.period_ms(pwm_period.count());
    this->reverse_dir = reverse_dir;
    stop();
}

void DCMotor::setDuty(float duty)
{
    assert(0.f <= duty && duty <= 1.f);

    last_duty = duty;
    bool go_forward = duty >= 0;

    if (reverse_dir)
    {
        // 逆回転の場合は進行方向を反転
        go_forward = !go_forward;
    }

    pwm.write(fabs(duty));         // デューティ比を設定
    dir.write(go_forward ? 0 : 1); // 回転方向を設定
}

void DCMotor::stop()
{
    setDuty(0.f);
}

float DCMotor::getDuty()
{
    return last_duty;
}
