#include "Mechanism/Arms/PseudoServo.hpp"

PseudoServo::PseudoServo(DCMotor &m, Encoder &e, LimitSwitch &sw, PIDController<float> pid_controller, float min_a, float max_a)
    : _m(m), _e(e), _sw(sw), pid_controller(pid_controller), _min_a(min_a), _max_a(max_a), _is_homed(false), _target_angle_deg(0.0f) {}

void PseudoServo::init()
{
    _m.stop();
    _e.reset();
}

bool PseudoServo::home()
{
    _m.setDuty(-0.5f);
    while (!_sw.isPressed())
    {
    }
    _m.stop();
    _e.reset();
    _is_homed = true;
    _target_angle_deg = 0.0f;
    return true;
}

void PseudoServo::setAngleDeg(float degrees)
{
    if (!_is_homed)
    {
        return;
    }
    if (degrees < _min_a)
    {
        _target_angle_deg = _min_a;
    }
    else if (degrees > _max_a)
    {
        _target_angle_deg = _max_a;
    }
    else
    {
        _target_angle_deg = degrees;
    }
}

float PseudoServo::getAngleDeg()
{
    return _e.getDegrees();
}

bool PseudoServo::isHomed()
{
    return _is_homed;
}

void PseudoServo::update()
{
    if (!_is_homed)
    {
        return;
    }
    float current_angle = _e.getDegrees();
    float duty = pid_controller.calculate(_target_angle_deg - current_angle);

    if (duty > 0.1)
    {
        printf("duty: %f\n", 0.1);
        _m.setDuty(0.1);
    }
    else if (duty < -0.1)
    {
        printf("duty: %f\n", -0.1);
        _m.setDuty(-0.1);
    }
    else
    {
        printf("duty: %f\n", duty);
        _m.setDuty(duty);
    }
}
