#pragma once
#include <mbed.h>
#include "Parts/DCMotor.hpp"
#include "Parts/Encoder.hpp"
#include "Parts/LimitSwitch.hpp"
#include "Mechanism/PIDController.hpp"

class PseudoServo
{
public:
    PseudoServo(DCMotor &m, Encoder &e, LimitSwitch &sw, PIDController<float> pid_controller, float min_a, float max_a);
    void init();
    bool home();
    void setAngleDeg(float degrees);
    float getAngleDeg();
    bool isHomed();
    void update();

    float _target_angle_deg;

private:
    DCMotor &_m;
    Encoder &_e;
    LimitSwitch &_sw;
    PIDController<float> pid_controller;
    float _min_a;
    float _max_a;
    bool _is_homed;
    static constexpr float KP = 0.005f;
};
