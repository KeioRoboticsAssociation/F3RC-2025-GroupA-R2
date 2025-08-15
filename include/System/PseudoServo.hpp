#pragma once
#include <mbed.h>
#include "Driver/DCMotor.hpp"
#include "Driver/Encoder.hpp"
#include "Driver/LimitSwitch.hpp"


class PseudoServo {
public:
    PseudoServo(DCMotor& m, Encoder& e, LimitSwitch& sw, float min_a, float max_a);
    void init();
    bool home();
    void setAngleDeg(float degrees);
    float getAngleDeg();
    bool isHomed();
    void update();



private:
    DCMotor& _m;
    Encoder& _e;
    LimitSwitch& _sw;
    float _min_a;
    float _max_a;
    bool _is_homed;
    float _target_angle_deg;
    static constexpr float KP = 0.05f;
};


