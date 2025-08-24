#pragma once
#include "Driver/DcMotor.hpp"
#include "Driver/Encoder.hpp"
#include "PIDController.hpp"

// PID制御を用いてDCモーターの出力を制御するクラス
class MotorController
{
public:
    DcMotor &motor;
    Encoder &encoder;
    PIDController<float> pid_controller;

    MotorController(DcMotor &motor, Encoder &encoder, PIDGain pid_gain = PIDGain{0.13, 0.0, 0.0, 20}, float max_duty = 1.0);
    void setTargetSpeed(float target_rps);
    void stop();
    float getSpeed();
    float getTargetSpeed();

private:
    Ticker ticker;

    float target_rps = 0;
    float current_rps = 0;
    float last_position = 0;
    float last_duty = 0;
    float max_duty;
    bool moving = false;

    void loop();
};
