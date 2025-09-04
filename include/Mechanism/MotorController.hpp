#pragma once
#include "Parts/DCMotor.hpp"
#include "Parts/Encoder.hpp"
#include "PIDController.hpp"
#include "mbed.h"

// PID制御を用いてDCモーターの出力を制御するクラス
class MotorController
{
public:
    DCMotor &motor;
    Encoder &encoder;
    PIDController<float> pid_controller;

    MotorController(DCMotor &motor, Encoder &encoder, PIDGain pid_gain = PIDGain{5.0, 0.0, 0.0, 20}, float max_duty = 1.0);
    void setTargetSpeed(float target_rps);
    void stop();
    float getSpeed();
    float getTargetSpeed();
    
    void print();

    // ループ呼び出しフラグ操作
    bool isLoopCalled() const { return loop_called; }
    void clearLoopFlag() { loop_called = false; }

    float output_;
    float encoder_;
    float current_rps_;

private:
    Ticker ticker;

    float target_rps = 0;
    float current_rps = 0;
    float last_position = 0;
    float last_duty = 0;
    float max_duty;
    bool moving = false;

    volatile bool loop_called = false; // ISR→main 通信用フラグ

    void loop();
};
