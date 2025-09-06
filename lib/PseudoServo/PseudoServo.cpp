#include "PseudoServo.hpp"
#include <cmath>

PseudoServo::PseudoServo(DCMotor& motor, Encoder& encoder)
    : m_motor(motor), m_encoder(encoder),
      m_kp(0), m_ki(0), m_kd(0), m_counts_per_meter(0),
      m_target_position_m(0), m_integral(0), m_prev_error(0)
{
}

void PseudoServo::setParams(float kp, float ki, float kd, float meters_per_rev, float counts_per_rev) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    if (meters_per_rev > 0) {
        m_counts_per_meter = counts_per_rev / meters_per_rev;
    }
}

void PseudoServo::setPosition(float position_m) {
    m_target_position_m = position_m;
}

void PseudoServo::update(float dt) {
    // 1. 現在位置を計算
    long current_count = m_encoder.getCount();
    float current_position_m = (float)current_count / m_counts_per_meter;

    // 2. 目標位置との誤差を計算
    float error = m_target_position_m - current_position_m;

    // 3. PID制御計算
    m_integral += error * dt;
    float derivative = (dt > 0) ? (error - m_prev_error) / dt : 0;
    
    float output = m_kp * error + m_ki * m_integral + m_kd * derivative;

    // 4. 出力を制限 (-1.0 ~ 1.0)
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;

    // 5. モーターに指令
    m_motor.setDuty(output);

    // 6. 次回計算のために誤差を保存
    m_prev_error = error;
}

void PseudoServo::stop() {
    m_motor.stop();
}

bool PseudoServo::isAtTarget(float tolerance_m) {
    long current_count = m_encoder.getCount();
    float current_position_m = (float)current_count / m_counts_per_meter;
    return std::abs(m_target_position_m - current_position_m) < tolerance_m;
}
