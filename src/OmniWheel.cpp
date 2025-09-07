#include "OmniWheel.hpp"
#include <cmath>

OmniWheel::OmniWheel(DCMotor& motor1, Encoder& encoder1,
                     DCMotor& motor2, Encoder& encoder2,
                     DCMotor& motor3, Encoder& encoder3)
    : m_motor1(motor1), m_encoder1(encoder1),
      m_motor2(motor2), m_encoder2(encoder2),
      m_motor3(motor3), m_encoder3(encoder3),
      m_wheel_radius(0.0f), m_body_radius(0.0f),
      m_counts_per_rev(0.0f),
      m_kp(0.0f), m_ki(0.0f), m_kd(0.0f),
      m_integral_limit(0.0f)
{
    stop(); // 初期化時に内部状態をリセット
}

void OmniWheel::setParams(float wheel_radius, float body_radius, float counts_per_rev,
                          float kp, float ki, float kd)
{
    m_wheel_radius = wheel_radius;
    m_body_radius = body_radius;
    m_counts_per_rev = counts_per_rev;
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    if (m_ki > 0.0f) {
        m_integral_limit = 0.5f / m_ki;
    } else {
        m_integral_limit = 0.0f;
    }
}

void OmniWheel::update(float dt) {
    if (dt <= 0) return;

    // --- モーター1のPID制御 ---
    int current_count1 = m_encoder1.getCount();
    int count_delta1 = current_count1 - m_prev_count1;
    float current_speed1 = (float)count_delta1 / m_counts_per_rev * (2.0f * RobotConfig::PI) / dt;
    m_prev_count1 = current_count1;

    float error1 = m_target_speed1 - current_speed1;
    m_integral1 += error1 * dt;
    if (m_integral1 > m_integral_limit) m_integral1 = m_integral_limit;
    else if (m_integral1 < -m_integral_limit) m_integral1 = -m_integral_limit;

    float derivative1 = (error1 - m_prev_error1) / dt;
    float duty1 = m_kp * error1 + m_ki * m_integral1 + m_kd * derivative1;
    m_prev_error1 = error1;

    // --- モーター2のPID制御 ---
    int current_count2 = m_encoder2.getCount();
    int count_delta2 = current_count2 - m_prev_count2;
    float current_speed2 = (float)count_delta2 / m_counts_per_rev * (2.0f * RobotConfig::PI) / dt;
    m_prev_count2 = current_count2;
    
    float error2 = m_target_speed2 - current_speed2;
    m_integral2 += error2 * dt;
    if (m_integral2 > m_integral_limit) m_integral2 = m_integral_limit;
    else if (m_integral2 < -m_integral_limit) m_integral2 = -m_integral_limit;

    float derivative2 = (error2 - m_prev_error2) / dt;
    float duty2 = m_kp * error2 + m_ki * m_integral2 + m_kd * derivative2;
    m_prev_error2 = error2;

    // --- モーター3のPID制御 ---
    int current_count3 = m_encoder3.getCount();
    int count_delta3 = current_count3 - m_prev_count3;
    float current_speed3 = (float)count_delta3 / m_counts_per_rev * (2.0f * RobotConfig::PI) / dt;
    m_prev_count3 = current_count3;

    float error3 = m_target_speed3 - current_speed3;
    m_integral3 += error3 * dt;
    if (m_integral3 > m_integral_limit) m_integral3 = m_integral_limit;
    else if (m_integral3 < -m_integral_limit) m_integral3 = -m_integral_limit;

    float derivative3 = (error3 - m_prev_error3) / dt;
    float duty3 = m_kp * error3 + m_ki * m_integral3 + m_kd * derivative3;
    m_prev_error3 = error3;

    // 最終的な出力のクランプは安全のために残します
    if (duty1 > 1.0f) duty1 = 1.0f; else if (duty1 < -1.0f) duty1 = -1.0f;
    if (duty2 > 1.0f) duty2 = 1.0f; else if (duty2 < -1.0f) duty2 = -1.0f;
    if (duty3 > 1.0f) duty3 = 1.0f; else if (duty3 < -1.0f) duty3 = -1.0f;
    
    m_motor1.setDuty(duty1);
    m_motor2.setDuty(duty2);
    m_motor3.setDuty(duty3);
}

void OmniWheel::move(float vx, float vy, float omega) {
    if (vx == 0.0f && vy == 0.0f && omega == 0.0f) {
        stop();
        return;
    }

    // === ★★★ 物理的に正しい逆運動学計算 (最終版) ★★★ ===
    // これまでの試行錯誤をリセットし、標準的な計算式に戻します。
    // オドメトリとゲインが正しければ、これで安定するはずです。
    m_target_speed1 = (1.0f / m_wheel_radius) * (-vy + m_body_radius * omega);
    m_target_speed2 = (1.0f / m_wheel_radius) * (vx * RobotConfig::SQRT3 / 2.0f + vy * 0.5f + m_body_radius * omega);
    m_target_speed3 = (1.0f / m_wheel_radius) * (-vx * RobotConfig::SQRT3 / 2.0f + vy * 0.5f + m_body_radius * omega);
}

void OmniWheel::stop() {
    m_target_speed1 = 0.0f;
    m_target_speed2 = 0.0f;
    m_target_speed3 = 0.0f;

    m_integral1 = 0.0f;
    m_integral2 = 0.0f;
    m_integral3 = 0.0f;

    m_prev_error1 = 0.0f;
    m_prev_error2 = 0.0f;
    m_prev_error3 = 0.0f;
    
    m_encoder1.reset();
    m_encoder2.reset();
    m_encoder3.reset();
    m_prev_count1 = m_encoder1.getCount();
    m_prev_count2 = m_encoder2.getCount();
    m_prev_count3 = m_encoder3.getCount();
}

OmniWheel::MotorDuties OmniWheel::getMotorDuties() const {
    return {
        m_motor1.getDuty(),
        m_motor2.getDuty(),
        m_motor3.getDuty()
    };
}
