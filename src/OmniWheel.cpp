#include "OmniWheel.hpp"
#include "RobotConfig.hpp" // 設定ファイルをインクルード
#include <cmath>
#include <algorithm>      // std::max, std::min のために追加

// M_PIが定義されていない場合のためのフォールバック
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

OmniWheel::OmniWheel(DCMotor& motor1, Encoder& encoder1,
                     DCMotor& motor2, Encoder& encoder2,
                     DCMotor& motor3, Encoder& encoder3)
    // メンバ初期化子リストで参照を初期化
    : m_motor1(motor1), m_encoder1(encoder1),
      m_motor2(motor2), m_encoder2(encoder2),
      m_motor3(motor3), m_encoder3(encoder3)
{
    // メンバ変数をゼロやデフォルト値で初期化
    m_wheel_radius = 0.0f;
    m_body_radius = 0.0f;
    m_rads_per_count = 0.0f;
    m_kp = 0.0f; m_ki = 0.0f; m_kd = 0.0f;

    // 起動時は停止状態にする
    stop();

    // 次回の速度計算のために、現在のエンコーダ値を保存
    m_prev_count1 = m_encoder1.getCount();
    m_prev_count2 = m_encoder2.getCount();
    m_prev_count3 = m_encoder3.getCount();
}

void OmniWheel::setParams(float wheel_radius, float body_radius, float counts_per_rev,
                          float kp, float ki, float kd)
{
    m_wheel_radius = wheel_radius;
    m_body_radius = body_radius;
    if (counts_per_rev > 0) {
        // 1カウントあたりの回転角度[rad]を計算
        m_rads_per_count = (2.0f * M_PI) / counts_per_rev;
    }
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void OmniWheel::move(float vx, float vy, float omega) {
    if (m_wheel_radius <= 0) return; // パラメータ未設定の場合は何もしない

    const float l = m_body_radius;
    const float r = m_wheel_radius;
    
    // 逆運動学の計算
    // モーターの物理的な配置はRobotConfig.hppで定義された定数に依存します
    // ※モーター配置の仮定は前回と同じです
    // motor1: +x軸から -90 deg (-y方向)
    // motor2: +x軸から +150 deg
    // motor3: +x軸から +30 deg
    // この計算は実際のロボットのモーター配置に合わせて調整が必要です
    m_target_speed1 = (1.0f / r) * (vx - l * omega);
    m_target_speed2 = (1.0f / r) * (RobotConfig::COS_120 * vx + RobotConfig::SIN_120 * vy - l * omega);
    m_target_speed3 = (1.0f / r) * (RobotConfig::COS_120 * vx - RobotConfig::SIN_120 * vy - l * omega);
}

void OmniWheel::update(float dt) {
    // 経過時間(dt)が非常に短い、または0以下の場合は処理をスキップ
    if (dt <= 1e-6f) return;

    // 1. 各エンコーダの現在のカウント値を取得
    long current_count1 = m_encoder1.getCount();
    long current_count2 = m_encoder2.getCount();
    long current_count3 = m_encoder3.getCount();

    // 2. 現在の角速度を計算 [rad/s]
    float current_speed1 = (float)(current_count1 - m_prev_count1) * m_rads_per_count / dt;
    float current_speed2 = (float)(current_count2 - m_prev_count2) * m_rads_per_count / dt;
    float current_speed3 = (float)(current_count3 - m_prev_count3) * m_rads_per_count / dt;

    // --- PID制御計算 (各モーターごと) ---
    // モーター1
    float error1 = m_target_speed1 - current_speed1;
    m_integral1 += error1 * dt;
    float derivative1 = (error1 - m_prev_error1) / dt;
    float output1 = m_kp * error1 + m_ki * m_integral1 + m_kd * derivative1;

    // モーター2
    float error2 = m_target_speed2 - current_speed2;
    m_integral2 += error2 * dt;
    float derivative2 = (error2 - m_prev_error2) / dt;
    float output2 = m_kp * error2 + m_ki * m_integral2 + m_kd * derivative2;
    
    // モーター3
    float error3 = m_target_speed3 - current_speed3;
    m_integral3 += error3 * dt;
    float derivative3 = (error3 - m_prev_error3) / dt;
    float output3 = m_kp * error3 + m_ki * m_integral3 + m_kd * derivative3;
    
    // 3. 計算結果をモーターに出力 (デューティ比を -1.0 ~ 1.0 の範囲に制限)
    m_motor1.setDuty(std::max(-1.0f, std::min(1.0f, output1)));
    m_motor2.setDuty(std::max(-1.0f, std::min(1.0f, output2)));
    m_motor3.setDuty(std::max(-1.0f, std::min(1.0f, output3)));

    // 4. 次回計算のために値を保存
    m_prev_count1 = current_count1;
    m_prev_count2 = current_count2;
    m_prev_count3 = current_count3;
    m_prev_error1 = error1;
    m_prev_error2 = error2;
    m_prev_error3 = error3;
}

void OmniWheel::stop() {
    // 目標速度をゼロに設定
    m_target_speed1 = 0.0f;
    m_target_speed2 = 0.0f;
    m_target_speed3 = 0.0f;

    // PIDの積分値や前回誤差などの状態をリセット
    m_integral1 = 0.0f; m_integral2 = 0.0f; m_integral3 = 0.0f;
    m_prev_error1 = 0.0f; m_prev_error2 = 0.0f; m_prev_error3 = 0.0f;
    
    // モーターを物理的に停止させる
    m_motor1.stop();
    m_motor2.stop();
    m_motor3.stop();
}
