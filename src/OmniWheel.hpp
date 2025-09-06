#ifndef OMNI_WHEEL_HPP
#define OMNI_WHEEL_HPP

#include "mbed.h"
#include <DCMotor.hpp>
#include <Encoder.hpp>

class OmniWheel {
public:
    /**
     * @brief コンストラクタ
     */
    OmniWheel(DCMotor& motor1, Encoder& encoder1,
              DCMotor& motor2, Encoder& encoder2,
              DCMotor& motor3, Encoder& encoder3);

    /**
     * @brief ロボットの物理パラメータとPIDゲインを設定します
     */
    void setParams(float wheel_radius, float body_radius, float counts_per_rev,
                   float kp, float ki, float kd);

    /**
     * @brief ロボットの目標速度を設定します
     */
    void move(float vx, float vy, float omega);

    /**
     * @brief PID制御ループ。メインループから一定周期で呼び出してください
     * @param dt 前回updateが呼ばれてからの経過時間 [s]
     */
    void update(float dt);

    /**
     * @brief 全てのモーターを停止し、PID制御の状態をリセットします
     */
    void stop();

private:
    // モーターとエンコーダのインスタンス
    DCMotor& m_motor1;
    Encoder& m_encoder1;
    DCMotor& m_motor2;
    Encoder& m_encoder2;
    DCMotor& m_motor3;
    Encoder& m_encoder3;

    // 物理パラメータ
    float m_wheel_radius;
    float m_body_radius;
    float m_rads_per_count;

    // PID制御関連の変数
    float m_kp, m_ki, m_kd;
    float m_target_speed1, m_target_speed2, m_target_speed3;
    float m_integral1, m_integral2, m_integral3;
    float m_prev_error1, m_prev_error2, m_prev_error3;
    long m_prev_count1, m_prev_count2, m_prev_count3;
};

#endif // OMNI_WHEEL_HPP
