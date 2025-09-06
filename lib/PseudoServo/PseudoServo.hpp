#ifndef PSEUDO_SERVO_HPP
#define PSEUDO_SERVO_HPP

#include "DCMotor.hpp"
#include "Encoder.hpp"

class PseudoServo {
public:
    /**
     * @brief コンストラクタ
     * @param motor 制御対象のDCモーター
     * @param encoder 位置フィードバック用のエンコーダ
     */
    PseudoServo(DCMotor& motor, Encoder& encoder);

    /**
     * @brief PIDゲインや物理パラメータを設定
     * @param kp Pゲイン
     * @param ki Iゲイン
     * @param kd Dゲイン
     * @param meters_per_rev ラック＆ピニオン機構で、モーターが1回転したときの移動量 [m]
     * @param counts_per_rev エンコーダの1回転あたりのカウント数
     */
    void setParams(float kp, float ki, float kd, float meters_per_rev, float counts_per_rev);

    /**
     * @brief 目標位置を設定
     * @param position_m スタート地点からの目標移動距離 [m]
     */
    void setPosition(float position_m);

    /**
     * @brief PID制御を更新する。メインループで定期的に呼び出す。
     * @param dt 前回の呼び出しからの経過時間 [s]
     */
    void update(float dt);

    /**
     * @brief モーターの動きを停止させる
     */
    void stop();

    /**
     * @brief 目標位置に到達したか判定
     * @param tolerance_m 許容誤差 [m]
     * @return 到達していればtrue
     */
    bool isAtTarget(float tolerance_m = 0.001f);

private:
    DCMotor& m_motor;
    Encoder& m_encoder;

    // パラメータ
    float m_kp, m_ki, m_kd;
    float m_counts_per_meter;

    // 状態変数
    float m_target_position_m;
    float m_integral;
    float m_prev_error;
};

#endif // PSEUDO_SERVO_HPP
