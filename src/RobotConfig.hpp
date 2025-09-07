#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <cmath>

namespace RobotConfig {
    // --- 数学定数 ---
    constexpr float SQRT3 = 1.73205081f;
    constexpr float PI = 3.14159265f;

    // --- 物理パラメータ ---
    constexpr float WHEEL_RADIUS_M = 0.05f;  // オムニホイールの半径 [m]
    constexpr float BODY_RADIUS_M = 0.15f;   // 中心からホイールまでの距離 [m]
    constexpr float COUNTS_PER_REV = 2048.0f; // モーター1回転のエンコーダカウント

    // --- 測定輪パラメータ ---
    constexpr float ODOM_WHEEL_RADIUS_M = 0.025f; // 測定輪の半径 [m]
    constexpr float ODOM_COUNTS_PER_REV = 2048.0f; // 測定輪エンコーダの分解能

    // === ★★★ IMUフィルタ パラメータ (追加) ★★★ ===
    // ローパスフィルタの平滑化係数alpha。0に近いほど強く平滑化（動きが滑らかになるが反応は鈍る）、
    // 1に近いほど生のデータに近い値（反応は速いがノイズは残りやすい）になります。
    // まずはこの値で試し、反応が鈍いと感じたら少しずつ大きくしてみてください。
    constexpr float IMU_FILTER_ALPHA = 0.1f;
    
    // === ★★★ 速度PIDゲイン (安定性重視) ★★★ ===
    // KPを下げて振動を抑制。KIでトルクを補い、小さなKDで応答性を安定させます。
    constexpr float KP = 0.05f;
    constexpr float KI = 0.02f;
    constexpr float KD = 0.001f;

    // --- 位置制御パラメータ ---
    constexpr float POS_CONTROL_KP = 1.5f;       // 直進制御のPゲイン
    constexpr float GOAL_THRESHOLD_M = 0.02f;    // ゴール座標の許容誤差 (2cm)

    // --- ★★★ 角度制御パラメータ (安定性重視) ★★★ ===
    // ANGLE_CONTROL_KPを大幅に下げ、回転時の振動とオーバーシュートを徹底的に抑制します。
    constexpr float ANGLE_CONTROL_KP = 0.5f;     // 回転制御のPゲイン
    constexpr float ANGLE_THRESHOLD_RAD = 0.05f; // ゴール角度の許容誤差 (約3度)

    // --- サーボモーター パラメータ ---
    constexpr int SERVO_MIN_PULSE_WIDTH = 500;
    constexpr int SERVO_MAX_PULSE_WIDTH = 2400;

    // --- アーム1 (CoalArm) パラメータ ---
    constexpr float ARM1_LIFT_KP = 20.0f;
    constexpr float ARM1_LIFT_KI = 0.1f;
    constexpr float ARM1_LIFT_KD = 0.3f;
    constexpr float ARM1_LIFT_METERS_PER_REV = 0.05f;
    constexpr float ARM1_LIFT_COUNTS_PER_REV = 3200.0f;
    constexpr float ARM1_GRIP_OPEN_DEG = 90.0f;
    constexpr float ARM1_GRIP_CLOSE_DEG = 20.0f;

    // --- アーム2 (OilArm) パラメータ ---
    constexpr float ARM2_LIFT_KP = 20.0f;
    constexpr float ARM2_LIFT_KI = 0.1f;
    constexpr float ARM2_LIFT_KD = 0.3f;
    constexpr float ARM2_LIFT_METERS_PER_REV = 0.05f;
    constexpr float ARM2_LIFT_COUNTS_PER_REV = 3200.0f;
    
    constexpr float ARM2_GRIP_KP = 15.0f;
    constexpr float ARM2_GRIP_KI = 0.1f;
    constexpr float ARM2_GRIP_KD = 0.2f;
    constexpr float ARM2_GRIP_METERS_PER_REV = 0.01f;
    constexpr float ARM2_GRIP_COUNTS_PER_REV = 3200.0f;
    constexpr float ARM2_GRIP_OPEN_ANGLE_DEG = 30.0f; // グリッパーオープン角度
    constexpr float ARM2_GRIP_CLOSE_ANGLE_DEG = -30.0f; // グリッパークローズ角度

    // --- アーム共通動作パラメータ ---
    constexpr float ARM_LIFT_HEIGHT_M = 0.1f; // アームを持ち上げる高さ (10cm)
}

#endif // ROBOT_CONFIG_HPP
