#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <chrono>

namespace RobotConfig {
    // ロボットの物理パラメータ
    constexpr float WHEEL_RADIUS = 0.045f; // ホイールの半径 [m]
    constexpr float BODY_RADIUS = 0.21f;  // 車体中心からホイール中心までの距離 [m]
    constexpr int COUNTS_PER_REV = 2048; // モーター1回転あたりのエンコーダカウント数（ギア比考慮後）

    // 制御周期
    constexpr float CONTROL_PERIOD = 0.01f; // 10ms

    // 三角関数
    constexpr float COS_120 = -0.5f; // cos(120°)
    constexpr float SIN_120 = 0.866f; // sin(120°)

    // 位置制御
    constexpr float POS_CONTROL_KP = 1.0f; // 位置制御の比例ゲイン
    constexpr float GOAL_THRESHOLD_M = 0.03f; // 目標位置到達の閾値 [m]
    constexpr float GOAL_THRESHOLD_RAD = 0.03f; // 目標位置到達の閾値 [rad]

    // サーボ
    constexpr chrono::microseconds SERVO_MIN_PULSE_WIDTH = 500us; // サーボの最小パルス幅 [μs]
    constexpr chrono::microseconds SERVO_MAX_PULSE_WIDTH = 2500us; // サーボの最大パルス幅 [μs]

    // オムニホイール PIDゲイン
    constexpr float KP = 0.5f; // 比例ゲイン
    constexpr float KI = 0.1f; // 積分ゲイン
    constexpr float KD = 0.01f; // 微分ゲイン

    // アーム1（石炭アーム）設定
    constexpr float ARM1_LIFT_KP = 5.0f; // 昇降用PID比例ゲイン
    constexpr float ARM1_LIFT_KI = 0.0f; // 昇降用PID積分ゲイン
    constexpr float ARM1_LIFT_KD = 0.1f; // 昇降用PID微分ゲイン
    constexpr float ARM1_METERS_PER_REV = 0.02f; // 昇降用モーターの1回転あたりの移動距離
    constexpr int ARM1_COUNTS_PER_REV = 2048; // 昇降用モーターの1回転あたりのエンコーダカウント数
    constexpr float ARM1_GRIPPER_OPEN_DEG = 45.0f; // グリッパーオープン角度
    constexpr float ARM1_GRIPPER_CLOSE_DEG = -45.0f; // グリッパークローズ角度

    // アーム2（石油アーム）設定
    constexpr float ARM2_LIFT_KP = 5.0f; // 昇降用PID比例ゲイン
    constexpr float ARM2_LIFT_KI = 0.0f; // 昇降用PID積分ゲイン
    constexpr float ARM2_LIFT_KD = 0.1f; // 昇降用PID微分ゲイン
    constexpr float ARM2_LIFT_METERS_PER_REV = 0.02f; // 昇降用モーターの1回転あたりの移動距離
    constexpr int ARM2_LIFT_COUNTS_PER_REV = 2048; // 昇降用モーターの1回転あたりのエンコーダカウント数
    constexpr float ARM2_GRIP_KP = 2.0f; // グリッパー用PID比例ゲイン
    constexpr float ARM2_GRIP_KI = 0.0f; // グリッパー用PID積分ゲイン
    constexpr float ARM2_GRIP_KD = 0.05f; // グリッパー用PID微分ゲイン
    constexpr float ARM2_GRIP_METERS_PER_REV = 0.0005f; // グリッパー用モーターの1回転あたりの移動距離
    constexpr int ARM2_GRIP_COUNTS_PER_REV = 2048; // グリッパー用モーターの1回転あたりのエンコーダカウント数
    constexpr float ARM2_GRIP_OPEN_ANGLE_DEG = 30.0f; // グリッパーオープン角度
    constexpr float ARM2_GRIP_CLOSE_ANGLE_DEG = -30.0f; // グリッパークローズ角度

} // namespace RobotConfig

#endif // ROBOT_CONFIG_HPP
