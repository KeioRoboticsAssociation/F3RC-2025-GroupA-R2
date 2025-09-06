#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

namespace RobotConfig {
    constexpr float WHEEL_RADIUS = 0.05f; // ホイールの半径 [m]
    constexpr float BODY_RADIUS = 0.15f;  // 車体中心からホイール中心までの距離 [m]
    constexpr int COUNTS_PER_REV = 2048; // モーター1回転あたりのエンコーダカウント数（ギア比考慮後）

    // PIDゲイン
    constexpr float KP = 0.5f; // 比例ゲイン
    constexpr float KI = 0.1f; // 積分ゲイン
    constexpr float KD = 0.01f; // 微分ゲイン

    // 制御周期
    constexpr float CONTROL_PERIOD = 0.01f; // 10ms

    // 三角関数
    constexpr float COS_120 = -0.5f; // cos(120°)
    constexpr float SIN_120 = 0.866f; // sin(120°)

} // namespace Variable

#endif // ROBOT_CONFIG_HPP
