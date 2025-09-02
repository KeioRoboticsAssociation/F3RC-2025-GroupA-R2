#pragma once
#include "Mechanism/PIDController.hpp"
#include "Provider/Wheel.hpp"
#include "Mechanism/WheelController.hpp"
#include "config.hpp"
#include "variable.hpp"
#include "Parts/DCMotor.hpp"

enum class TargetMode
{
    Position, // 位置制御
    Velocity  // 速度制御
};

class BehaviorController
{
public:
    // デフォルト WheelConfig
    static std::array<WheelConfig, 3> defaultWheelConfigs() {
        return {WheelConfig{0.0f},
                WheelConfig{120.0f * M_PI / 180},
                WheelConfig{240.0f * M_PI / 180}};
    }

    // デフォルトモーター配列
    static std::array<MotorController*, 3> defaultMotors();

    // フルコンストラクタ（デフォルト引数で簡単呼び出し可能）
    BehaviorController(PIDGain x_pid_gain = BehaviorControllerParameter::x_gain,
                       PIDGain y_pid_gain = BehaviorControllerParameter::y_gain,
                       PIDGain angle_pid_gain = BehaviorControllerParameter::angle_gain,
                       const std::array<WheelConfig, 3>& wheel_configs = defaultWheelConfigs(),
                       const std::array<MotorController*, 3>& motors = defaultMotors(),
                       float max_speed = 100.0f);

    // 座標・速度・加速度設定
    void setTargetPosition(double x, double y);
    void setTargetVelocity(double vx, double vy);
    void setTargetAngle(double theta);
    void setTargetAngularVelocity(double omega);
    void setPosition(double x, double y);
    void setVelocity(double vx, double vy);
    void setAcceleration(double ax, double ay);
    void setAngle(double theta);
    void setAngularVelocity(double omega);
    void setAngularAcceleration(double alpha);

    // 挙動実行
    void move();
    void stop();

private:
    WheelController<3> wheel_controller;

    PIDController<double> x_pid_controller;
    PIDController<double> y_pid_controller;
    PIDController<double> angle_pid_controller;

    TargetMode xy_target_mode;
    TargetMode angle_target_mode;

    // 状態量
    double current_pos_x = 0;
    double current_pos_y = 0;
    double current_angle = 0;
    double current_velocity_x = 0;
    double current_velocity_y = 0;
    double current_velocity_angle = 0;
    double current_acceleration_x = 0;
    double current_acceleration_y = 0;
    double current_acceleration_angle = 0;

    // 目標値
    double target_pos_x = 0;
    double target_pos_y = 0;
    double target_angle = 0;
    double target_velocity_x = 0;
    double target_velocity_y = 0;
    double target_velocity_angle = 0;

    // PID制御で目標速度を算出
    Twist calculateTargetVelocity();
};
