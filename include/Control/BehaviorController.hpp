#pragma once
#include "Mechanism/PIDController.hpp"
#include "Provider/Wheel.hpp"

enum class TargetMode
{
    Position, // 位置制御
    Velocity  // 速度制御
};

class BehaviorController
{
public:
    BehaviorController(PIDGain x_pid_gain, PIDGain y_pid_gain, PIDGain angle_pid_gain);
    // 座標(ロボット位置原点の座標系)の目標値を設定
    void setTargetPosition(double x, double y);
    // 並進速度(ロボット位置原点の座標系)の目標値を設定
    void setTargetVelocity(double vx, double vy);
    // 回転角の目標値を設定
    void setTargetAngle(double theta);
    // 回転角速度の目標値を設定
    void setTargetAngularVelocity(double omega);
    // 現在の座標(ロボット位置原点の座標系)を設定
    void setPosition(double x, double y);
    // 現在の並進速度(ロボット位置原点の座標系)を設定
    void setVelocity(double vx, double vy);
    // 現在の並進加速度(ロボット位置原点の座標系)を設定
    void setAcceleration(double ax, double ay);
    // 現在の回転角度の設定
    void setAngle(double theta);
    // 現在の回転角速度の設定
    void setAngularVelocity(double omega);
    // 現在の回転角加速度の設定
    void setAngularAcceleration(double alpha);
    // PID制御で目標速度を算出する
    Twist calculateTargetVelocity();

    double current_pos_x;
    double current_pos_y;
    double current_angle;
    double current_velocity_x;
    double current_velocity_y;
    double current_velocity_angle;
    double current_acceleration_x;
    double current_acceleration_y;
    double current_acceleration_angle;
    double target_pos_x;
    double target_pos_y;
    double target_angle;
    double target_velocity_x;
    double target_velocity_y;
    double target_velocity_angle;

private:
    PIDController<double> x_pid_controller;
    PIDController<double> y_pid_controller;
    PIDController<double> angle_pid_controller;
    // calculateTargetVelocityで扱うx, yの目標が位置か速度のどちらなのか。
    // true: 位置, false: 速度
    TargetMode xy_target_mode;
    // calculateTargetVelocityで扱うangleの目標が位置か速度のどちらなのか。
    // true: 位置, false: 速度
    TargetMode angle_target_mode;
};
