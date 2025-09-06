#pragma once
#include <DCMotor.hpp>
#include <Encoder.hpp>
#include "Mechanism/PIDController.hpp"
#include "Mechanism/Wheel.hpp"
#include "Mechanism/WheelController.hpp"
#include "Mechanism/MotorController.hpp"
#include "config.hpp"
#include "variable.hpp"
#include <DCMotor.hpp>
#include "Control/StateEstimator.hpp"
#include "Mechanism/WheelController.hpp"

enum class TargetMode
{
    Position, // 位置制御
    Velocity  // 速度制御
};

class BehaviorController
{
public:
    
    //=====================================================================================
    //　 以下、使用するメソッド一覧　(速度がおすすめ)　※これらメソッドは原則ループ内で呼び出し
    //   加速度指定は未だ未実装
    //=====================================================================================

    BehaviorController(PIDGain x_pid_gain, PIDGain y_pid_gain, PIDGain angle_pid_gain);
    // 座標(ロボット位置原点の座標系)の目標値を設定
    void setTargetPosition(double x, double y);
    // 並進速度(ロボット位置原点の座標系)の目標値を設定
    void setTargetVelocity(double vx, double vy);
    // 回転角の目標値を設定
    void setTargetAngle(double theta);
    // 回転角速度の目標値を設定
    void setTargetAngularVelocity(double omega);

    // 一気に現状を設定する
    void updateFromStateEstimator(const StateEstimator& state);

    // 位置のPID制御で目標速度を算出する
    Twist calculateTargetVelocity();
    // 速度のPID制御で目標加速度を算出する
    TwistAccel calculateTargetAccel();
    // 実際に動かす
    void setMotor();
    // 完全に停止する
    void stop();

    void test();
    void setPGain(float p);
    

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
    DCMotor FrontMotor;
    Encoder FrontEncoder;
    DCMotor RearLeftMotor;
    Encoder RearLeftEncoder;
    DCMotor RearRightMotor;
    Encoder RearRightEncoder;

    MotorController FrontMotorController;
    MotorController RearLeftMotorController;
    MotorController RearRightMotorController;

    PIDController<double> x_pid_controller;
    PIDController<double> y_pid_controller;
    PIDController<double> angle_pid_controller;
    PIDController<double> vx_pid_controller;
    PIDController<double> vy_pid_controller;
    PIDController<double> omega_pid_controller;
    // calculateTargetVelocityで扱うx, yの目標が位置か速度のどちらなのか。
    // true: 位置, false: 速度
    TargetMode xy_target_mode = TargetMode::Velocity;
    // calculateTargetVelocityで扱うangleの目標が位置か速度のどちらなのか。
    // true: 位置, false: 速度
    TargetMode angle_target_mode = TargetMode::Velocity;

    WheelController<3> wheel_v_controller;

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
};
