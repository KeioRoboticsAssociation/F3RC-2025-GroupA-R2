#pragma once
#include "Wheel.hpp"
#include "WheelController.hpp"
#include "Odometry.hpp"

// N: 車輪の数
//
// OdomWheelは自己位置と目標位置を基にPID制御で機体の目標速度を計算し、WheelControllerに渡す。
// WheelControllerは入力された目標速度を各車輪の目標速度に変換し、MotorControllerに渡す。
// MotorControllerはPID制御を行い入力された目標速度に近づける。
template <int N>
class OdomWheel
{
public:
    OdomWheel(const std::array<WheelConfig, N> &wheel_configs, const std::array<MotorController *, N> &motors, const std::array<Encoder *, N> &encoders, chrono::microseconds update_interval = chrono::microseconds(5000))
        : wheel_controller(wheel_configs, motors), odometry(wheel_configs, encoders, update_interval)
    {
        wheel_controller.setTargetTwist({0.0, 0.0, 0.0});
        odometry.setPose({0.0, 0.0, 0.0});
    }

    void updateTargetTwist()
    {
        Pose current_pose = odometry.getCurrentPose();
        Pose target_pose = getTargetPose();

        Pose error = target_pose - current_pose;
        // 返ってくるのはTwistなので型変換
        Pose result = pid_controller.calculate(error);
        Twist target_twist{result.x, result.y, result.theta};

        wheel_controller.setTargetTwist(target_twist);
    }

    void setTargetPose(Pose target_pose)
    {
        this->target_pose = target_pose;
    }

    Pose getTargetPose()
    {
        return target_pose;
    }

    PIDController<Pose> pid_controller;
    WheelController<N> wheel_controller;
    Odometry<N> odometry;

private:
    Pose target_pose;
};
