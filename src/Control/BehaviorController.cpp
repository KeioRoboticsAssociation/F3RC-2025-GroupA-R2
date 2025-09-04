#include "Control/BehaviorController.hpp"
#include "Provider/Wheel.hpp"

BehaviorController::BehaviorController(PIDGain x_pid_gain, PIDGain y_pid_gain, PIDGain angle_pid_gain)
    : x_pid_controller(x_pid_gain), y_pid_controller(y_pid_gain), angle_pid_controller(angle_pid_gain) {}

void BehaviorController::setTargetPosition(double x, double y)
{
    target_pos_x = x;
    target_pos_y = y;
    xy_target_mode = TargetMode::Position;
}

void BehaviorController::setTargetVelocity(double vx, double vy)
{
    target_velocity_x = vx;
    target_velocity_y = vy;
    xy_target_mode = TargetMode::Velocity;
}

void BehaviorController::setTargetAngle(double theta)
{
    target_angle = theta;
    angle_target_mode = TargetMode::Position;
}

void BehaviorController::setTargetAngularVelocity(double omega)
{
    target_velocity_angle = omega;
    angle_target_mode = TargetMode::Velocity;
}

void BehaviorController::setPosition(double x, double y)
{
    current_pos_x = x;
    current_pos_y = y;
}

void BehaviorController::setVelocity(double vx, double vy)
{
    current_velocity_x = vx;
    current_velocity_y = vy;
}

void BehaviorController::setAcceleration(double ax, double ay)
{
    current_acceleration_x = ax;
    current_acceleration_y = ay;
}

void BehaviorController::setAngle(double theta)
{
    current_angle = theta;
}

void BehaviorController::setAngularVelocity(double omega)
{
    current_velocity_angle = omega;
}

void BehaviorController::setAngularAcceleration(double alpha)
{
    current_acceleration_angle = alpha;
}

Twist BehaviorController::calculateTargetVelocity()
{
    double x_output;
    double y_output;
    if (xy_target_mode == TargetMode::Position)
    {
        x_output = x_pid_controller.calculate(target_pos_x - current_pos_x);
        y_output = y_pid_controller.calculate(target_pos_y - current_pos_y);
    }
    else
    {
        x_output = target_velocity_x;
        y_output = target_velocity_y;
    }

    double angle_output;
    if (angle_target_mode == TargetMode::Position)
    {
        angle_output = angle_pid_controller.calculate(target_angle - current_angle);
    }
    else
    {
        angle_output = target_velocity_angle;
    }

    return Twist{(float)x_output, (float)y_output, (float)angle_output};
}
