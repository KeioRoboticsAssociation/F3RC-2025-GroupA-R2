#include "Provider/ImuOdometry.hpp"
#include <cmath> // M_PI, cos, sin

ImuOdometry::ImuOdometry(Database& db) : database_(db) {
    // 内部で保持する姿勢と速度をゼロで初期化
    pose_ = {0.0, 0.0, 0.0};
    velocity_ = {0.0, 0.0};
}

void ImuOdometry::update(double dt) {
    // --- 1. 角度の更新 ---
    const double angular_velocity_dps = database_.getAngularVelocity();
    const double angular_velocity_rad = angular_velocity_dps * M_PI / 180.0;
    pose_.theta += angular_velocity_rad * dt;

    // 角度を -PI から PI の範囲に正規化 
    pose_.theta = fmod(pose_.theta + M_PI, 2.0 * M_PI);
    if (pose_.theta < 0) {
        pose_.theta += 2.0 * M_PI;
    }
    pose_.theta -= M_PI;

    // --- 2. 位置と速度の更新 ---
    const std::pair<double, double> body_accel = database_.getAcceleration();
    const double ax_body = body_accel.first;
    const double ay_body = body_accel.second;

    const double theta = pose_.theta;
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    // 機体座標系の加速度をワールド座標系に変換
    const double ax_world = ax_body * cos_theta - ay_body * sin_theta;
    const double ay_world = ax_body * sin_theta + ay_body * cos_theta;

    // ワールド座標系での速度を更新（加速度を積分）
    velocity_.first += ax_world * dt;
    velocity_.second += ay_world * dt;

    // ワールド座標系での位置を更新（速度を積分）
    pose_.x += velocity_.first * dt;
    pose_.y += velocity_.second * dt;
}

std::pair<double, double> ImuOdometry::getVelocity() const {
    return velocity_;
}

Pose2D ImuOdometry::getPose() const {
    return pose_;
}

void ImuOdometry::setPose(const Pose2D& pose) {
    pose_ = pose;
    velocity_ = {0.0, 0.0};
}
