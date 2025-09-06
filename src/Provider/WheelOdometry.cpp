#include "Provider/WheelOdometry.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

WheelOdometry::WheelOdometry(Encoder* x_encoder, Encoder* y_encoder, float wheel_radius, int counts_per_revolution)
    : x_encoder_(x_encoder),
      y_encoder_(y_encoder),
      // 1カウントあたりの移動距離を先に計算しておく
      // 円周 / 1回転あたりのカウント数
      distance_per_count_( (2.0f * M_PI * wheel_radius) / static_cast<float>(counts_per_revolution) )
{
    // 最初のエンコーダ値を読み込み、前回値として保存
    prev_count_x_ = x_encoder_->getCount();
    prev_count_y_ = y_encoder_->getCount();

    // 内部タイマーを開始
    timer_.start();
}

void WheelOdometry::update() {
    // 前回updateが呼ばれてからの経過時間を取得 (秒)
    const float dt = timer_.read();
    timer_.reset();

    // ゼロ除算を防止
    if (dt < 1e-6) {
        return;
    }

    // 現在のエンコーダ値を取得
    const int current_count_x = x_encoder_->getCount();
    const int current_count_y = y_encoder_->getCount();

    // 前回からの差分を計算
    const int delta_count_x = current_count_x - prev_count_x_;
    const int delta_count_y = current_count_y - prev_count_y_;

    // 前回値を更新
    prev_count_x_ = current_count_x;
    prev_count_y_ = current_count_y;

    // 移動距離を計算 (m)
    const float distance_x = static_cast<float>(delta_count_x) * distance_per_count_;
    const float distance_y = static_cast<float>(delta_count_y) * distance_per_count_;

    // 前回の速度を保存
    const Velocity prev_velocity = current_velocity_;

    // 現在の速度を計算 (m/s)
    current_velocity_.vx = distance_x / dt;
    current_velocity_.vy = distance_y / dt;

    // 現在の加速度を計算 (m/s^2)
    current_acceleration_.ax = (current_velocity_.vx - prev_velocity.vx) / dt;
    current_acceleration_.ay = (current_velocity_.vy - prev_velocity.vy) / dt;
}

Velocity WheelOdometry::getVelocity() const {
    return current_velocity_;
}

Acceleration WheelOdometry::getAcceleration() const {
    return current_acceleration_;
}
