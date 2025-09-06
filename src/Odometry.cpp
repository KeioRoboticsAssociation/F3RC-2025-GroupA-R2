#include "Odometry.hpp"
#include "RobotConfig.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Odometry::Odometry(Encoder& encoder_x, Encoder& encoder_y, Imu& imu)
    : m_encoder_x(encoder_x), m_encoder_y(encoder_y), m_imu(imu) 
{
    // パラメータを設定
    if (RobotConfig::COUNTS_PER_REV > 0) {
        float circumference = RobotConfig::WHEEL_RADIUS * 2 * M_PI;
        m_meters_per_count = circumference / RobotConfig::COUNTS_PER_REV;
    } else {
        m_meters_per_count = 0.0f;
    }

    // 状態を初期化
    reset();
}

void Odometry::update() {
    // 1. センサーから現在の値を取得
    long current_count_x = m_encoder_x.getCount();
    long current_count_y = m_encoder_y.getCount();
    // IMUのYawをラジアンに変換して取得
    m_theta = m_imu.getYaw() * (M_PI / 180.0f);

    // 2. 前回からのエンコーダの変化量を計算
    long delta_count_x = current_count_x - m_prev_count_x;
    long delta_count_y = current_count_y - m_prev_count_y;

    // 3. ロボット座標系での移動量 [m] を計算
    // ※エンコーダの回転方向によって符号が逆になる場合は、-1を掛けて調整してください
    float dx = (float)delta_count_x * m_meters_per_count;
    float dy = (float)delta_count_y * m_meters_per_count;

    // 4. 座標変換：ロボット座標系の移動量をワールド座標系の移動量に変換
    float dX = dx * cos(m_theta) - dy * sin(m_theta);
    float dY = dx * sin(m_theta) + dy * cos(m_theta);

    // 5. ワールド座標を更新
    m_x += dX;
    m_y += dY;

    // 6. 次回計算のために現在のエンコーダ値を保存
    m_prev_count_x = current_count_x;
    m_prev_count_y = current_count_y;
}

void Odometry::reset(float x, float y, float theta_rad) {
    m_x = x;
    m_y = y;
    m_theta = theta_rad;
    m_imu.resetYaw(); // IMUのヨー角もリセット

    // エンコーダの現在値を初期値として設定
    m_prev_count_x = m_encoder_x.getCount();
    m_prev_count_y = m_encoder_y.getCount();
}

float Odometry::getThetaDeg() const {
    return m_theta * (180.0f / M_PI);
}
