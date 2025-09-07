#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "Encoder.hpp"
#include "Imu.hpp"
#include "RobotConfig.hpp"
#include "mbed.h"

class Odometry {
public:
    Odometry(Encoder& encoder_x, Encoder& encoder_y, Imu& imu);

    void update(float dt);

    void reset();

    float getX() const;
    float getY() const;
    float getThetaRad() const;
    float getThetaDeg() const;

    // ★追加: 平滑化された角速度を取得する関数
    float getFilteredAngularVelocity() const;

private:
    Encoder& m_encoder_x;
    Encoder& m_encoder_y;
    Imu& m_imu;

    // 自己位置
    float m_x;
    float m_y;
    float m_theta_rad;

    // 速度計算用の前回エンコーダ値
    int m_prev_count_x;
    int m_prev_count_y;

    // ★変更: IMU角速度計算用の履歴データ (循環バッファ)
    std::pair<float, uint32_t> m_yaw_history[RobotConfig::IMU_HISTORY_SIZE];
    int m_history_index;
    bool m_history_filled;
    float m_filtered_angular_velocity;
};

#endif // ODOMETRY_HPP
