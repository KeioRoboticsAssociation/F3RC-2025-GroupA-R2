#include "Odometry.hpp"
#include "RobotConfig.hpp"
#include <cmath>

Odometry::Odometry(Encoder& encoder_x, Encoder& encoder_y, Imu& imu)
    : m_encoder_x(encoder_x), m_encoder_y(encoder_y), m_imu(imu)
{
    reset();
}

void Odometry::update(float dt) {
    if (dt <= 0) return;

    // === 1. 測定輪エンコーダから移動距離(dx, dy)を計算 ===
    int current_count_x = m_encoder_x.getCount();
    int delta_count_x = current_count_x - m_prev_count_x;
    m_prev_count_x = current_count_x;
    float dx = (float)delta_count_x / RobotConfig::ODOM_COUNTS_PER_REV * (2.0f * RobotConfig::PI * RobotConfig::ODOM_WHEEL_RADIUS_M);

    int current_count_y = m_encoder_y.getCount();
    int delta_count_y = current_count_y - m_prev_count_y;
    m_prev_count_y = current_count_y;
    float dy = (float)delta_count_y / RobotConfig::ODOM_COUNTS_PER_REV * (2.0f * RobotConfig::PI * RobotConfig::ODOM_WHEEL_RADIUS_M);


    // === 2. IMUからヨー角を取得し、履歴を更新 ===
    double yaw_deg = m_imu.getYaw();
    m_theta_rad = yaw_deg * RobotConfig::PI / 180.0f;
    auto current_time = Kernel::Clock::now().time_since_epoch().count();

    m_yaw_history[m_history_index] = {m_theta_rad, current_time};
    
    int oldest_index = (m_history_index + 1) % RobotConfig::IMU_HISTORY_SIZE;
    if (!m_history_filled && oldest_index == 0) {
        m_history_filled = true;
    }

    // === 3. ヨー角の履歴から安定した角速度を計算 ===
    if (m_history_filled) {
        const auto& newest_data = m_yaw_history[m_history_index];
        const auto& oldest_data = m_yaw_history[oldest_index];

        float delta_yaw = newest_data.first - oldest_data.first;
        uint32_t delta_tick = newest_data.second - oldest_data.second;
        
        // 角度の-PI~PIの回り込みを補正
        if (delta_yaw > RobotConfig::PI) delta_yaw -= 2.0f * RobotConfig::PI;
        if (delta_yaw < -RobotConfig::PI) delta_yaw += 2.0f * RobotConfig::PI;

        float delta_t_sec = (float)delta_tick / (float)OS_TICK_FREQ;

        if (delta_t_sec > 1e-6) { // ゼロ除算防止
            m_filtered_angular_velocity = delta_yaw / delta_t_sec;
        }
    }
    
    m_history_index = oldest_index;

    // === 4. ワールド座標系での位置を更新 ===
    float dX = dx * cos(m_theta_rad) - dy * sin(m_theta_rad);
    float dY = dx * sin(m_theta_rad) + dy * cos(m_theta_rad);
    
    m_x += dX;
    m_y += dY;
}

void Odometry::reset() {
    m_x = 0.0f;
    m_y = 0.0f;
    m_theta_rad = 0.0f;
    
    m_imu.resetYaw();

    m_encoder_x.reset();
    m_encoder_y.reset();
    m_prev_count_x = m_encoder_x.getCount();
    m_prev_count_y = m_encoder_y.getCount();
    
    m_history_index = 0;
    m_history_filled = false;
    m_filtered_angular_velocity = 0.0f;
    // 履歴バッファを初期化
    for(int i = 0; i < RobotConfig::IMU_HISTORY_SIZE; ++i) {
        m_yaw_history[i] = {0.0f, 0};
    }
}

float Odometry::getX() const { return m_x; }
float Odometry::getY() const { return m_y; }
float Odometry::getThetaRad() const { return m_theta_rad; }
float Odometry::getThetaDeg() const { return m_theta_rad * 180.0f / RobotConfig::PI; }
float Odometry::getFilteredAngularVelocity() const { return m_filtered_angular_velocity; }
