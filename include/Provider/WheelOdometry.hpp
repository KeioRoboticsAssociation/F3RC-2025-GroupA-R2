//==================================================
// 測定輪から速度・加速度を算出するクラス
//==================================================
#pragma once
#include <Encoder.hpp>
#include "mbed.h" // Timerのために追加

// 速度と加速度を格納する構造体
struct Velocity { float vx = 0.0f; float vy = 0.0f; };
struct Acceleration { float ax = 0.0f; float ay = 0.0f; };

class WheelOdometry {
public:
    /**
     * @brief コンストラクタ
     * @param x_encoder X軸方向のエンコーダへのポインタ
     * @param y_encoder Y軸方向のエンコーダへのポインタ
     * @param wheel_radius 測定輪の半径 (m)
     * @param counts_per_revolution エンコーダの1回転あたりのカウント数 (CPR)。4逓倍した値を指定してください。
     */
    WheelOdometry(Encoder* x_encoder, Encoder* y_encoder, float wheel_radius, int counts_per_revolution);

    /**
     * @brief 速度と加速度を更新します。この関数を定期的に呼び出してください。
     */
    void update();

    /**
     * @brief 現在の速度を取得します。
     * @return Velocity構造体 {vx, vy} (m/s)
     */
    Velocity getVelocity() const;

    /**
     * @brief 現在の加速度を取得します。
     * @return Acceleration構造体 {ax, ay} (m/s^2)
     */
    Acceleration getAcceleration() const;

private:
    Encoder* x_encoder_;
    Encoder* y_encoder_;
    
    // 物理パラメータ
    const float distance_per_count_; // 1カウントあたりの移動距離 (m)

    // 内部状態
    int prev_count_x_;
    int prev_count_y_;
    
    Velocity current_velocity_;
    Acceleration current_acceleration_;
    
    // 時間計測用
    Timer timer_;
};
