//==================================================
// 測定輪から速度・加速度を算出するクラス
//==================================================
#pragma once
#include "Parts/Encoder.hpp"
#include <cmath>

struct Velocity { float vx; float vy; };
struct Acceleration { float ax; float ay; };

class WheelOdometry {
public:
    WheelOdometry(Encoder* x_encoder, Encoder* y_encoder, float wheel_radius, int resolution);

    Velocity calculateVelocity(float dt);
    Acceleration calculateAcceleration(float dt);

private:
    float countToRadians(int delta_count) const;

    Encoder* x_encoder;
    Encoder* y_encoder;
    float wheel_radius;
    int resolution;

    int prev_count_x;
    int prev_count_y;

    Velocity current_vel;
    Velocity prev_vel;
};
