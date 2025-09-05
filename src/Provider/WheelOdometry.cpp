// Odometry.cpp
#include "Provider/WheelOdometry.hpp"
#include <cstdio>

WheelOdometry::WheelOdometry(Encoder* x_enc, Encoder* y_enc, float r, int res)
    : x_encoder(x_enc), y_encoder(y_enc), wheel_radius(r), resolution(res),
      prev_count_x(0), prev_count_y(0), current_vel{0,0}, prev_vel{0,0}
{
    //printf("WheelOdometry initialized\n");
}

Velocity WheelOdometry::calculateVelocity(float dt) {


    int count_x_now = x_encoder->getCount();
    int count_y_now = y_encoder->getCount();
    // printf("count_x_now: %d\n", count_x_now);
    // printf("count_y_now: %d\n", count_y_now);


    int delta_x = count_x_now - prev_count_x;
    int delta_y = count_y_now - prev_count_y;
    // printf("int delta_x: %d = count_x_now: %d - prev_count_x: %d\n", delta_x,count_x_now, prev_count_x);
    // printf("int delta_y: %d = count_y_now: %d - prev_count_y: %d\n", delta_y,count_y_now, prev_count_y);
    

    prev_count_x = count_x_now;
    prev_count_y = count_y_now;

    float delta_rad_x = countToRadians(delta_x);
    float delta_rad_y = countToRadians(delta_y);
    // printf("delta_rad_x : %f\n", delta_rad_x);
    // printf("delta_rad_y : %f\n", delta_rad_y);

    float omega_x = delta_rad_x / dt;
    float omega_y = delta_rad_y / dt;
    // printf("omega_x: %f\n", omega_x);
    // printf("omega_y: %f\n", omega_y);

    current_vel.vx = omega_x * wheel_radius;
    current_vel.vy = omega_y * wheel_radius;

    return current_vel;
}

Acceleration WheelOdometry::calculateAcceleration(float dt) {
    Acceleration acc;
    acc.ax = (current_vel.vx - prev_vel.vx) / dt;
    acc.ay = (current_vel.vy - prev_vel.vy) / dt;
    // printf("acc.ax: %f\n", acc.ax);
    // printf("acc.ay: %f\n", acc.ay);
    prev_vel = current_vel;
    return acc;
}

float WheelOdometry::countToRadians(int delta_count) const {
    return static_cast<float>(delta_count) * (2.0f * M_PI / resolution);
}
