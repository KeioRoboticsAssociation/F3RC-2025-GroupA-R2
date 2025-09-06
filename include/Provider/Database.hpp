#pragma once
#include <utility>
#include "Parts/Imu.hpp"
#include "Parts/LimitSwitch.hpp"
#include "Parts/TimeOfFlightSensor.hpp"
#include "Parts/Encoder.hpp"
#include "Provider/WheelOdometry.hpp"

class Database
{
public:
    Database(Imu &imu, WheelOdometry &wheel_odom, TimeOfFlightSensor front_tof_sensor, TimeOfFlightSensor side_tof_sensor);
    // 加速度センサー由来のx, y方向の加速度[m/s^2]を取得する
    std::pair<double, double> getAcceleration();
    // 加速度センサー由来の角速度[rad/s]を取得する
    double getAngularVelocity();
    // 赤外線センサー1の値を取得する
    bool getFrontTofStatus();
    // 赤外線センサー2の値を取得する
    bool getSideTofStatus();

private:
    Imu &imu;
    WheelOdometry &wheel_odom;
    TimeOfFlightSensor front_tof_sensor;
    TimeOfFlightSensor side_tof_sensor;
};
