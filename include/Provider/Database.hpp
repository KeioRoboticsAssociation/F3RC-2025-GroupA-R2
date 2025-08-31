#pragma once
#include <utility>
#include "Parts/Imu.hpp"
#include "Parts/LimitSwitch.hpp"
#include "Parts/TimeOfFlightSensor.hpp"

class Database
{
public:
    Database(Imu &imu, TimeOfFlightSensor front_time_of_flight_sensor, TimeOfFlightSensor side_time_of_flight_sensor, LimitSwitch front_limit_switch, LimitSwitch side_limit_switch);
    // x, y方向の加速度[m/s^2]を取得する
    std::pair<double, double> getAcceleration();
    // 角速度[rad/s]を取得する
    double getAngularVelocity();
    // 赤外線センサー1の値を取得する
    bool getFrontTofStatus();
    // 赤外線センサー2の値を取得する
    bool getSideTofStatus();
    bool getFrontLimitSwitchStatus();
    bool getSideLimitSwitchStatus();

private:
    Imu &imu;
    TimeOfFlightSensor front_time_of_flight_sensor;
    TimeOfFlightSensor side_time_of_flight_sensor;
    LimitSwitch front_limit_switch;
    LimitSwitch side_limit_switch;
};
