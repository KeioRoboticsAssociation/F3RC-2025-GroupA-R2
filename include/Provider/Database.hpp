#pragma once
#include <utility>
#include "Driver/Encoder.hpp"
#include "Driver/Imu.hpp"
#include "Driver/IrSensor.hpp"
#include "Driver/LimitSwitch.hpp"

class Database
{
public:
    Database(Imu &imu, IRSensor ir_sensor, LimitSwitch front_limit_switch, LimitSwitch side_limit_switch);
    // x, y方向の加速度[m/s^2]を取得する
    std::pair<double, double> getAcceleration();
    // 角速度[deg/s]を取得する
    double getAngularVelocity();
    // 赤外線センサーの値を取得する
    bool getIRSensorRaw();
    bool getFrontLimitSwitchStatus();
    bool getSideLimitSwitchStatus();

private:
    Imu &imu;
    IRSensor ir_sensor;
    LimitSwitch front_limit_switch;
    LimitSwitch side_limit_switch;
};
