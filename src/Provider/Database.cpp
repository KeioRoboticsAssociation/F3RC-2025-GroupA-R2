#include "Provider/Database.hpp"
#include <math.h>

Database::Database(Imu &imu, WheelOdometry &wheel_odom, TimeOfFlightSensor front_tof_sensor, TimeOfFlightSensor side_tof_sensor, LimitSwitch front_limit_switch, LimitSwitch side_limit_switch)
    : imu(imu), wheel_odom(wheel_odom), front_tof_sensor(front_tof_sensor), side_tof_sensor(side_tof_sensor), front_limit_switch(front_limit_switch), side_limit_switch(side_limit_switch)
{
    this->imu.init();
    front_limit_switch.init();
    side_limit_switch.init();
}

std::pair<double, double> Database::getAcceleration()
{
    auto acc = wheel_odom.getAcceleration();
    return {acc.ax, acc.ay};
}

double Database::getAngularVelocity()
{
    return imu.getAngularVelocity().z * 180. / M_PI;
}

bool Database::getFrontTofStatus()
{
    return front_tof_sensor.isDetecting();
}

bool Database::getSideTofStatus()
{
    return side_tof_sensor.isDetecting();
}

bool Database::getFrontLimitSwitchStatus()
{
    return front_limit_switch.isPressed();
}

bool Database::getSideLimitSwitchStatus()
{
    return side_limit_switch.isPressed();
}
