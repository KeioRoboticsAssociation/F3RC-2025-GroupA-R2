#include "Provider/Database.hpp"
#include <math.h>

Database::Database(Imu &imu, TimeOfFlightSensor front_time_of_flight_sensor, TimeOfFlightSensor side_time_of_flight_sensor, LimitSwitch front_limit_switch, LimitSwitch side_limit_switch)
    : imu(imu), front_time_of_flight_sensor(front_time_of_flight_sensor), side_time_of_flight_sensor(side_time_of_flight_sensor), front_limit_switch(front_limit_switch), side_limit_switch(side_limit_switch)
{
    this->imu.init();
    front_limit_switch.init();
    side_limit_switch.init();
}

std::pair<double, double> Database::getAcceleration()
{
    double x = imu.getLinearAcceleration().x;
    double y = imu.getLinearAcceleration().y;
    return {x, y};
}

double Database::getAngularVelocity()
{
    return imu.getAngularVelocity().z * 180. / M_PI;
}

bool Database::getFrontTofStatus()
{
    return front_time_of_flight_sensor.isDetecting_1();
}

bool Database::getSideTofStatus()
{
    return front_time_of_flight_sensor.isDetecting_1();
}

bool Database::getFrontLimitSwitchStatus()
{
    return front_limit_switch.isPressed();
}

bool Database::getSideLimitSwitchStatus()
{
    return side_limit_switch.isPressed();
}
