#include "Provider/Database.hpp"

Database::Database(Imu &imu, IRSensor ir_sensor, LimitSwitch front_limit_switch, LimitSwitch side_limit_switch)
    : imu(imu), ir_sensor(ir_sensor), front_limit_switch(front_limit_switch), side_limit_switch(side_limit_switch)
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
    return imu.getAngularVelocity().z;
}

bool Database::getIRSensorRaw()
{
    ir_sensor.isDetected();
}

bool Database::getFrontLimitSwitchStatus()
{
    return front_limit_switch.isPressed();
}

bool Database::getSideLimitSwitchStatus()
{
    return side_limit_switch.isPressed();
}
