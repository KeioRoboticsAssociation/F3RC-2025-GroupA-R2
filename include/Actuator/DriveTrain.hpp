#pragma once
#include "System/WheelController/WheelController.hpp"

template <int N>
class DriveTrain
{
public:
    DriveTrain(WheelController<N> wheel_controller);
    void setTargetTwist(Twist twist);

private:
    WheelController<N> wheel_controller;
};
