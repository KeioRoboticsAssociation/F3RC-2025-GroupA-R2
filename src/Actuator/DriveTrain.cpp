#include "Actuator/DriveTrain.hpp"

template <int N>
DriveTrain<N>::DriveTrain(WheelController<N> wheel_controller) : wheel_controller(wheel_controller) {}

template <int N>
void DriveTrain<N>::setTargetTwist(Twist twist)
{
    wheel_controller.setTargetTwist(twist);
}
