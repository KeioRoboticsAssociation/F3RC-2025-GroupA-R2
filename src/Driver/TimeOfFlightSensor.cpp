#include "Driver/TimeOfFlightSensor.hpp"

TimeOfFlightSensor::TimeOfFlightSensor(PinName pin) : _sensorPin(pin) {}

void TimeOfFlightSensor::init() {
    _sensorPin.mode(PullUp);
}

bool TimeOfFlightSensor::isDetecting() {
    return _sensorPin;
}
