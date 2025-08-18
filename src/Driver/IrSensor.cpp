#include "Driver/IrSensor.hpp"

IRSensor::IRSensor(PinName out) : _sensorPin(out) {}

void IRSensor::init() {
    _sensorPin.mode(PullUp);
}

bool IRSensor::isDetecting() {
    return _sensorPin;
}
