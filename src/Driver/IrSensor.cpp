#include "Driver/IrSensor.hpp"

IRSensor::IRSensor(PinName pin) : _sensorPin(pin) {}

bool IRSensor::isDetected() {
    return _sensorPin.read();
}
