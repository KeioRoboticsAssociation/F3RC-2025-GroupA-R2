#include "TimeOfFlightSensor.hpp"

// 今回は閾値を一つしか用いない
TimeOfFlightSensor::TimeOfFlightSensor(PinName q1) : _sensorPin(q1) {}

void TimeOfFlightSensor::init() {
    // 特に処理は不要
}

bool TimeOfFlightSensor::isDetecting() {
    return _sensorPin.read();
}
