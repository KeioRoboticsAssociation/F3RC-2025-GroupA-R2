#include "Parts/TimeOfFlightSensor.hpp"

TimeOfFlightSensor::TimeOfFlightSensor(PinName q1, PinName q2) : _sensorPin1(q1), _sensorPin2(q2) {}

void TimeOfFlightSensor::init() {
    // 特に処理は不要
}

bool TimeOfFlightSensor::isDetecting_1() {
    return _sensorPin1.read();
}

bool TimeOfFlightSensor::isDetecting_2() {
    return _sensorPin2.read();
}
