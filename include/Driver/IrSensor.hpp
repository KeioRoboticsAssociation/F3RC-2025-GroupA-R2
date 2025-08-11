#ifndef IRSENSOR_HPP
#define IRSENSOR_HPP

#include <mbed.h>

class IRSensor {
    public:
        IRSensor(PinName pin);
        bool isDetected(); // WTT12L-B1531 が物体検知型のため
    private:
        DigitalIn _sensorPin;
};

#endif /* IRSENSOR_HPP */
