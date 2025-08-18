#ifndef IRSENSOR_HPP
#define IRSENSOR_HPP

#include <mbed.h>

class IRSensor {
    public:
        IRSensor(PinName out);
        void init();
        bool isDetecting();
    private:
        DigitalIn _sensorPin;
};

#endif /* IRSENSOR_HPP */
