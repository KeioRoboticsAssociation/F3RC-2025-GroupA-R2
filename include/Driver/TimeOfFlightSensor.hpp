#ifndef TIMEOFFLIGHTSENSOR_HPP
#define TIMEOFFLIGHTSENSOR_HPP

#include <mbed.h>

class TimeOfFlightSensor {
    public:
        TimeOfFlightSensor(PinName out);
        void init();
        bool isDetecting();
    private:
        DigitalIn _sensorPin;
};

#endif /* TIMEOFFLIGHTSENSOR_HPP */
