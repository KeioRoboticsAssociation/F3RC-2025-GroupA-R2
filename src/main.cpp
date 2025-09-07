#include <mbed.h>
#include "Driver/DCMotor.hpp"
#include "config.hpp"

int main()
{
    DCMotor dc2(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    DCMotor dc3(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    dc2.setDuty(0.5);
    dc3.setDuty(-0.5);

    wait_us(9000000); // 10s

    dc2.stop();
    dc3.stop();
}
