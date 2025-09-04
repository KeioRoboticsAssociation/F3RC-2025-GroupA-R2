#include <iostream>

void _main();
int main() { std::cin.tie(nullptr); std::ios::sync_with_stdio(false); _main(); }

#include "config.hpp"
#include "Parts/Imu.hpp"

Imu imu(PinsForSensor::IMU_SDA, PinsForSensor::IMU_SCL);
UnbufferedSerial pc(USBTX, USBRX);

void _main() {
    pc.baud(9600);

    if(!imu.init()) {
        printf("IMU initialization failed\n");
        return;
    }
    printf("IMU initialization complete\n");

    while(true) {
        double yaw = imu.getYaw();
        double pitch = imu.getPitch();
        double roll = imu.getRoll();
        auto angVel = imu.getAngularVelocity();
        auto linAcc = imu.getLinearAcceleration();
        bool calibrated = imu.isCalibrated();

        printf("                    Yaw: %7.2f, Pitch: %7.2f, Roll: %7.2f\n", yaw, pitch, roll);
        printf("   Angular Velocity - X: %7.2f,     Y: %7.2f,    Z: %7.2f\n", angVel.x, angVel.y, angVel.z);
        printf("Linear Acceleration - X: %7.2f,     Y: %7.2f,    Z: %7.2f\n", linAcc.x, linAcc.y, linAcc.z);
        printf("             Calibrated: %7s\n", calibrated ? "Yes" : "No");
        printf("------------------------------------------------------------\n");

        ThisThread::sleep_for(1000ms);
    }
}
