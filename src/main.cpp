#include <mbed.h>
#include "config.hpp"
#include "System/WheelController/OdomWheel.hpp"

int main()
{
    // エンコーダーの初期化
    Encoder encoder1(InterruptInPins::OMNI_ENCODER1_A, DigitalInPins::OMNI_ENCODER1_B);
    Encoder encoder2(InterruptInPins::OMNI_ENCODER2_A, DigitalInPins::OMNI_ENCODER2_B);
    Encoder encoder3(InterruptInPins::OMNI_ENCODER3_A, DigitalInPins::OMNI_ENCODER3_B);
    Encoder encoder4(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B);
    Encoder encoder5(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER1_B);
    std::array<Encoder *, 5> encoders = {&encoder1, &encoder2, &encoder3, &encoder4, &encoder5};

    // DCモーターの初期化
    // DCモーター4, 5は測定輪のためPIDGainを全て0にしており、PWM信号は送られない。
    DCMotor dc1(PwmOutPins::OMNI_MOTOR1_PWM, DigitalOutPins::OMNI_MOTOR1_DIR);
    DCMotor dc2(PwmOutPins::OMNI_MOTOR2_PWM, DigitalOutPins::OMNI_MOTOR2_DIR);
    DCMotor dc3(PwmOutPins::OMNI_MOTOR3_PWM, DigitalOutPins::OMNI_MOTOR3_DIR);
    DCMotor dc4(PwmOutPins::OMNI_DUMMY_PWM, DigitalOutPins::OMNI_DUMMY_DIR);
    DCMotor dc5(PwmOutPins::OMNI_DUMMY_PWM, DigitalOutPins::OMNI_DUMMY_DIR);

    // モーターコントローラの初期化
    // PIDGainの値は要検証
    MotorController motor1(dc1, encoder1, PIDGain({0.1, 0.01, 0.01, 100}));
    MotorController motor2(dc2, encoder2, PIDGain({0.1, 0.01, 0.01, 100}));
    MotorController motor3(dc3, encoder3, PIDGain({0.1, 0.01, 0.01, 100}));
    MotorController motor4(dc4, encoder4, PIDGain());
    MotorController motor5(dc5, encoder5, PIDGain());
    std::array<MotorController *, 5> motor_controller = {&motor1, &motor2, &motor3, &motor4, &motor5};

    // 用設定
    float WHEEL_RAD = 100.0;
    float TREAD_RAD = 100.0;
    std::array<WheelConfig, 5> config = {
        // 駆動輪3つ
        WheelConfig{
            .wheel_radius = WHEEL_RAD, // ホイール半径
            .wheel_x = 0.0,            // ホイールのx座標
            .wheel_y = TREAD_RAD,      // ホイールのy座標
            .wheel_theta = M_PI        // ホイールの向き
        },
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = -(float)M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = -(float)0.5 * TREAD_RAD,
            .wheel_theta = 5 * M_PI / 3},
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = +(float)M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = -(float)0.5 * TREAD_RAD,
            .wheel_theta = M_PI / 3},
        // 測定輪2つ
        WheelConfig{
            .wheel_radius = 0,
            .wheel_x = 0,
            .wheel_y = 0,
            .wheel_theta = 0},
        WheelConfig{
            .wheel_radius = 0,
            .wheel_x = 0,
            .wheel_y = 0,
            .wheel_theta = 0},
    };

    OdomWheel<5> odom_wheel(config, motor_controller, encoders);
}
