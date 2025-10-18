#include <mbed.h>
#include "Driver/ServoMotor.hpp"
#include "System/WheelController/OdomWheel.hpp"
#include "config.hpp"

int main()
{
    constexpr int frequency = 100;
    constexpr chrono::microseconds wait_time = 1s / frequency;

    // 足回り配置の設定　(ここでは駆動輪を測定輪として使用)
    constexpr float WHEEL_RAD = 30.0f;
    constexpr float TREAD_RAD = 210.0f;
    std::array<WheelConfig, 3> config = {
        // 前方
        WheelConfig{
            .wheel_radius = WHEEL_RAD, // ホイール半径
            .wheel_x = 0.0f,           // ホイールのx座標
            .wheel_y = TREAD_RAD,      // ホイールのy座標
            .wheel_theta = 0.0f        // ホイールの向き
        },
        // 左後方
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)-M_SQRT3 / 2.0f * TREAD_RAD,
            .wheel_y = -TREAD_RAD / 2.0f,
            .wheel_theta = M_PI * 2.0f / 3.0f,
        },
        // 右後方
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)+M_SQRT3 / 2.0f * TREAD_RAD,
            .wheel_y = -TREAD_RAD / 2.0f,
            .wheel_theta = M_PI * 4.0f / 3.0f,
        },
    };

    Encoder encoder1(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B, 2048, true);
    Encoder encoder2(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B, 2048, true);
    Encoder encoder3(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B);
    std::array<Encoder *, 3> encoders = {&encoder1, &encoder2, &encoder3};

    // 前方
    DCMotor dc1(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR, true);
    // 左後方
    DCMotor dc2(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    // 右後方
    DCMotor dc3(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    PIDGain motor_pid_gain = PIDGain({0.1f, 0.0f, 0.0f, frequency});
    PIDGain odom_wheel_pid_gain = PIDGain({1.0f, 0.0f, 0.0f, frequency});

    MotorController motor1(dc1, encoder1, motor_pid_gain);
    MotorController motor2(dc2, encoder2, motor_pid_gain);
    MotorController motor3(dc3, encoder3, motor_pid_gain);
    std::array<MotorController *, 3> motor_controller = {&motor1, &motor2, &motor3};

    OdomWheel<3> odom_wheel(odom_wheel_pid_gain, config, motor_controller, encoders);

    odom_wheel.setTargetPose(Pose{0.0f, 7000.0f, 0.0f}); // 単位: (mm, mm, rad)

    while (true)
    {
        printf("   encoder_counts: %8d, %8d, %8d\n", encoder1.getCount(), encoder2.getCount(), encoder3.getCount());

        odom_wheel.updateTargetTwist();

        printf("             duty: %8.3f, %8.3f, %8.3f\n", dc1.getDuty(), dc2.getDuty(), dc3.getDuty());
        printf("\n");

        wait_us(wait_time.count());
    }
}
