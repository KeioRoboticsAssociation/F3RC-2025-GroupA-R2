#include <mbed.h>
#include "Driver/ServoMotor.hpp"
#include "System/WheelController/OdomWheel.hpp"
#include "config.hpp"

int main_5()
{
    // エンコーダーの初期化
    Encoder encoder1(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B, 2048, true);
    Encoder encoder2(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B, 2048, true);
    Encoder encoder3(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B, 2048, true);
    Encoder encoder4(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B);
    Encoder encoder5(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, true);
    std::array<Encoder *, 5> encoders = {&encoder1, &encoder2, &encoder3, &encoder4, &encoder5};

    // DCモーターの初期化
    DCMotor dc1(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
    DCMotor dc2(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    DCMotor dc3(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);
    DCMotor dc4(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);
    DCMotor dc5(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    // モーターコントローラの初期化
    MotorController motor1(dc1, encoder1, PIDGain({10, 0, 0, 100}));
    MotorController motor2(dc2, encoder2, PIDGain({10, 0, 0, 100}));
    MotorController motor3(dc3, encoder3, PIDGain({10, 0, 0, 100}));
    MotorController motor4(dc4, encoder4, PIDGain({0, 0, 0, 100}));
    MotorController motor5(dc5, encoder5, PIDGain({0, 0, 0, 100}));
    std::array<MotorController *, 5> motor_controller = {&motor1, &motor2, &motor3, &motor4, &motor5};

    // 足回り配置の設定　(ここでは駆動輪を測定輪として使用)
    float WHEEL_RAD = 30.0;
    float TREAD_RAD = 210.0;
    std::array<WheelConfig, 5> config = {
        WheelConfig{
            .wheel_radius = WHEEL_RAD, // ホイール半径
            .wheel_x = 0.0,            // ホイールのx座標
            .wheel_y = TREAD_RAD,      // ホイールのy座標
            .wheel_theta = M_PI        // ホイールの向き
        },
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)-M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = (float)-0.5 * TREAD_RAD,
            .wheel_theta = 5 * M_PI / 3},
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)+M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = (float)-0.5 * TREAD_RAD,
            .wheel_theta = M_PI / 3},
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = 0.0,
            .wheel_y = 0.0,
            .wheel_theta = M_PI / 2},
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = 50, // 感覚値
            .wheel_y = TREAD_RAD / 2,
            .wheel_theta = M_PI},
    };

    OdomWheel<5> odom_wheel(PIDGain{25, 0, 0, 100}, config, motor_controller, encoders);
    odom_wheel.setTargetPose(Pose{0.0, 10.0, 0.0});

    while (true)
    {
        odom_wheel.updateTargetTwist();
        printf("measuring_encoder_counts: %d, %d, %d, %d, %d\n", encoder1.getCount(), encoder2.getCount(), encoder3.getCount(), encoder4.getCount(), encoder5.getCount());
        printf("\n");
        wait_us(10000); // 10ms
    }
}

int main_3()
{
    Encoder encoder1(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B, 2048, true);
    Encoder encoder2(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B, 2048, true);
    Encoder encoder3(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B, 2048, true);
    std::array<Encoder *, 3> encoders = {&encoder1, &encoder2, &encoder3};

    // PI
    DCMotor dc1(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
    // 左後方
    DCMotor dc2(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    // 右後方
    DCMotor dc3(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    MotorController motor1(dc1, encoder1, PIDGain({10, 0, 0, 100}));
    MotorController motor2(dc2, encoder2, PIDGain({10, 0, 0, 100}));
    MotorController motor3(dc3, encoder3, PIDGain({10, 0, 0, 100}));
    std::array<MotorController *, 3> motor_controller = {&motor1, &motor2, &motor3};

    // 足回り配置の設定　(ここでは駆動輪を測定輪として使用)
    float WHEEL_RAD = 30.0;
    float TREAD_RAD = 210.0;
    std::array<WheelConfig, 3> config = {
        // PI
        WheelConfig{
            .wheel_radius = WHEEL_RAD, // ホイール半径
            .wheel_x = 0.0,            // ホイールのx座標
            .wheel_y = TREAD_RAD,      // ホイールのy座標
            .wheel_theta = M_PI        // ホイールの向き
        },
        // 左後方
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)-M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = (float)-0.5 * TREAD_RAD,
            .wheel_theta = 5 * M_PI / 3},
        // 右後方
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)+M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = (float)-0.5 * TREAD_RAD,
            .wheel_theta = M_PI / 3},
    };

    OdomWheel<3> odom_wheel(PIDGain{5, 0, 0, 100}, config, motor_controller, encoders);
    odom_wheel.setTargetPose(Pose{0.0, 1000.0, 0.0}); // 前方1m

    while (true)
    {
        odom_wheel.updateTargetTwist();
        printf("measuring_encoder_counts: %d, %d, %d\n", encoder1.getCount(), encoder2.getCount(), encoder3.getCount());
        printf("\n");
        wait_us(10000); // 10ms
    }
}

int main_2()
{
    // エンコーダーの初期化
    Encoder encoder4(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B);
    Encoder encoder5(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, 2048, true);

    while (true)
    {
        printf("measuring_encoder_counts: %d, %d\n", encoder4.getCount(), encoder5.getCount());
        wait_us(10000); // 10ms
    }
}

void encoder_test()
{
    Encoder encoder1(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B, 2048, true);
    Encoder encoder2(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B, 2048, true);
    Encoder encoder3(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B, 2048, true);
    std::array<Encoder *, 3> encoders = {&encoder1, &encoder2, &encoder3};

    // PI
    DCMotor dc1(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
    // 左後方
    DCMotor dc2(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    // 右後方
    DCMotor dc3(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    MotorController motor1(dc1, encoder1, PIDGain({10, 0, 0, 100}));
    MotorController motor2(dc2, encoder2, PIDGain({10, 0, 0, 100}));
    MotorController motor3(dc3, encoder3, PIDGain({10, 0, 0, 100}));
    std::array<MotorController *, 3> motor_controller = {&motor1, &motor2, &motor3};

    // 足回り配置の設定　(ここでは駆動輪を測定輪として使用)
    float WHEEL_RAD = 30.0;
    float TREAD_RAD = 210.0;
    std::array<WheelConfig, 3> config = {
        // PI
        WheelConfig{
            .wheel_radius = WHEEL_RAD, // ホイール半径
            .wheel_x = 0.0,            // ホイールのx座標
            .wheel_y = TREAD_RAD,      // ホイールのy座標
            .wheel_theta = M_PI        // ホイールの向き
        },
        // 左後方
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)-M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = -TREAD_RAD / 2,
            .wheel_theta = 5 * M_PI / 3},
        // 右後方
        WheelConfig{
            .wheel_radius = WHEEL_RAD,
            .wheel_x = (float)+M_SQRT3 / 2 * TREAD_RAD,
            .wheel_y = -TREAD_RAD / 2,
            .wheel_theta = M_PI / 3},
    };

    OdomWheel<3> odom_wheel(PIDGain{25, 0, 0, 100}, config, motor_controller, encoders);
    odom_wheel.setTargetPose(Pose{0.0, 10.0, 0.0});

    while (true)
    {
        encoder2.addCount(5);
        encoder3.addCount(5);
        auto pos = odom_wheel.odometry.getCurrentPose();
        printf("%f, %f, %f\n", pos.x, pos.y, pos.theta);
        printf("measuring_encoder_counts: %d, %d, %d\n", encoder1.getCount(), encoder2.getCount(), encoder3.getCount());
        wait_us(10000); // 10ms
    }
}

void main_without_encoder()
{
    DCMotor dc1(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
    DCMotor dc2(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    DCMotor dc3(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);
    dc2.setDuty(0.3);
    dc3.setDuty(0.3);
    wait_us(1000000); // 1s

    dc3.stop();
    wait_us(10000); // 10ms

    dc3.setDuty(0.3);
    wait_us(1000000);
}

int main()
{
    // main_5();
    main_3();
    // main_2();
    // encoder_test();
    // main_without_encoder();
}
