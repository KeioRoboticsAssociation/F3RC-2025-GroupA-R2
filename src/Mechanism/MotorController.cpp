#include "Mechanism/MotorController.hpp"

MotorController::MotorController(DCMotor &motor, Encoder &encoder, PIDGain pid_gain, float max_duty)
    : motor(motor), encoder(encoder), pid_controller(pid_gain)
{
    using namespace std::chrono;

    // 周期を μs に変換（frequency [Hz] → 周期 [us]）
    auto period_us = microseconds(
        static_cast<int>(1'000'000 / pid_controller.getFrequency())
    );

    // コンストラクタで attach → loop が常に動作
    ticker.attach(callback(this, &MotorController::loop), period_us);

    this->max_duty = max_duty; // 最大デューティ比を設定
    printf("succeed in constructor!!!!!!\n");
}

void MotorController::loop()
{
    // ISR 内なので printf は禁止
    loop_called = true; // main 側に通知

    if (!moving) {
        return;
    }

    // 速度の更新
    float current_position = encoder.getRotations();                                  // 現在の位置を取得
    current_rps = (current_position - last_position) * pid_controller.getFrequency(); // 現在の速度を取得
    last_position = current_position;

    // PID制御
    float output = pid_controller.calculate(target_rps - current_rps);
    float duty = last_duty + output / pid_controller.getFrequency();

    // デバッグ用値
    output_ = output;
    encoder_ = current_position;
    current_rps_ = current_rps;

    // デューティ比の制限
    if (duty > max_duty) {
        duty = max_duty;
        pid_controller.reset(); // 最大速度に達したら積分をリセット
    }
    else if (duty < -max_duty) {
        duty = -max_duty;
        pid_controller.reset(); // 最小速度に達したら積分をリセット
    }

    last_duty = duty;

    // モーターへの出力
    motor.setDuty(duty);
}

void MotorController::setTargetSpeed(float target_rps)
{
    moving = true;
    this->target_rps = target_rps;
}

void MotorController::stop()
{
    target_rps = 0;
    pid_controller.reset();
    moving = false;
    motor.setDuty(0);
}

float MotorController::getSpeed()
{
    return current_rps;
}

float MotorController::getTargetSpeed()
{
    return target_rps;
}

void MotorController::print()
{
    printf("started in class\n");
    // printf("output: %f\n", output_ * 1000.0f);
    // printf("encoder: %f\n", encoder_);
    // printf("current_rps: %f\n", current_rps_);
}
