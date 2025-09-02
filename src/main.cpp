#include "mbed.h"
#include "Control/BehaviorController.hpp"

int main() {
    DCMotor FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
    DCMotor RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    DCMotor RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    
    // FrontMotor.setDuty(1.0f);
    RearLeftMotor.setDuty(1.0f);
    printf("1.0\n");
    RearRightMotor.setDuty(-1.0f);
    printf("-1.0\n");
    wait_us(4000000);

    // FrontMotor.setDuty(0.0f);
    RearLeftMotor.setDuty(0.0f);
    printf("0.0\n");
    RearRightMotor.setDuty(0.0f);
    printf("0.0\n");
    wait_us(4000000);

        


    // // 1. BehaviorController の生成（デフォルト PID ゲイン・Wheel・Motor）
    // BehaviorController behavior;

    // // 2. 目標座標・速度の設定
    // behavior.setTargetPosition(0, 2.0);       // 位置制御の目標
    // //behavior.setTargetAngle(3.14 / 2);          // 90度

    // // 3. 初期状態設定（必要なら）
    // behavior.setPosition(0.0, 0.0);
    // behavior.setAngle(0.0);

    // // 4. ループで動作
    // while (true) {
    //     // PID計算してモーターに指示
    //     behavior.move();

    //     // 状態を更新する場合はここで setPosition 等を呼ぶ
    //     // 例: センサーから現在位置を取得して
    //     // behavior.setPosition(current_x, current_y);

    //     // ThisThread::sleep_for(100ms);  // 10Hz 制御周期
    //     wait_us(5000);
    //     behavior.stop();
    // }

    return 0;
}
