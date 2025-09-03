#include "mbed.h"
#include "Control/BehaviorController.hpp"   
#include "Provider/Wheel.hpp"               
#include "Mechanism/MotorController.hpp"       //検証済み
#include "Mechanism/WheelController.hpp"
#include <iostream>

DCMotor FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
Encoder FrontEncoder(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B);
DCMotor RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
Encoder RearLeftEncoder(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B);
DCMotor RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);
Encoder RearRightEncoder(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B);

MotorController mc(FrontMotor, FrontEncoder);
MotorController mc2(RearLeftMotor, RearLeftEncoder);
MotorController mc3(RearRightMotor, RearRightEncoder);

// オムニホイールの配置を設定
float WHEEL_RAD = 30.0; // 車輪の半径
float TREAD_RAD = 230.0; // 中心から車輪までの距離


std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RAD, // 車輪の半径
        .wheel_x = 160.0, // 車輪のx座標
        .wheel_y = TREAD_RAD,  // 車輪のy座標
        .wheel_theta = M_PI // 車輪の角度
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = - M_SQRT3 / 2 * TREAD_RAD,
        .wheel_y = - 0.5 * TREAD_RAD, 
        .wheel_theta = M_PI / 3
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD, 
        .wheel_x = + M_SQRT3 / 2 * TREAD_RAD, 
        .wheel_y = - 0.5 * TREAD_RAD,
        .wheel_theta = 5 * M_PI / 3
    }
};

// オドメトリとホイールコントローラの設定
//Odometry<3> odometry(config, {&encoder1, &encoder2, &encoder3});
WheelController<3> controller(config, {&mc, &mc2, &mc3});

int main() {
    Twist tw;
    tw.vx = 0.0f;
    tw.vy = 100.0f;
    tw.omega = 0.0f;
    while(true){
        controller.setTargetTwist(tw);
        //std::cout << controller.twistToMotorSpeeds(tw) << std::endl;
        printf("twistToMotorSpeeds[0]:%f\n", controller.twistToMotorSpeeds(tw)[0]);
        printf("twistToMotorSpeeds[1]:%f\n", controller.twistToMotorSpeeds(tw)[1]);
        printf("twistToMotorSpeeds[2]:%f\n", controller.twistToMotorSpeeds(tw)[2]);
    }

    return 0;
}
    

    
//     // RearRightMotor.setDuty(1.0f);
//     // wait_us(6000000);
//     // RearRightMotor.setDuty(0.0f);


//     mc.setTargetSpeed(1.0f);
//     mc2.setTargetSpeed(1.0f);
//     mc3.setTargetSpeed(1.0f);
//     printf("started\n");

//     while (true) {
//         // ループ呼び出し確認
//         if (mc.isLoopCalled()) {
//             // printf("LOOP!!!\n");
//             mc.clearLoopFlag();
//         }

//         mc3.print();

//         ThisThread::sleep_for(50ms); // 適度に休ませる

//     }

//     return 0;
// }


// #include "mbed.h"
// #include "Control/BehaviorController.hpp"
// #include "Mechanism/MotorController.hpp"
// #include <iostream>

//int main() {
    // BehaviorController BC;
    // BC.setPosition(0.0,0.0);
    // BC.setAngle(0.0);
    // BC.setTargetPosition(0.0, 2.0);
    // BC.setTargetAngle(0.0);

    // //std::cout << BC.calculateTargetVelocity() << std::endl;
    // BC.move();

//     printf("s\n");
//     DCMotor FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
//     // DCMotor RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
//     // DCMotor RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

//     Encoder FrontEncoder(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B);
//     MotorController mc(FrontMotor, FrontEncoder);

//     mc.setTargetSpeed(1.0f);
//     printf("started\n");
//     while(true){
//         // std::cout << FrontEncoder.getCount() << std::endl;
//         mc.print();
//     }


    
    // // FrontMotor.setDuty(1.0f);
    // RearLeftMotor.setDuty(1.0f);
    // printf("1.0\n");
    // RearRightMotor.setDuty(-1.0f);
    // printf("-1.0\n");
    // wait_us(4000000);

    // // FrontMotor.setDuty(0.0f);
    // RearLeftMotor.setDuty(0.0f);
    // printf("0.0\n");
    // RearRightMotor.setDuty(0.0f);
    // printf("0.0\n");
    // wait_us(4000000);



        


//     // 1. BehaviorController の生成（デフォルト PID ゲイン・Wheel・Motor）
//     BehaviorController behavior;

//     // 2. 目標座標・速度の設定
//     behavior.setTargetPosition(0, 2.0);       // 位置制御の目標
//     //behavior.setTargetAngle(3.14 / 2);          // 90度

//     // 3. 初期状態設定（必要なら）
//     behavior.setPosition(0.0, 0.0);
//     behavior.setAngle(0.0);

//     // 4. ループで動作
//     while (true) {
//         // PID計算してモーターに指示
//         behavior.move();

//         // 状態を更新する場合はここで setPosition 等を呼ぶ
//         // 例: センサーから現在位置を取得して
//         // behavior.setPosition(current_x, current_y);

//         // ThisThread::sleep_for(100ms);  // 10Hz 制御周期
//         wait_us(5000);
//         behavior.stop();
//     }



    

//    return 0;
//}
