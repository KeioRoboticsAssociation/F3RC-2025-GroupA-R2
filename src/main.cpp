// // //====================================================
// // //  本環境用Main関数
// // //====================================================
// #include "Provider/Database.hpp"
// #include "Provider/WheelOdometry.hpp"
// #include "Provider/ImuOdometry.hpp"
// #include "Control/StateEstimator.hpp"
// #include "Mechanism/PIDController.hpp"
// #include "Control/BehaviorController.hpp"
// #include "Control/SectionManager.hpp"
// #include "Control/Section/Section1.hpp"

// int main() {
//     Database db;
//     WheelOdometry wheel_odom;
//     ImuOdometry imu_odom;
//     StateEstimator state(db, wheel_odom, imu_odom);

//     PIDGain x_pid, y_pid, ang_pid;
//     BehaviorController behavior(x_pid, y_pid, ang_pid);

//     // セクションを登録
//     SectionManager manager({
//         std::make_unique<Section1>(),
//         // std::make_unique<Section2>(),
//         // ...
//     });

//     while (!manager.allFinished()) {
//         double dt = getDt();

//         db.update();
//         state.update(dt);

//         // 各セクションに処理を委譲
//         manager.update(state, behavior, dt);

//         // モータ駆動
//         Twist target = behavior.calculateTargetVelocity();
//         behavior.setMotor();

//         wait(dt);
//     }
// }


// //====================================================
// //     WheelOdem検証用
// //====================================================

// #include <mbed.h>
// #include "Provider/WheelOdometry.hpp"
// #include "Parts/Encoder.hpp" // 実際のエンコーダークラスのヘッダ
// #include "config.hpp"

// // 必要な構造体（未定義ならここで定義）
// struct Velocity {
//     float vx;
//     float vy;
// };

// struct Acceleration {
//     float ax;
//     float ay;
// };

// // UART出力設定（必要に応じて）
// // UnbufferedSerial pc(USBTX, USBRX, 115200); // TX, RX, ボーレート

// void print(const char* format, ...) {
//     char buf[128];
//     va_list args;
//     va_start(args, format);
//     vsnprintf(buf, sizeof(buf), format, args);
//     va_end(args);
//     pc.write(buf, strlen(buf));
// }

// int main() {
//     // 実機のエンコーダー設定
//     // 例：PA_7 = A相, PA_6 = B相 のように
//     Encoder x_encoder(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B);
//     Encoder y_encoder(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B);


//     float wheel_radius = 0.03f;   // 直径6cmのホイール
//     int resolution = 1024;        // エンコーダの分解能（パルス数）

//     WheelOdometry odom(&x_encoder, &y_encoder, wheel_radius, resolution);

//     const float dt = 0.1f; // 秒（100ms）

//     while (true) {
//         Velocity vel = odom.calculateVelocity(dt);
//         Acceleration acc = odom.calculateAcceleration(dt);

//         print("Vel(vx=%.3f, vy=%.3f) [m/s], Acc(ax=%.3f, ay=%.3f) [m/s^2]\r\n",
//               vel.vx, vel.vy, acc.ax, acc.ay);

//         ThisThread::sleep_for(100ms);
//     }
// }

// //-------------------------------------------
// // 出力例
// // WheelOdometry initialized
// // Step  0: Vel(vx=3.68, vy=1.84), Acc(ax=36.84, ay=18.42)
// // Step  1: Vel(vx=3.68, vy=1.84), Acc(ax=0.00, ay=0.00)
// // Step  2: Vel(vx=3.68, vy=1.84), Acc(ax=0.00, ay=0.00)
// //-------------------------------------------


// //====================================================
// //     ImuOdem検証用
// //====================================================
// #include <cstdio>
// #include <chrono>
// #include <thread>
// #include "Provider/Database.hpp"
// #include "Provider/ImuOdometry.hpp"
// #include "config.hpp"

// int main() {
//     Imu imu(PinsForSensor::IMU_TX, PinsForSensor::IMU_RX);
//     TimeOfFlightSensor frontTof(PinsForSensor::TOF1);
//     TimeOfFlightSensor sideTof(PinsForSensor::TOF2);
//     LimitSwitch frontSwitch(LimitSwitchPins::FRONT_LIMIT);
//     LimitSwitch sideSwitch(LimitSwitchPins::SIDE_LIMIT);


//     Database db(imu, frontTof, sideTof, frontSwitch, sideSwitch);
//     ImuOdometry odom(db);

//     const double dt = 0.1; // 100ms周期
//     const int steps = 50;  // 5秒分更新

//     for (int i = 0; i < steps; ++i) {
//         odom.update(dt);

//         auto [vx, vy] = odom.getVelocity();
//         Pose2D pose = odom.getPose();

//         printf("Step %2d: Pos(x=%.2f, y=%.2f, θ=%.2f deg), Vel(vx=%.2f, vy=%.2f)\n",
//                i,
//                pose.x, pose.y, pose.theta * 180.0 / M_PI,
//                vx, vy);

//         //std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         wait_us(100000);
//     }

//     return 0;
// }



// //====================================================
// //     DataBase検証用
// //====================================================


// #include "Provider/Database.hpp"
// #include "config.hpp"

// int main() {
//     Imu imu(PinsForSensor::IMU_TX, PinsForSensor::IMU_RX);
//     TimeOfFlightSensor frontTof(PinsForSensor::TOF1);
//     TimeOfFlightSensor sideTof(PinsForSensor::TOF2);
//     LimitSwitch frontSwitch(LimitSwitchPins::FRONT_LIMIT);
//     LimitSwitch sideSwitch(LimitSwitchPins::SIDE_LIMIT);

//     Database db(imu, frontTof, sideTof, frontSwitch, sideSwitch);

//     auto [ax, ay] = db.getAcceleration();
//     printf("Accel X: %.2f, Y: %.2f\n", ax, ay);

//     printf("Angular Vel: %.2f deg/s\n", db.getAngularVelocity());

//     printf("Front TOF: %s\n", db.getFrontTofStatus() ? "Detected" : "Not Detected");
//     printf("Side TOF: %s\n", db.getSideTofStatus() ? "Detected" : "Not Detected");

//     printf("Front Switch: %s\n", db.getFrontLimitSwitchStatus() ? "Pressed" : "Not Pressed");
//     printf("Side Switch: %s\n", db.getSideLimitSwitchStatus() ? "Pressed" : "Not Pressed");

//     return 0;
// }



//====================================================
//     WHEEL_ODEM検証用
//====================================================

#include "Control/BehaviorController.hpp"
#include "Provider/WheelOdometry.hpp"
#include "Parts/Encoder.hpp"
#include "config.hpp"

Encoder Y_MESURE_ENCODER(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, 2048, true);
Encoder X_MESURE_ENCODER(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B);

float wheel_radius = 0.03f; // 3cm
int resolution = 2048;      // 1回転あたりのカウント数
WheelOdometry odom(&X_MESURE_ENCODER, &Y_MESURE_ENCODER, wheel_radius, resolution);

float dt = 0.01f; // 10ms周期

void loop() {
    Velocity vel = odom.calculateVelocity(dt);
    Acceleration acc = odom.calculateAcceleration(dt);

    // printf("vel: vx=%.3f [m/s], vy=%.3f [m/s]\n", vel.vx, vel.vy);
    // printf("acc: ax=%.3f [m/s^2], ay=%.3f [m/s^2]\n", acc.ax, acc.ay);
}

#include "Parts/DCMotor.hpp"


int main(){
    // printf("Main Start");
    BehaviorController BC(BehaviorControllerParameter::x_gain, BehaviorControllerParameter::y_gain, BehaviorControllerParameter::angle_gain);
    
    while(true){
        // printf("main roop\n");
        loop();
        BC.setTargetVelocity(0.0,20.0);
        // printf("%f\n", odem.calculateVelocity(dt).vx);
        BC.setVelocity(odom.calculateVelocity(dt).vx,odom.calculateVelocity(dt).vy);
        // BC.setVelocity(0.0, 0.0);
        BC.setTargetAngularVelocity(0.0);
        // BC.setAngularVelocity(0.0);
        BC.setMotor();
        //BC.test();

    }

    return 0;
}


//========================================================================================================================
// #include "mbed.h"
// #include "Control/BehaviorController.hpp"   
// #include "Mechanism/Wheel.hpp"               
// #include "Mechanism/MotorController.hpp"       //検証済み
// #include "Mechanism/WheelController.hpp"
// #include "config.hpp"
// #include <iostream>

// DCMotor FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
// Encoder FrontEncoder(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B);
// DCMotor RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
// Encoder RearLeftEncoder(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B);
// DCMotor RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);
// Encoder RearRightEncoder(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B);

// MotorController mc(FrontMotor, FrontEncoder);
// MotorController mc2(RearLeftMotor, RearLeftEncoder);
// MotorController mc3(RearRightMotor, RearRightEncoder);

// // オムニホイールの配置を設定
// float WHEEL_RAD = 100.0; // 車輪の半径
// float TREAD_RAD = 100.0; // 中心から車輪までの距離
// std::array<WheelConfig, 3> config = {
//     WheelConfig{
//         .wheel_radius = WHEEL_RAD, // 車輪の半径
//         .wheel_x = 0.0, // 車輪のx座標
//         .wheel_y = TREAD_RAD,  // 車輪のy座標
//         .wheel_theta = M_PI // 車輪の角度
//     }, 
//     WheelConfig{
//         .wheel_radius = WHEEL_RAD, 
//         .wheel_x = - M_SQRT3 / 2 * TREAD_RAD,
//         .wheel_y = - 0.5 * TREAD_RAD, 
//         .wheel_theta = M_PI / 3
//     }, 
//     WheelConfig{
//         .wheel_radius = WHEEL_RAD, 
//         .wheel_x = + M_SQRT3 / 2 * TREAD_RAD, 
//         .wheel_y = - 0.5 * TREAD_RAD,
//         .wheel_theta = 5 * M_PI / 3
//     }
// };


// // オドメトリとホイールコントローラの設定
// //Odometry<3> odometry(config, {&encoder1, &encoder2, &encoder3});
// WheelController<3> controller(config, {&mc, &mc2, &mc3});

// int main() {
//     Twist tw;
//     tw.vx = 0.0f;
//     tw.vy = 10.0f;
//     tw.omega = 0.0f;
//     while(true){
//         controller.setTargetTwist(tw);
//         //std::cout << controller.twistToMotorSpeeds(tw) << std::endl;
//         printf("twistToMotorSpeeds[0]:%f\n", controller.twistToMotorSpeeds(tw)[0]);
//         printf("twistToMotorSpeeds[1]:%f\n", controller.twistToMotorSpeeds(tw)[1]);
//         printf("twistToMotorSpeeds[2]:%f\n", controller.twistToMotorSpeeds(tw)[2]);
//     }

//     return 0;
// }
    

    
// //     // RearRightMotor.setDuty(1.0f);
// //     // wait_us(6000000);
// //     // RearRightMotor.setDuty(0.0f);


// //     mc.setTargetSpeed(1.0f);
// //     mc2.setTargetSpeed(1.0f);
// //     mc3.setTargetSpeed(1.0f);
// //     printf("started\n");

// //     while (true) {
// //         // ループ呼び出し確認
// //         if (mc.isLoopCalled()) {
// //             // printf("LOOP!!!\n");
// //             mc.clearLoopFlag();
// //         }

// //         mc3.print();

// //         ThisThread::sleep_for(50ms); // 適度に休ませる

// //     }

// //     return 0;
// // }


// // #include "mbed.h"
// // #include "Control/BehaviorController.hpp"
// // #include "Mechanism/MotorController.hpp"
// // #include <iostream>

// //int main() {
//     // BehaviorController BC;
//     // BC.setPosition(0.0,0.0);
//     // BC.setAngle(0.0);
//     // BC.setTargetPosition(0.0, 2.0);
//     // BC.setTargetAngle(0.0);

//     // //std::cout << BC.calculateTargetVelocity() << std::endl;
//     // BC.move();

// //     printf("s\n");
// //     DCMotor FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
// //     // DCMotor RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
// //     // DCMotor RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

// //     Encoder FrontEncoder(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B);
// //     MotorController mc(FrontMotor, FrontEncoder);

// //     mc.setTargetSpeed(1.0f);
// //     printf("started\n");
// //     while(true){
// //         // std::cout << FrontEncoder.getCount() << std::endl;
// //         mc.print();
// //     }


    
//     // // FrontMotor.setDuty(1.0f);
//     // RearLeftMotor.setDuty(1.0f);
//     // printf("1.0\n");
//     // RearRightMotor.setDuty(-1.0f);
//     // printf("-1.0\n");
//     // wait_us(4000000);

//     // // FrontMotor.setDuty(0.0f);
//     // RearLeftMotor.setDuty(0.0f);
//     // printf("0.0\n");
//     // RearRightMotor.setDuty(0.0f);
//     // printf("0.0\n");
//     // wait_us(4000000);



        


// //     // 1. BehaviorController の生成（デフォルト PID ゲイン・Wheel・Motor）
// //     BehaviorController behavior;

// //     // 2. 目標座標・速度の設定
// //     behavior.setTargetPosition(0, 2.0);       // 位置制御の目標
// //     //behavior.setTargetAngle(3.14 / 2);          // 90度

// //     // 3. 初期状態設定（必要なら）
// //     behavior.setPosition(0.0, 0.0);
// //     behavior.setAngle(0.0);

// //     // 4. ループで動作
// //     while (true) {
// //         // PID計算してモーターに指示
// //         behavior.move();

// //         // 状態を更新する場合はここで setPosition 等を呼ぶ
// //         // 例: センサーから現在位置を取得して
// //         // behavior.setPosition(current_x, current_y);

// //         // ThisThread::sleep_for(100ms);  // 10Hz 制御周期
// //         wait_us(5000);
// //         behavior.stop();
// //     }



    

// //    return 0;
// //}
