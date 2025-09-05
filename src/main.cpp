#include "mbed.h"
#include "Control/StateEstimator.hpp"
#include "Control/BehaviorController.hpp"
#include "Provider/WheelOdometry.hpp"
#include "Provider/ImuOdometry.hpp"
#include "Provider/Database.hpp"
#include "Parts/Imu.hpp"
#include "Parts/TimeOfFlightSensor.hpp"
#include "Parts/LimitSwitch.hpp"
#include "Parts/Encoder.hpp"
#include "config.hpp"

// ===== センサ・デバイスのインスタンス化 =====
Imu imu(PinsForSensor::IMU_SDA, PinsForSensor::IMU_SCL);
TimeOfFlightSensor front_tof(PinsForSensor::TOF1), side_tof(PinsForSensor::TOF2);
LimitSwitch front_limit(LimitSwitchPins::FRONT_LIMIT), side_limit(LimitSwitchPins::SIDE_LIMIT);

// Database
Database db(imu, front_tof, side_tof, front_limit, side_limit);

// エンコーダ（測定輪用）
Encoder x_encoder(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B);
Encoder y_encoder(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B);

// WheelOdometry
WheelOdometry wheel_odom(&x_encoder, &y_encoder, /*wheel_radius=*/0.03f, /*resolution=*/2048);

// ImuOdometry
ImuOdometry imu_odom(db);

// StateEstimator
StateEstimator state_estimator(db, wheel_odom, imu_odom);

// PIDゲイン（frequencyはint）
PIDGain x_gain{0.1f, 0.0f, 0.1f, 1};
PIDGain y_gain{0.1f, 0.0f, 0.1f, 1};
PIDGain angle_gain{0.1f, 0.0f, 0.1f, 1};

BehaviorController controller(x_gain, y_gain, angle_gain);

int log_cnt = 0;

int main() {
    printf("=== Robot Start ===\n");

    constexpr float dt = 0.01f;
    constexpr int dt_us = static_cast<int>(dt * 1e6);

    // 自己位置の登録
    state_estimator.resetCoordinateSystem();

    // 目標位置 (2.0, 0.0) と目標角度 π/2 rad
    controller.setTargetPosition(0.0, 0.0);
    controller.setTargetAngle(M_PI / 2.);

    // ------------------------
    // Pゲイン変化用 Timer
    // ------------------------
    Timer timer;
    timer.start();

    const float P_start = 0.1f;
    const float P_end   = 1.5f;
    const float duration = 30.0f; // 10秒かけて P を増加

    while (true) {
        log_cnt = ++log_cnt % 100;
        if (log_cnt == 0) {
            printf("\n-----\n");
        }

        // 状態推定
        state_estimator.update(dt);

        // 推定結果を BehaviorController に渡す
        Pose2D pose = state_estimator.getRelativePosition();
        auto vel = state_estimator.getVelocity();
        auto acc = state_estimator.getAcceleration();
        double ang_vel = state_estimator.getAngularVelocity();

        if (log_cnt == 0) {
            printf("Estimated Pose: x=%f, y=%f, theta=%f\n", pose.x, pose.y, pose.theta);
            printf("Estimated Velocity: vx=%f, vy=%f\n", vel.first, vel.second);
            printf("Estimated Acceleration: ax=%f, ay=%f\n", acc.first, acc.second);
            printf("Estimated Angular Velocity: omega=%f\n", ang_vel);
        }

        controller.updateFromStateEstimator(state_estimator);

        // ------------------------
        // Pゲインをなめらかに変更
        // ------------------------
        float t = std::chrono::duration<float>(timer.elapsed_time()).count();
        float alpha = t / duration;
        if (alpha > 1.0f) alpha = 1.0f;
        float P_gain = P_start + (P_end - P_start) * alpha;
        controller.setPGain(P_gain);

        // ------------------------
        // モータ出力
        // ------------------------
        controller.setMotor();

        // // デバッグ出力
        if (log_cnt == 0) printf("time=%f, P=%f, pose=(%f,%f,%f)\n", t, P_gain, pose.x, pose.y, pose.theta);

        // printf("IMU:%f",imu.getYaw());

        // 周期待ち
        wait_us(dt_us);


        // if (10.0 > timer.read() == 5.0){
        //     controller.stop();
        //     wait_us(3000000);
        //     float _y = state_estimator.getRelativePosition().y;
        //     float _t = state_estimator.getRelativePosition().theta;
        //     controller.setTargetPosition(3.0, _y);
        //     controller.setTargetAngle(_t);
        // }else if(timer.read() == 15.0){
        //     controller.stop();
        //     wait_us(3000000);
        //     float _x = state_estimator.getRelativePosition().x;
        //     float _y = state_estimator.getRelativePosition().y;
        //     controller.setTargetPosition(_x,_y);
        //     controller.setTargetAngle(90.0);
        // }else if(timer.read() == 25){
        //     controller.stop();
        //     wait_us(3000000);
        //     break;

        // }
    }
}
