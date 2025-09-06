#include <mbed.h>

#include "PinConfig.hpp"
#include "RobotConfig.hpp"

#include "OmniWheel.hpp"
#include "Odometry.hpp"
#include "CoalArmController.hpp"
#include "OilArmController.hpp"

UnbufferedSerial pc(USBTX, USBRX); // PCとのシリアル通信

// === ハードウェアインスタンス ===
DCMotor motor1(PinConfig::OMNI1_PWM, PinConfig::OMNI1_DIR);
Encoder encoder1(PinConfig::OMNI1_ENC_A, PinConfig::OMNI1_ENC_B);
DCMotor motor2(PinConfig::OMNI2_PWM, PinConfig::OMNI2_DIR);
Encoder encoder2(PinConfig::OMNI2_ENC_A, PinConfig::OMNI2_ENC_B);
DCMotor motor3(PinConfig::OMNI3_PWM, PinConfig::OMNI3_DIR);
Encoder encoder3(PinConfig::OMNI3_ENC_A, PinConfig::OMNI3_ENC_B);

Encoder encoder4(PinConfig::ODO_X_ENC_A, PinConfig::ODO_X_ENC_B); // X方向測定輪
Encoder encoder5(PinConfig::ODO_Y_ENC_A, PinConfig::ODO_Y_ENC_B); // Y方向測定輪
Imu imu(PinConfig::IMU_SDA, PinConfig::IMU_SCL); // IMU

DCMotor coal_lift_motor(PinConfig::ARM1_LIFT_PWM, PinConfig::ARM1_LIFT_DIR);
Encoder coal_lift_encoder(PinConfig::ARM1_LIFT_ENC_A, PinConfig::ARM1_LIFT_ENC_B);
ServoMotor coal_gripper_servo(PinConfig::ARM1_GRIPPER_PWM, RobotConfig::SERVO_MIN_PULSE_WIDTH, RobotConfig::SERVO_MAX_PULSE_WIDTH);

DCMotor oil_lift_motor(PinConfig::ARM2_LIFT_PWM, PinConfig::ARM2_LIFT_DIR);
Encoder oil_lift_encoder(PinConfig::ARM2_LIFT_ENC_A, PinConfig::ARM2_LIFT_ENC_B);
DCMotor oil_gripper_motor(PinConfig::ARM2_GRIPPER_PWM, PinConfig::ARM2_GRIPPER_DIR);
Encoder oil_gripper_encoder(PinConfig::ARM2_GRIPPER_ENC_A, PinConfig::ARM2_GRIPPER_ENC_B);

// === ソフトウェアモジュール ===
OmniWheel omni(motor1, encoder1, motor2, encoder2, motor3, encoder3);
Odometry odometry(encoder4, encoder5, imu);

PseudoServo coal_lift(coal_lift_motor, coal_lift_encoder);
CoalArmController coal_arm_controller(coal_lift, coal_gripper_servo);

PseudoServo oil_lift(oil_lift_motor, oil_lift_encoder);
PseudoServo oil_gripper(oil_gripper_motor, oil_gripper_encoder);
OilArmController oil_arm_controller(oil_lift, oil_gripper);

const auto CONTROL_PERIOD = 10ms; // 制御周期 10ms
Timer control_timer;

// === 状態定義 ===
enum class RobotState {
    INITIALIZING,
    // Step 1
    MOVE_FORWARD_1,
    // Step 2
    TURN_LEFT_60_A, ARM2_LIFT_UP_A, ARM2_GRAB_A, ARM2_LIFT_DOWN_A, TURN_RIGHT_60_A,
    // Step 3
    MOVE_BACKWARD_1,
    // Step 4
    TURN_RIGHT_90,
    // Step 5
    MOVE_FORWARD_2,
    // Step 6
    ARM1_LIFT_UP_A, ARM1_GRAB_A, ARM1_LIFT_DOWN_A,
    // Step 7
    TURN_LEFT_60_B, ARM1_LIFT_UP_B, ARM1_RELEASE_B, ARM1_LIFT_DOWN_B, TURN_RIGHT_60_B,
    // Step 8
    MOVE_FORWARD_3,
    // Step 9
    TURN_LEFT_90,
    // Step 10
    MOVE_FORWARD_4,
    // Step 11 (Repeat Step 2 & 7)
    REPEAT_TURN_LEFT_60_A, REPEAT_ARM2_LIFT_UP_A, REPEAT_ARM2_GRAB_A, REPEAT_ARM2_LIFT_DOWN_A, REPEAT_TURN_RIGHT_60_A,
    REPEAT_TURN_LEFT_60_B, REPEAT_ARM1_LIFT_UP_B, REPEAT_ARM1_RELEASE_B, REPEAT_ARM1_LIFT_DOWN_B, REPEAT_TURN_RIGHT_60_B,
    
    COMPLETE,
    ERROR
};
RobotState current_state = RobotState::INITIALIZING;

// === 制御変数 ===
const auto CONTROL_PERIOD = 10ms;
Timer control_timer;
float target_x = 0.0f, target_y = 0.0f, target_theta = 0.0f;

// === ヘルパー関数 ===
// 角度の差分を -PI ~ PI の範囲で計算
float normalize_angle(float angle) {
    while (angle > RobotConfig::PI) angle -= 2.0f * RobotConfig::PI;
    while (angle < -RobotConfig::PI) angle += 2.0f * RobotConfig::PI;
    return angle;
}

// 状態遷移時にログを出力する関数
void change_state(RobotState next_state, const char* state_name) {
    omni.move(0, 0, 0); // 安全のため状態遷移時に一度停止
    current_state = next_state;
    printf("State -> %s\n", state_name);
}

int main() {
    pc.baud(9600);
    printf("--- Robot Sequence Program ---\n");

    // === 初期化 ===
    if (!imu.init()) {
        printf("Error: IMU initialization failed!\n");
        change_state(RobotState::ERROR, "ERROR");
    }
    omni.setParams(RobotConfig::WHEEL_RADIUS_M, RobotConfig::BODY_RADIUS_M, RobotConfig::COUNTS_PER_REV,
                   RobotConfig::KP, RobotConfig::KI, RobotConfig::KD);
    odometry.reset();
    coal_arm_controller.init();
    oil_arm_controller.init();
    
    printf("Initialization complete.\n");

    // === シーケンス開始 ===
    change_state(RobotState::MOVE_FORWARD_1, "MOVE_FORWARD_1 (2.0m)");
    float start_x = odometry.getX();
    float start_y = odometry.getY();
    target_x = start_x + 2.0f;
    target_y = start_y;

    control_timer.start();
    while (current_state != RobotState::COMPLETE && current_state != RobotState::ERROR) {
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(control_timer.elapsed_time()).count() / 1000000.0f;
        control_timer.reset();

        // --- 共通更新処理 ---
        odometry.update();
        omni.update(dt);
        coal_arm_controller.update(dt);
        oil_arm_controller.update(dt);

        float current_x = odometry.getX();
        float current_y = odometry.getY();
        float current_theta = odometry.getThetaRad();
        
        // --- ステートマシン ---
        switch (current_state) {
            // === Step 1: 2m前進 ===
            case RobotState::MOVE_FORWARD_1: {
                float error_x = target_x - current_x;
                float error_y = target_y - current_y;
                if (sqrt(error_x*error_x + error_y*error_y) < RobotConfig::POS_THRESHOLD_M) {
                    target_theta = current_theta + RobotConfig::PI / 3.0f; // 60 deg
                    change_state(RobotState::TURN_LEFT_60_A, "TURN_LEFT_60_A");
                } else {
                    float vx = (error_x * cos(current_theta) + error_y * sin(current_theta)) * RobotConfig::POS_CONTROL_KP;
                    float vy = (-error_x * sin(current_theta) + error_y * cos(current_theta)) * RobotConfig::POS_CONTROL_KP;
                    omni.move(vx, vy, 0);
                }
                break;
            }

            // === Step 2: 左60度旋回 & アーム2動作 & 右60度旋回 ===
            case RobotState::TURN_LEFT_60_A: {
                float angle_error = normalize_angle(target_theta - current_theta);
                if (abs(angle_error) < RobotConfig::ANGLE_THRESHOLD_RAD) {
                    oil_arm_controller.liftTo(RobotConfig::ARM_LIFT_HEIGHT_M);
                    change_state(RobotState::ARM2_LIFT_UP_A, "ARM2_LIFT_UP_A");
                } else {
                    omni.move(0, 0, angle_error * RobotConfig::ANGLE_CONTROL_KP);
                }
                break;
            }
            case RobotState::ARM2_LIFT_UP_A:
                if (oil_arm_controller.isLiftAtTarget()) {
                    oil_arm_controller.closeGripper();
                    change_state(RobotState::ARM2_GRAB_A, "ARM2_GRAB_A");
                }
                break;
            case RobotState::ARM2_GRAB_A:
                if (oil_arm_controller.isGripperAtTarget()) {
                    oil_arm_controller.liftTo(0.0f);
                    change_state(RobotState::ARM2_LIFT_DOWN_A, "ARM2_LIFT_DOWN_A");
                }
                break;
            case RobotState::ARM2_LIFT_DOWN_A:
                if (oil_arm_controller.isLiftAtTarget()) {
                    target_theta = target_theta - RobotConfig::PI / 3.0f; // 元の向きに戻る
                    change_state(RobotState::TURN_RIGHT_60_A, "TURN_RIGHT_60_A");
                }
                break;
            case RobotState::TURN_RIGHT_60_A: {
                float angle_error = normalize_angle(target_theta - current_theta);
                if (abs(angle_error) < RobotConfig::ANGLE_THRESHOLD_RAD) {
                    target_x = current_x + (-3.6f) * cos(current_theta);
                    target_y = current_y + (-3.6f) * sin(current_theta);
                    change_state(RobotState::MOVE_BACKWARD_1, "MOVE_BACKWARD_1 (3.6m)");
                } else {
                    omni.move(0, 0, angle_error * RobotConfig::ANGLE_CONTROL_KP);
                }
                break;
            }

            // ... 他のステップも同様に実装 ...

            // === 最終状態 ===
            default:
                omni.move(0,0,0);
                change_state(RobotState::COMPLETE, "COMPLETE");
                break;
        }

        // 周期維持
        ThisThread::sleep_until(control_timer.read_high_resolution_us() + std::chrono::duration_cast<std::chrono::microseconds>(CONTROL_PERIOD).count());
    }

    if (current_state == RobotState::COMPLETE) {
        printf("--- Sequence Complete ---\n");
    }
    omni.stop();
    return 0;
}
