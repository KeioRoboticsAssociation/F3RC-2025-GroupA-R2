#include <mbed.h>

#include "PinConfig.hpp"
#include "RobotConfig.hpp"

#include "OmniWheel.hpp"
#include "Odometry.hpp"

UnbufferedSerial pc(USBTX, USBRX); // PCとのシリアル通信

DCMotor motor1(PinConfig::OMNI1_PWM, PinConfig::OMNI1_DIR);
Encoder encoder1(PinConfig::OMNI1_ENC_A, PinConfig::OMNI1_ENC_B);
DCMotor motor2(PinConfig::OMNI2_PWM, PinConfig::OMNI2_DIR);
Encoder encoder2(PinConfig::OMNI2_ENC_A, PinConfig::OMNI2_ENC_B);
DCMotor motor3(PinConfig::OMNI3_PWM, PinConfig::OMNI3_DIR);
Encoder encoder3(PinConfig::OMNI3_ENC_A, PinConfig::OMNI3_ENC_B);
OmniWheel omni(motor1, encoder1, motor2, encoder2, motor3, encoder3);

Encoder encoder4(PinConfig::ODO_X_ENC_A, PinConfig::ODO_X_ENC_B); // X方向測定輪
Encoder encoder5(PinConfig::ODO_Y_ENC_A, PinConfig::ODO_Y_ENC_B); // Y方向測定輪
Imu imu(PinConfig::IMU_SDA, PinConfig::IMU_SCL); // IMU
Odometry odometry(encoder4, encoder5, imu);

const auto CONTROL_PERIOD = 10ms; // 制御周期 10ms
Timer control_timer;

constexpr float TARGET_X = 0.5f; // 目標X座標 [m]
constexpr float TARGET_Y = 0.0f; // 目標Y座標 [m]
constexpr float TARGET_YAW = 0.0f; // 目標方位角 [rad]

int main() {
    pc.baud(9600);

    // === 初期化シーケンス ===
    printf("Robot initializing...\n");
    if (!imu.init()) {
        printf("IMU initialization failed!\n");
        return -1;
    }
    omni.setParams(RobotConfig::WHEEL_RADIUS, RobotConfig::BODY_RADIUS, 
                   RobotConfig::COUNTS_PER_REV,
                   RobotConfig::KP, RobotConfig::KI, RobotConfig::KD);
    odometry.reset();
    printf("Initialization complete.\n");

    control_timer.start();
    while (true) {
        // --- 1. 周期管理 ---
        // ループ１周にかかった時間を計算
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(control_timer.elapsed_time()).count() / 1000000.0f;
        control_timer.reset();

        // --- 2. 状態観測 (State Estimation) ---
        odometry.update();
        float current_x = odometry.getX();
        float current_y = odometry.getY();
        float current_theta = odometry.getThetaRad();

        // --- 3. 高レベル制御 (Position Control) ---
        // 目標までの誤差をワールド座標系で計算
        float error_x = TARGET_X - current_x;
        float error_y = TARGET_Y - current_y;
        float distance_to_goal = sqrt(error_x * error_x + error_y * error_y);

        float target_vx = 0.0f;
        float target_vy = 0.0f;
        float target_omega = 0.0f; // 今回は回転制御は行わない

        // ゴールに到達していなければ、目標速度を計算
        if (distance_to_goal > RobotConfig::GOAL_THRESHOLD_M) {
            // ワールド座標系での誤差を、ロボット座標系での速度指令に変換
            // これにより、ロボットは常にゴールの方を向いて進もうとする
            target_vx = (error_x * cos(current_theta) + error_y * sin(current_theta)) * RobotConfig::POS_CONTROL_KP;
            target_vy = (-error_x * sin(current_theta) + error_y * cos(current_theta)) * RobotConfig::POS_CONTROL_KP;
        }
        
        // --- 4. 低レベル制御 (Velocity Control) ---
        omni.move(target_vx, target_vy, target_omega);
        omni.update(dt); // 速度PID制御を実行

        // --- 5. デバッグ情報表示 (100msごとに表示) ---
        static Timer debug_timer;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(debug_timer.elapsed_time()).count() > 100) {
            printf("Pos: (%.2f, %.2f) Goal: (%.2f, %.2f) VelCmd: (%.2f, %.2f)\n",
                   current_x, current_y, TARGET_X, TARGET_Y, target_vx, target_vy);
            debug_timer.reset();
        }

        // 制御周期を維持するために待機
        ThisThread::sleep_for(CONTROL_PERIOD);
    }
}
