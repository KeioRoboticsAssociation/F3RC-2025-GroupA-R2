#include "mbed.h"
#include "PinConfig.hpp"
#include "RobotConfig.hpp"
#include "OmniWheel.hpp"
#include "Odometry.hpp"

// PCとのシリアル通信
UnbufferedSerial pc(USBTX, USBRX);

// === ハードウェアインスタンス (アーム以外) ===
DCMotor motor1(PinConfig::OMNI1_PWM, PinConfig::OMNI1_DIR);
Encoder encoder1(PinConfig::OMNI1_ENC_A, PinConfig::OMNI1_ENC_B);
DCMotor motor2(PinConfig::OMNI2_PWM, PinConfig::OMNI2_DIR);
Encoder encoder2(PinConfig::OMNI2_ENC_A, PinConfig::OMNI2_ENC_B);
DCMotor motor3(PinConfig::OMNI3_PWM, PinConfig::OMNI3_DIR);
Encoder encoder3(PinConfig::OMNI3_ENC_A, PinConfig::OMNI3_ENC_B);

Encoder encoder4(PinConfig::ODO_X_ENC_A, PinConfig::ODO_X_ENC_B, 2048, true); // X方向測定輪
Encoder encoder5(PinConfig::ODO_Y_ENC_A, PinConfig::ODO_Y_ENC_B); // Y方向測定輪
Imu imu(PinConfig::IMU_SDA, PinConfig::IMU_SCL); // IMU

// === ソフトウェアモジュール (アーム以外) ===
OmniWheel omni(motor1, encoder1, motor2, encoder2, motor3, encoder3);
Odometry odometry(encoder4, encoder5, imu);

// === テスト用の状態定義 ===
enum class TestState {
    INITIALIZING,
    FORWARD,
    WAIT_1,
    TURN_RIGHT,
    WAIT_2,
    SIDEWAYS_RIGHT,
    STOP
};
TestState current_state = TestState::INITIALIZING;

// === 制御変数 ===
const auto CONTROL_PERIOD = 10ms;
Timer control_timer;
Timer state_timer;

// ヘルパー関数
void change_state(TestState next_state, const char* state_name) {
    omni.move(0, 0, 0); 
    current_state = next_state;
    state_timer.reset();
    state_timer.start();
    printf("\n--- State -> %s ---\n", state_name);
}

int main() {
    pc.baud(9600);
    printf("--- OmniWheel & Odometry Test Program ---\n");

    // === 初期化 ===
    if (!imu.init()) {
        printf("Error: IMU initialization failed!\n");
        return -1;
    }
    omni.setParams(RobotConfig::WHEEL_RADIUS_M, RobotConfig::BODY_RADIUS_M, RobotConfig::COUNTS_PER_REV,
                   RobotConfig::KP, RobotConfig::KI, RobotConfig::KD);
    odometry.reset();
    printf("Initialization complete.\n");

    change_state(TestState::FORWARD, "FORWARD (2 seconds)");

    control_timer.start();
    while (current_state != TestState::STOP) {
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(control_timer.elapsed_time()).count() / 1000000.0f;
        control_timer.reset();

        // --- 共通更新処理 ---
        odometry.update(dt);
        omni.update(dt);

        // --- デバッグ情報表示 ---
        auto duties = omni.getMotorDuties();
        float angular_velocity_z = imu.getAngularVelocity().z;

        printf("X:%6.3f Y:%6.3f Th:%6.1f | AV_Z:%6.2f | M1:%5.2f M2:%5.2f M3:%5.2f\r",
               odometry.getX(), odometry.getY(), odometry.getThetaDeg(),
               angular_velocity_z,
               duties.duty1, duties.duty2, duties.duty3);
        fflush(stdout);

        // --- テスト用ステートマシン ---
        switch (current_state) {
            case TestState::FORWARD:
                omni.move(0.05f, 0.0f, 0.0f); // 前方に0.05m/sで進む
                if (std::chrono::duration_cast<std::chrono::seconds>(state_timer.elapsed_time()).count() >= 2) {
                    change_state(TestState::WAIT_1, "WAIT (2 seconds)");
                }
                break;

            case TestState::WAIT_1:
                omni.move(0.0f, 0.0f, 0.0f); // 停止
                if (std::chrono::duration_cast<std::chrono::seconds>(state_timer.elapsed_time()).count() >= 2) {
                    change_state(TestState::TURN_RIGHT, "TURN_RIGHT (-1.0 rad/s)");
                }
                break;

            case TestState::TURN_RIGHT:
                omni.move(0.0f, 0.0f, -1.0f); // 右に約60deg/sで回転
                if (odometry.getThetaRad() <= -RobotConfig::PI / 2.0f) { // 90度回転したら
                    change_state(TestState::WAIT_2, "WAIT (2 seconds)");
                }
                break;

            case TestState::WAIT_2:
                omni.move(0.0f, 0.0f, 0.0f); // 停止
                if (std::chrono::duration_cast<std::chrono::seconds>(state_timer.elapsed_time()).count() >= 2) {
                    change_state(TestState::SIDEWAYS_RIGHT, "SIDEWAYS_RIGHT (2 seconds)");
                }
                break;

            case TestState::SIDEWAYS_RIGHT:
                omni.move(0.0f, 0.2f, 0.0f); // 右に0.2m/sで平行移動
                if (std::chrono::duration_cast<std::chrono::seconds>(state_timer.elapsed_time()).count() >= 2) {
                    change_state(TestState::STOP, "STOP");
                }
                break;
            
            default:
                change_state(TestState::STOP, "STOP");
                break;
        }
        
        ThisThread::sleep_for(CONTROL_PERIOD);
    }
    
    omni.stop();
    printf("\n--- Test Complete ---\n");
    while(true){
        // 終了後もオドメトリを更新し続ける
        odometry.update(0.1f);
        printf("Final Pos -> X: %6.3f [m], Y: %6.3f [m], Theta: %6.3f [deg]\r",
               odometry.getX(), odometry.getY(), odometry.getThetaDeg());
        fflush(stdout);
        ThisThread::sleep_for(100ms);
    }
}
