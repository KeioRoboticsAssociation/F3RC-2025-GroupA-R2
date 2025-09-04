#include "Control/Section/Section6.hpp"

Section6::Section6(StateEstimator& state, BehaviorController& behavior)
    : estimator_(state), behavior_(behavior), finished_(false), moved_to_position_(false), arm_operation_completed_(false) {
}

void Section6::run(StateEstimator& state, BehaviorController& behavior, double dt) {
    while (!finished_) {
        // 最新状態を取得
        estimator_.update(dt);
        
        if (!moved_to_position_) {
            // 前進・旋回を組み合わせてアイテム搬入位置へ移動
            // 実際の実装では、座標や角度に基づいて精密な制御を行う
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0); // mm/s → m/s に変換
            behavior_.setTargetAngle(TARGET_ANGLE);
            
            // 簡易的な位置判定（実際の実装では座標ベースの判定を使用）
            // ここでは一定時間後に移動完了とみなす
            static int move_timer = 0;
            move_timer++;
            if (move_timer > 100) { // 仮のタイマー
                moved_to_position_ = true;
                move_timer = 0;
            }
        } else if (!arm_operation_completed_) {
            // アーム制御
            operateArms();
            arm_operation_completed_ = true;
        } else {
            // 動作終了時に停止
            behavior_.setTargetVelocity(SPEED_STOP, 0.0);
            behavior_.setTargetAngle(TARGET_ANGLE);
            
            // セクション終了
            finished_ = true;
            break;
        }
        
        // 毎ループでsetMotor()を呼び出して挙動を反映
        behavior_.setMotor();
        
        // ループ間隔を制御（必要に応じて）
        ThisThread::sleep_for(10ms);
    }
}

bool Section6::isFinished() const {
    return finished_;
}

void Section6::operateArms() {
    // アーム操作のダミー実装
    // 実際の実装では CoalArmController / OilArmController を使用
    // coalArm.place();
    // oilArm.place();
    printf("Section6: アーム操作を実行しました（石炭・石油を発電所に搬入）\n");
}
