#include "Control/Section/Section2.hpp"

Section2::Section2(StateEstimator& state, BehaviorController& behavior)
    : estimator_(state), behavior_(behavior), finished_(false), coal_collected_(false) {
}

void Section2::run(StateEstimator& state, BehaviorController& behavior, double dt) {
    while (!finished_) {
        // 最新状態を取得
        estimator_.update(dt);
        
        // 前方のToFセンサー値を取得
        TOFStatus tof_status = estimator_.getTOFStatus();
        
        if (!coal_collected_) {
            // 石炭を取得
            collectCoal();
            coal_collected_ = true;
            
            // 石炭取得後、前進開始
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0); // mm/s → m/s に変換
        } else {
            // 石炭取得済み、壁衝突を待つ
            if (tof_status.front) {
                // ToFが検知する（近距離）→ 壁衝突と判定
                // モーター停止
                behavior_.setTargetVelocity(SPEED_STOP, 0.0);
                
                // StateEstimatorをリセット
                estimator_.resetCoordinateSystem();
                
                // セクション終了
                finished_ = true;
                break;
            } else {
                // 前進継続
                behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0);
            }
        }
        
        // 毎ループでsetMotor()を呼び出して挙動を反映
        behavior_.setMotor();
        
        // ループ間隔を制御（必要に応じて）
        ThisThread::sleep_for(10ms);
    }
}

bool Section2::isFinished() const {
    return finished_;
}

void Section2::collectCoal() {
    // 石炭取得のダミー実装
    // 実際の実装では CoalArmController を使用
    // coalArm.collect();
    printf("Section2: 石炭を取得しました\n");
}
