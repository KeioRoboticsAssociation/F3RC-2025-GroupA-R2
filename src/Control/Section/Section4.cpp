#include "Control/Section/Section4.hpp"

Section4::Section4(StateEstimator& state, BehaviorController& behavior)
    : estimator_(state), behavior_(behavior), finished_(false), started_moving_(false), coal_collected_(false) {
}

void Section4::run(StateEstimator& state, BehaviorController& behavior, double dt) {
    while (!finished_) {
        // 最新状態を取得
        estimator_.update(dt);
        
        // 前方のToFセンサー値を取得
        TOFStatus tof_status = estimator_.getTOFStatus();
        
        if (!started_moving_) {
            // 壁衝突後の状態から前進開始
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0); // mm/s → m/s に変換
            started_moving_ = true;
        } else if (!coal_collected_) {
            // 石炭回収
            collectCoal();
            coal_collected_ = true;
            
            // 石炭回収後、さらに前進
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0);
        } else {
            // 石炭回収済み、壁衝突を待つ
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

bool Section4::isFinished() const {
    return finished_;
}

void Section4::collectCoal() {
    // 石炭回収のダミー実装
    // 実際の実装では CoalArmController を使用
    // coalArm.collect();
    printf("Section4: 石炭を回収しました\n");
}
