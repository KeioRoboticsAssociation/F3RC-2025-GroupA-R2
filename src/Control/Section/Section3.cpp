#include "Control/Section/Section3.hpp"

Section3::Section3(StateEstimator& state, BehaviorController& behavior)
    : estimator_(state), behavior_(behavior), finished_(false), started_moving_(false) {
}

void Section3::run(StateEstimator& state, BehaviorController& behavior, double dt) {
    while (!finished_) {
        // 最新状態を取得
        estimator_.update(dt);
        
        // 前方のToFセンサー値を取得
        TOFStatus tof_status = estimator_.getTOFStatus();
        
        if (!started_moving_) {
            // 壁衝突後の状態から前進開始
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0); // mm/s → m/s に変換
            started_moving_ = true;
        } else {
            // 前進中、ToFセンサーに基づいて制御
            if (!tof_status.front) {
                // ToFが検知しない（遠距離）→ 高速前進
                behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0);
            } else {
                // ToFが検知する（近距離）→ 壁衝突と判定
                // モーター停止
                behavior_.setTargetVelocity(SPEED_STOP, 0.0);
                
                // StateEstimatorをリセット
                estimator_.resetCoordinateSystem();
                
                // セクション終了
                finished_ = true;
                break;
            }
        }
        
        // 毎ループでsetMotor()を呼び出して挙動を反映
        behavior_.setMotor();
        
        // ループ間隔を制御（必要に応じて）
        ThisThread::sleep_for(10ms);
    }
}

bool Section3::isFinished() const {
    return finished_;
}
