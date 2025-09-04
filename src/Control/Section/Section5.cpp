#include "Control/Section/Section5.hpp"

Section5::Section5(StateEstimator& state, BehaviorController& behavior)
    : estimator_(state), behavior_(behavior), finished_(false), started_moving_(false), 
      power_plant_collision_(false), moving_to_negative_y_(false) {
}

void Section5::run(StateEstimator& state, BehaviorController& behavior, double dt) {
    while (!finished_) {
        // 最新状態を取得
        estimator_.update(dt);
        
        // 前方のToFセンサー値を取得
        TOFStatus tof_status = estimator_.getTOFStatus();
        
        if (!started_moving_) {
            // 壁衝突後の状態から前進開始
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0); // mm/s → m/s に変換
            started_moving_ = true;
        } else if (!power_plant_collision_) {
            // 発電所の壁に衝突するまで前進
            if (tof_status.front) {
                // ToFが検知する（近距離）→ 発電所の壁に衝突
                power_plant_collision_ = true;
                moving_to_negative_y_ = true;
                
                // -Y軸方向（画像下方向）に直進開始
                behavior_.setTargetVelocity(0.0, -SPEED_FAST / 1000.0); // mm/s → m/s に変換
            } else {
                // 前進継続
                behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0);
            }
        } else if (moving_to_negative_y_) {
            // -Y軸方向に直進中、壁衝突を待つ
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
                // -Y軸方向に直進継続
                behavior_.setTargetVelocity(0.0, -SPEED_FAST / 1000.0);
            }
        }
        
        // 毎ループでsetMotor()を呼び出して挙動を反映
        behavior_.setMotor();
        
        // ループ間隔を制御（必要に応じて）
        ThisThread::sleep_for(10ms);
    }
}

bool Section5::isFinished() const {
    return finished_;
}
