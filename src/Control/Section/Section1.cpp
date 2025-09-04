#include "Control/Section/Section1.hpp"

Section1::Section1(StateEstimator& state, BehaviorController& behavior)
    : estimator_(state), behavior_(behavior), finished_(false) {
}

void Section1::run(StateEstimator& state, BehaviorController& behavior, double dt) {
    while (!finished_) {
        // 最新状態を取得
        estimator_.update(dt);
        
        // 前方のToFセンサー値を取得
        TOFStatus tof_status = estimator_.getTOFStatus();
        
        // 要件に基づく制御ロジック
        // 注意: 現在のToFセンサーは距離値ではなく検知状態（bool）のみを返すため、
        // 理想的な距離値ベースの制御の代わりに検知状態を使用します
        
        if (!tof_status.front) {
            // ToFが検知しない（遠距離）→ 高速前進 200mm/s
            behavior_.setTargetVelocity(SPEED_FAST / 1000.0, 0.0); // mm/s → m/s に変換
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
        
        // 毎ループでsetMotor()を呼び出して挙動を反映
        behavior_.setMotor();
        
        // ループ間隔を制御（必要に応じて）
        ThisThread::sleep_for(10ms);
    }
}

bool Section1::isFinished() const {
    return finished_;
}
