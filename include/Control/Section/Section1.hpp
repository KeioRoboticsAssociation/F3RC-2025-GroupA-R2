#pragma once
#include "Control/BaseSection.hpp"

/**
 * @brief Section1: スタート→ToF検知で減速→前方の壁に衝突（座標リセット）
 * 
 * このセクションでは以下の制御を行います：
 * 1. 前方のToFセンサー値を監視
 * 2. ToF > 200mm: 前進速度 200mm/s
 * 3. 200mm >= ToF > 100mm: 減速して 100mm/s
 * 4. ToF <= 100mm: 壁衝突と判定し、モーター停止
 * 5. 壁衝突時にStateEstimatorをリセット
 */
class Section1 : public BaseSection {
public:
    /**
     * @brief コンストラクタ
     * @param state StateEstimatorへの参照
     * @param behavior BehaviorControllerへの参照
     */
    Section1(StateEstimator& state, BehaviorController& behavior);
    
    /**
     * @brief セクションの実行（1周期分の処理）
     * @param state StateEstimatorへの参照
     * @param behavior BehaviorControllerへの参照
     * @param dt 更新周期[s]
     */
    void run(StateEstimator& state, BehaviorController& behavior, double dt) override;
    
    /**
     * @brief セクションが終了したかどうかを判定
     * @return true: 終了, false: 継続
     */
    bool isFinished() const override;

private:
    StateEstimator& estimator_;
    BehaviorController& behavior_;
    bool finished_;
    
    // ToFセンサーの閾値 [mm]
    static constexpr double TOF_FAR_THRESHOLD = 200.0;    // 遠距離閾値
    static constexpr double TOF_NEAR_THRESHOLD = 100.0;   // 近距離閾値
    
    // 速度設定 [mm/s]
    static constexpr double SPEED_FAST = 200.0;           // 高速
    static constexpr double SPEED_SLOW = 100.0;           // 減速
    static constexpr double SPEED_STOP = 0.0;             // 停止
};
