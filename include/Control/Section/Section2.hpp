#pragma once
#include "Control/BaseSection.hpp"

/**
 * @brief Section2: 前方壁衝突後 → 石炭を取得 → 再度同じ壁に衝突 → 座標リセット
 * 
 * このセクションでは以下の制御を行います：
 * 1. 前方壁衝突後の状態から開始
 * 2. 石炭を取得（CoalArmController呼び出し）
 * 3. 再度同じ壁に衝突（100mm以下）
 * 4. 壁衝突時にStateEstimatorをリセット
 */
class Section2 : public BaseSection {
public:
    /**
     * @brief コンストラクタ
     * @param state StateEstimatorへの参照
     * @param behavior BehaviorControllerへの参照
     */
    Section2(StateEstimator& state, BehaviorController& behavior);
    
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
    bool coal_collected_;
    
    // ToFセンサーの閾値 [mm]
    static constexpr double TOF_NEAR_THRESHOLD = 100.0;   // 近距離閾値
    
    // 速度設定 [mm/s]
    static constexpr double SPEED_FAST = 200.0;           // 高速
    static constexpr double SPEED_STOP = 0.0;             // 停止
    
    // 石炭取得のダミー関数
    void collectCoal();
};
