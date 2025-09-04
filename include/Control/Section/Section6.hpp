#pragma once
#include "Control/BaseSection.hpp"

/**
 * @brief Section6: 発電所にアイテムを入れる動作
 * 
 * このセクションでは以下の制御を行います：
 * 1. 前進・旋回を組み合わせてアイテム搬入位置へ移動
 * 2. アーム制御（CoalArmController / OilArmController の呼び出し）
 * 3. 動作終了時に停止
 */
class Section6 : public BaseSection {
public:
    /**
     * @brief コンストラクタ
     * @param state StateEstimatorへの参照
     * @param behavior BehaviorControllerへの参照
     */
    Section6(StateEstimator& state, BehaviorController& behavior);
    
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
    bool moved_to_position_;
    bool arm_operation_completed_;
    
    // 速度設定 [mm/s]
    static constexpr double SPEED_FAST = 200.0;           // 高速
    static constexpr double SPEED_STOP = 0.0;             // 停止
    
    // 角度設定 [rad]
    static constexpr double TARGET_ANGLE = 0.0;           // 目標角度
    
    // アーム操作のダミー関数
    void operateArms();
};
