#ifndef OIL_ARM_CONTROLLER_HPP
#define OIL_ARM_CONTROLLER_HPP

#include "PseudoServo.hpp"

class OilArmController {
public:
    /**
     * @brief コンストラクタ
     * @param lift_motor 昇降機構用の疑似サーボ
     * @param gripper_motor グリッパー機構用の疑似サーボ
     */
    OilArmController(PseudoServo& lift_motor, PseudoServo& gripper_motor);

    /**
     * @brief パラメータの初期化
     */
    void init();

    /**
     * @brief 2つの疑似サーボの制御を更新する
     * @param dt 経過時間 [s]
     */
    void update(float dt);

    /**
     * @brief アームを指定した高さに動かす
     * @param height_m 目標の高さ [m]
     */
    void liftTo(float height_m);
    
    /**
     * @brief グリッパーを開く
     */
    void openGripper();

    /**
     * @brief グリッパーを閉じる
     */
    void closeGripper();

private:
    PseudoServo& m_lift_motor;
    PseudoServo& m_gripper_motor;
};

#endif // OIL_ARM_CONTROLLER_HPP
