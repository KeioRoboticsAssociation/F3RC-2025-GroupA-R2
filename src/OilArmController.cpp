#include "OilArmController.hpp"
#include "RobotConfig.hpp"

OilArmController::OilArmController(PseudoServo& lift_motor, PseudoServo& gripper_motor)
    : m_lift_motor(lift_motor), m_gripper_motor(gripper_motor)
{
}

void OilArmController::init() {
    // 昇降用モーターのパラメータ設定
    m_lift_motor.setParams(
        RobotConfig::ARM2_LIFT_KP, RobotConfig::ARM2_LIFT_KI, RobotConfig::ARM2_LIFT_KD,
        RobotConfig::ARM2_LIFT_METERS_PER_REV, RobotConfig::ARM2_LIFT_COUNTS_PER_REV
    );

    // グリッパー用モーターのパラメータ設定
    m_gripper_motor.setParams(
        RobotConfig::ARM2_GRIP_KP, RobotConfig::ARM2_GRIP_KI, RobotConfig::ARM2_GRIP_KD,
        RobotConfig::ARM2_GRIP_METERS_PER_REV, RobotConfig::ARM2_GRIP_COUNTS_PER_REV
    );
}

void OilArmController::update(float dt) {
    // 両方の疑似サーボの制御を更新する
    m_lift_motor.update(dt);
    m_gripper_motor.update(dt);
}

void OilArmController::liftTo(float height_m) {
    m_lift_motor.setPosition(height_m);
}

void OilArmController::openGripper() {
    // RobotConfigで定義された「開いた状態」の位置を目標に設定
    m_gripper_motor.setPosition(RobotConfig::ARM2_GRIP_OPEN_ANGLE_DEG);
}

void OilArmController::closeGripper() {
    // RobotConfigで定義された「閉じた状態」の位置を目標に設定
    m_gripper_motor.setPosition(RobotConfig::ARM2_GRIP_CLOSE_ANGLE_DEG);
}
