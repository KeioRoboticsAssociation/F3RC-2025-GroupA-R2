#include "CoalArmController.hpp"
#include "RobotConfig.hpp" // アーム用の設定を追加

CoalArmController::CoalArmController(PseudoServo& lift_motor, ServoMotor& gripper_servo)
    : m_lift_motor(lift_motor), m_gripper_servo(gripper_servo)
{
}

void CoalArmController::init() {
    m_lift_motor.setParams(
        RobotConfig::ARM1_LIFT_KP, RobotConfig::ARM1_LIFT_KI, RobotConfig::ARM1_LIFT_KD,
        RobotConfig::ARM1_LIFT_METERS_PER_REV, RobotConfig::ARM1_LIFT_COUNTS_PER_REV
    );
    release(); // 最初は開いておく
}

void CoalArmController::update(float dt) {
    m_lift_motor.update(dt);
}

void CoalArmController::liftTo(float height_m) {
    m_lift_motor.setPosition(height_m);
}

void CoalArmController::grab() {
    m_gripper_servo.setAngleDeg(RobotConfig::ARM1_GRIP_CLOSE_DEG);
}

void CoalArmController::release() {
    m_gripper_servo.setAngleDeg(RobotConfig::ARM1_GRIP_OPEN_DEG);
}

bool CoalArmController::isLiftAtTarget(float tolerance_m) {
    return m_lift_motor.isAtTarget(tolerance_m);
}
