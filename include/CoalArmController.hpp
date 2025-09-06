#ifndef COAL_ARM_CONTROLLER_HPP
#define COAL_ARM_CONTROLLER_HPP

#include "PseudoServo.hpp"
#include "ServoMotor.hpp"

class CoalArmController {
public:
    CoalArmController(PseudoServo& lift_motor, ServoMotor& gripper_servo);

    void init();
    void update(float dt);

    // 高レベルな動作コマンド
    void liftTo(float height_m);
    void grab();
    void release();

    bool isLiftAtTarget(float tolerance_m = 0.001f);

private:
    PseudoServo& m_lift_motor;
    ServoMotor& m_gripper_servo;
};

#endif // COAL_ARM_CONTROLLER_HPP
