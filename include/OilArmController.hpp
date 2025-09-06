#ifndef OIL_ARM_CONTROLLER_HPP
#define OIL_ARM_CONTROLLER_HPP

#include "PseudoServo.hpp"

class OilArmController {
public:
    OilArmController(PseudoServo& lift_motor, PseudoServo& gripper_motor);

    void init();
    void update(float dt);

    // 高レベルな動作コマンド
    void liftTo(float height_m);
    void openGripper();
    void closeGripper();

    bool isLiftAtTarget(float tolerance_m = 0.001f);
    bool isGripperAtTarget(float tolerance_m = 0.001f);

private:
    PseudoServo& m_lift_motor;
    PseudoServo& m_gripper_motor;
};

#endif // OIL_ARM_CONTROLLER_HPP
