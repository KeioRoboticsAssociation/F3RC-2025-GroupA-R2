#ifndef INCLUDED_VARIABLE_HPP_
#define INCLUDED_VARIABLE_HPP_

// Parameters for the oil arm.
namespace oil_arm_config {
    // Arm Servo Angles (degrees)
    constexpr float kArmInitialAngle = 90.0f;
    constexpr float kArmExtendAngle = 0.0f;
    constexpr float kArmLiftAngle = 45.0f;
    constexpr float kArmPlaceAngle = 20.0f;

    // Gripper Servo Angles (degrees)
    constexpr float kGripperOpenAngle = 90.0f;
    constexpr float kGripperCloseAngle = 0.0f;
}

#endif // INCLUDED_VARIABLE_HPP_
