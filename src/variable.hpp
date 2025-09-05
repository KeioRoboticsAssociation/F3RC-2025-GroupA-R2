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

namespace CoalArmParameter {
    // 角度（度）
    constexpr float ARM_INIT_ANGLE = 0.0f;
    constexpr float ARM_DOWN_ANGLE = -45.0f;
    constexpr float ARM_UP_ANGLE   = 0.0f;

    constexpr float GRIP_OPEN_ANGLE  = 0.0f;
    constexpr float GRIP_CLOSE_ANGLE = 45.0f;

    // 許容誤差
    constexpr float ARM_DOWN_THRESHOLD = -44.0f;
    constexpr float ARM_UP_THRESHOLD   = -1.0f;

    // タイマー（ms）
    constexpr int ARM_MOVE_TIMEOUT_MS   = 3000;
    constexpr int GRIP_ACTION_TIMEOUT_MS = 1000;
}

namespace BehaviorControllerParameter {
    // PIDゲイン設定
    constexpr PIDGain x_gain{1.0, 0.0, 0.0};
    constexpr PIDGain y_gain{1.0, 0.0, 0.0};
    constexpr PIDGain angle_gain{1.0, 0.0, 0.0};

    constexpr float WHEEL_RAD = 45.0;           // 車輪の半径
    constexpr float MESURE_WHEEL_RAD = 45.0;    // 測定輪の半径
    constexpr float TREAD_RAD = 210.0;          // 中心から車輪までの距離
}
#endif // INCLUDED_VARIABLE_HPP_