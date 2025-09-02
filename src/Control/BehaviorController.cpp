#include "Control/BehaviorController.hpp"

// defaultMotors() 実装
std::array<MotorController*, 3> BehaviorController::defaultMotors() {
    static DCMotor FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR);
    static DCMotor RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR);
    static DCMotor RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR);

    static Encoder FrontEncoder(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B);
    static Encoder RearLeftEncoder(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B);
    static Encoder RearRightEncoder(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B);

    static MotorController FrontMotorController(FrontMotor, FrontEncoder);
    static MotorController RearLeftMotorController(RearLeftMotor, RearLeftEncoder);
    static MotorController RearRightMotorController(RearRightMotor, RearRightEncoder);

    return { &FrontMotorController, &RearLeftMotorController, &RearRightMotorController };
}

// コンストラクタ
BehaviorController::BehaviorController(PIDGain x_pid_gain,
                                       PIDGain y_pid_gain,
                                       PIDGain angle_pid_gain,
                                       const std::array<WheelConfig, 3>& wheel_configs,
                                       const std::array<MotorController*, 3>& motors,
                                       float max_speed)
    : x_pid_controller(x_pid_gain),
      y_pid_controller(y_pid_gain),
      angle_pid_controller(angle_pid_gain),
      wheel_controller(wheel_configs, motors, max_speed),
      xy_target_mode(TargetMode::Velocity),
      angle_target_mode(TargetMode::Velocity) {}

// 各種 set メソッド
void BehaviorController::setTargetPosition(double x, double y) { target_pos_x = x; target_pos_y = y; xy_target_mode = TargetMode::Position; }
void BehaviorController::setTargetVelocity(double vx, double vy) { target_velocity_x = vx; target_velocity_y = vy; xy_target_mode = TargetMode::Velocity; }
void BehaviorController::setTargetAngle(double theta) { target_angle = theta; angle_target_mode = TargetMode::Position; }
void BehaviorController::setTargetAngularVelocity(double omega) { target_velocity_angle = omega; angle_target_mode = TargetMode::Velocity; }
void BehaviorController::setPosition(double x, double y) { current_pos_x = x; current_pos_y = y; }
void BehaviorController::setVelocity(double vx, double vy) { current_velocity_x = vx; current_velocity_y = vy; }
void BehaviorController::setAcceleration(double ax, double ay) { current_acceleration_x = ax; current_acceleration_y = ay; }
void BehaviorController::setAngle(double theta) { current_angle = theta; }
void BehaviorController::setAngularVelocity(double omega) { current_velocity_angle = omega; }
void BehaviorController::setAngularAcceleration(double alpha) { current_acceleration_angle = alpha; }

// PIDで計算
Twist BehaviorController::calculateTargetVelocity() {
    double x_output = (xy_target_mode == TargetMode::Position) ? x_pid_controller.calculate(target_pos_x - current_pos_x) : target_velocity_x;
    double y_output = (xy_target_mode == TargetMode::Position) ? y_pid_controller.calculate(target_pos_y - current_pos_y) : target_velocity_y;
    double angle_output = (angle_target_mode == TargetMode::Position) ? angle_pid_controller.calculate(target_angle - current_angle) : target_velocity_angle;
    return Twist{ (float)x_output, (float)y_output, (float)angle_output };
}

// 実際にWheelControllerに指示
void BehaviorController::move() { wheel_controller.setTargetTwist(calculateTargetVelocity()); }
void BehaviorController::stop() { wheel_controller.stop(); }
