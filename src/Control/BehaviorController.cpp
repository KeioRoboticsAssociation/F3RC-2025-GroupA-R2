#include "Control/BehaviorController.hpp"
#include "config.hpp"
#include <cmath> 


// オムニホイールの配置を設定
float WHEEL_RAD_ = BehaviorControllerParameter::WHEEL_RAD; // 車輪の半径
float TREAD_RAD_ = BehaviorControllerParameter::TREAD_RAD; // 中心から車輪までの距離
std::array<WheelConfig, 3> config = {
    WheelConfig{
        .wheel_radius = WHEEL_RAD_, // 車輪の半径
        .wheel_x = 0.0, // 車輪のx座標
        .wheel_y = TREAD_RAD_,  // 車輪のy座標
        .wheel_theta = M_PI // 車輪の角度
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD_, 
        .wheel_x = - M_SQRT3 / 2 * TREAD_RAD_,
        .wheel_y = - 0.5 * TREAD_RAD_, 
        .wheel_theta = M_PI / 3
    }, 
    WheelConfig{
        .wheel_radius = WHEEL_RAD_, 
        .wheel_x = + M_SQRT3 / 2 * TREAD_RAD_, 
        .wheel_y = - 0.5 * TREAD_RAD_,
        .wheel_theta = 5 * M_PI / 3
    }
};

BehaviorController::BehaviorController(PIDGain x_pid_gain, PIDGain y_pid_gain, PIDGain angle_pid_gain)
    : FrontMotor(PwmOutPins::OMUNI_MOTOR1_PWM, DigitalOutPins::OMUNI_MOTOR1_DIR),
      FrontEncoder(InterruptInPins::OMUNI_ENCODER1_A, DigitalInPins::OMUNI_ENCODER1_B),
      RearLeftMotor(PwmOutPins::OMUNI_MOTOR2_PWM, DigitalOutPins::OMUNI_MOTOR2_DIR),
      RearLeftEncoder(InterruptInPins::OMUNI_ENCODER2_A, DigitalInPins::OMUNI_ENCODER2_B),
      RearRightMotor(PwmOutPins::OMUNI_MOTOR3_PWM, DigitalOutPins::OMUNI_MOTOR3_DIR),
      RearRightEncoder(InterruptInPins::OMUNI_ENCODER3_A, DigitalInPins::OMUNI_ENCODER3_B),
      FrontMotorController(FrontMotor, FrontEncoder),
      RearLeftMotorController(RearLeftMotor, RearLeftEncoder),
      RearRightMotorController(RearRightMotor, RearRightEncoder),
      x_pid_controller(x_pid_gain),
      y_pid_controller(y_pid_gain),
      angle_pid_controller(angle_pid_gain),
      wheel_v_controller(config, {&FrontMotorController, &RearLeftMotorController, &RearRightMotorController})
      {
        x_pid_controller.setFrequency(x_pid_gain.frequency);
        y_pid_controller.setFrequency(y_pid_gain.frequency);
        angle_pid_controller.setFrequency(angle_pid_gain.frequency);
        
      }

void BehaviorController::setTargetPosition(double x, double y)
{
    target_pos_x = x;
    target_pos_y = y;
    xy_target_mode = TargetMode::Position;
}

void BehaviorController::setTargetAngle(double theta)
{
    target_angle = theta;
    angle_target_mode = TargetMode::Position;
}

void BehaviorController::setTargetVelocity(double vx, double vy)
{
    target_velocity_x = vx;
    target_velocity_y = vy;
    xy_target_mode = TargetMode::Velocity;
}

void BehaviorController::setTargetAngularVelocity(double omega)
{
    target_velocity_angle = omega;
    angle_target_mode = TargetMode::Velocity;
}

void BehaviorController::setPosition(double x, double y)
{
    current_pos_x = x;
    current_pos_y = y;
}

void BehaviorController::setVelocity(double vx, double vy)
{
    current_velocity_x = vx;
    current_velocity_y = vy;
}

void BehaviorController::setAcceleration(double ax, double ay)
{
    current_acceleration_x = ax;
    current_acceleration_y = ay;
}

void BehaviorController::setAngle(double theta)
{
    current_angle = theta;
}

void BehaviorController::setAngularVelocity(double omega)
{
    current_velocity_angle = omega;
}

void BehaviorController::setAngularAcceleration(double alpha)
{
    current_acceleration_angle = alpha;
}

/**
 * @brief 制御モードに応じて目標速度(Twist)を計算します。
 * * @return Twist 目標の並進速度(vx, vy)と角速度(omega)
 */
Twist BehaviorController::calculateTargetVelocity()
{
    double x_output;
    double y_output;
    double angle_output;

    // XY方向の目標速度を計算
    if (xy_target_mode == TargetMode::Position)
    {
        // 位置制御モード：位置偏差からPIDで目標速度を計算
        x_output = x_pid_controller.calculate(target_pos_x - current_pos_x);
        y_output = y_pid_controller.calculate(target_pos_y - current_pos_y);
    }
    else // TargetMode::Velocity
    {
        // 速度制御モード：設定された目標速度をそのまま使用
        x_output = target_velocity_x;
        y_output = target_velocity_y;
    }

    // 角度（回転）の目標速度（角速度）を計算
    if (angle_target_mode == TargetMode::Position)
    {
        // 角度制御モード：角度偏差からPIDで目標角速度を計算
        angle_output = angle_pid_controller.calculate(target_angle - current_angle);
    }
    else // TargetMode::Velocity
    {
        // 角速度制御モード：設定された目標角速度をそのまま使用
        angle_output = target_velocity_angle;
    }

    // ==========================================================
    //                        NaNチェック
    // ==========================================================
    if (std::isnan(x_output) || std::isnan(y_output) || std::isnan(angle_output))
    {
        // いずれかの出力がNaNだった場合の処理
        printf("!!! NaN detected in calculateTargetVelocity() !!!\n");
        printf("x_output: %f, y_output: %f, angle_output: %f\n", x_output, y_output, angle_output);
        
        // 安全のため、全てのモーターを停止させるゼロの値を返す
        return Twist{0.0f, 0.0f, 0.0f};
    }
    // ==========================================================

    return Twist{(float)x_output, (float)y_output, (float)angle_output};
}

TwistAccel BehaviorController::calculateTargetAccel(){
    double ax_output;
    double ay_output;
    double angular_acceleratior_output;

    vx_pid_controller.setGain({10.0f,0.0f,0.0f,1});
    vy_pid_controller.setGain({10.0f,0.0f,0.0f,1});
    omega_pid_controller.setGain({10.0f,0.0f,0.0f,1});

    ax_output = vx_pid_controller.calculate(target_velocity_x - current_velocity_x);
    ay_output = vy_pid_controller.calculate(target_velocity_y - current_velocity_y);
    angular_acceleratior_output = omega_pid_controller.calculate(target_velocity_angle - current_velocity_angle);

    extern int log_cnt;
    if (log_cnt == 0) {
        printf("ax_output:%f, %f\n", target_velocity_x, current_velocity_x);
        printf("ay_output:%f, %f\n", target_velocity_y, current_velocity_y);
        printf("angular_acceleratior_output:%f, %f\n", target_velocity_angle, current_velocity_angle);
        printf("cal_vel:%f, %f, %f\n", ax_output, ay_output, angular_acceleratior_output);
    }

    return TwistAccel{(float)ax_output, (float)ay_output, (float)angular_acceleratior_output };
}


/**
 * @brief 計算された目標速度をモーターコントローラに設定します。
 * この関数を周期的に呼び出すことで、ロボットの制御を行います。
 */
void BehaviorController::setMotor(){
    // 制御モードに応じて計算された目標速度(Twist)を取得
    Twist target_twist = calculateTargetVelocity();
    
    // OmniWheelVelocityControllerに目標速度をセット
    // これにより、内部で各モーターの速度PID制御が実行されます
    wheel_v_controller.setTargetTwist(target_twist);
}

void BehaviorController::stop(){
    FrontMotor.setDuty(0.0f);
    RearLeftMotor.setDuty(0.0f);
    RearRightMotor.setDuty(0.0f);
}

void BehaviorController::test(){
    FrontMotor.setDuty(0.5f);
    RearLeftMotor.setDuty(0.5f);
    RearRightMotor.setDuty(0.5f);
    wait_us(1000000);
}


void BehaviorController::updateFromStateEstimator(const StateEstimator& state) {
    Pose2D pose = state.getRelativePosition();
    auto vel = state.getVelocity();
    auto acc = state.getAcceleration();
    double ang = pose.theta;
    double ang_vel = state.getAngularVelocity();

    setPosition(pose.x, pose.y);
    setVelocity(vel.first, vel.second);
    setAcceleration(acc.first, acc.second);
    setAngle(ang);
    setAngularVelocity(ang_vel);
}

void BehaviorController::setPGain(float p) {
    x_pid_controller.setP(p);
    y_pid_controller.setP(p);
    angle_pid_controller.setP(p);
}
