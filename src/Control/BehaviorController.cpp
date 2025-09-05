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

void BehaviorController::setTargetVelocity(double vx, double vy)
{
    target_velocity_x = vx;
    target_velocity_y = vy;
    xy_target_mode = TargetMode::Velocity;
}

void BehaviorController::setTargetAngle(double theta)
{
    target_angle = theta;
    angle_target_mode = TargetMode::Position;
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

Twist BehaviorController::calculateTargetVelocity()
{
    double x_output;
    double y_output;
    double angle_output;
    x_output = x_pid_controller.calculate(target_pos_x - current_pos_x);
    y_output = y_pid_controller.calculate(target_pos_y - current_pos_y);
    angle_output = angle_pid_controller.calculate(target_angle - current_angle);

    

    //if (xy_target_mode == TargetMode::Position)
    // {
    //     x_output = x_pid_controller.calculate(target_pos_x - current_pos_x);
    //     y_output = y_pid_controller.calculate(target_pos_y - current_pos_y);
    // }
    // else
    // {
    //     x_output = target_velocity_x;
    //     y_output = target_velocity_y;
    // }

    // double angle_output;
    // if (angle_target_mode == TargetMode::Position)
    // {
    //     angle_output = angle_pid_controller.calculate(target_angle - current_angle);
    // }
    // else
    // {
    //     angle_output = target_velocity_angle;
    // }

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


    // printf("error_x:%f error_y:%f error_angle:%f\n",
    //    target_pos_x - current_pos_x,
    //    target_pos_y - current_pos_y,
    //    target_angle - current_angle);

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

    printf("ay_output:%f, %f\n", target_velocity_y, current_velocity_y);
    printf("cal_vel:%f, %f, %f\n", ax_output, ay_output, angular_acceleratior_output);

    return TwistAccel{(float)ax_output, (float)ay_output, (float)angular_acceleratior_output };
}



void BehaviorController::setMotor(){

    // オムニホイールの配置を設定
    // wheel_v_controller(config, {&FrontMotorController, &RearLeftMotorController, &RearRightMotorController})(config, {&FrontMotorController, &RearLeftMotorController, &RearRightMotorController});

    if (xy_target_mode == TargetMode::Position){
        wheel_v_controller.setTargetTwist(calculateTargetVelocity());
    }else if(xy_target_mode == TargetMode::Velocity){
        // 10ms周期
        float dt = 0.01f;
        wheel_v_controller.setTargetTwistAccel(calculateTargetAccel(), dt);
        //ループでの呼び出し前提であることに注意
        wait_us(10000);
    }
    
    printf("前方カウント数：%d\n",FrontEncoder.getCount());
    printf("左後方カウント数：%d\n",RearLeftEncoder.getCount());
    printf("右後方カウント数：%d\n",RearRightEncoder.getCount());

    //std::cout << controller.twistToMotorSpeeds(tw) << std::endl;

    // printf("calculateTargetVelocity().vx:%f\n", calculateTargetVelocity().vx);
    // printf("calculateTargetVelocity().vy:%f\n", calculateTargetVelocity().vy);
    // printf("calculateTargetVelocity().omega:%f\n", calculateTargetVelocity().omega);
    
   
    // printf("twistToMotorSpeeds[0]:%f\n", controller.twistToMotorSpeeds(calculateTargetVelocity())[0]);
    // printf("twistToMotorSpeeds[1]:%f\n", controller.twistToMotorSpeeds(calculateTargetVelocity())[1]);
    // printf("twistToMotorSpeeds[2]:%f\n", controller.twistToMotorSpeeds(calculateTargetVelocity())[2]);


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
    // printf("FrontEncoder:%d\n", FrontEncoder.getCount());
    // printf("RearLeftEncoder:%d\n", RearLeftEncoder.getCount());
    // printf("RearRightEncoder:%d\n", RearRightEncoder.getCount());

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
