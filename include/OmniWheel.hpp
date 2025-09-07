#ifndef OMNI_WHEEL_HPP
#define OMNI_WHEEL_HPP

#include "DCMotor.hpp"
#include "Encoder.hpp"
#include "RobotConfig.hpp"

class OmniWheel {
public:
    OmniWheel(DCMotor& motor1, Encoder& encoder1,
              DCMotor& motor2, Encoder& encoder2,
              DCMotor& motor3, Encoder& encoder3);

    void setParams(float wheel_radius, float body_radius, float counts_per_rev,
                   float kp, float ki, float kd);

    void move(float vx, float vy, float omega);

    void update(float dt);

    void stop();

    struct MotorDuties {
        float duty1;
        float duty2;
        float duty3;
    };
    MotorDuties getMotorDuties() const;

private:
    // 制御対象
    DCMotor& m_motor1;
    Encoder& m_encoder1;
    DCMotor& m_motor2;
    Encoder& m_encoder2;
    DCMotor& m_motor3;
    Encoder& m_encoder3;

    // 物理パラメータ
    float m_wheel_radius;
    float m_body_radius;
    float m_counts_per_rev;

    // PID制御用パラメータ
    float m_kp, m_ki, m_kd;
    float m_integral_limit; // ★追加: 積分値の上限
    float m_target_speed1, m_target_speed2, m_target_speed3;
    float m_integral1, m_integral2, m_integral3;
    float m_prev_error1, m_prev_error2, m_prev_error3;
    
    int m_prev_count1, m_prev_count2, m_prev_count3;
};

#endif // OMNI_WHEEL_HPP
