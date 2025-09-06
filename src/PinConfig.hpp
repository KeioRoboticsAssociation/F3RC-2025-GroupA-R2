#ifndef PINCONFIG_HPP
#define PINCONFIG_HPP

#include <PinNames.h>

namespace PinConfig {
    constexpr PinName OMNI1_PWM = PA_0; // DC1
    constexpr PinName OMNI1_DIR = PA_4;
    constexpr PinName OMNI1_ENC_A = PA_9; // ENC3
    constexpr PinName OMNI1_ENC_B = PA_8;

    constexpr PinName OMNI2_PWM = PA_1; // DC2
    constexpr PinName OMNI2_DIR = PB_0;
    constexpr PinName OMNI2_ENC_A = PB_13; // ENC4
    constexpr PinName OMNI2_ENC_B = PB_14;

    constexpr PinName OMNI3_PWM = PB_2; // DC4
    constexpr PinName OMNI3_DIR = PB_15;
    constexpr PinName OMNI3_ENC_A = PC_10; // ENC5
    constexpr PinName OMNI3_ENC_B = PD_2;

    constexpr PinName ODO_X_ENC_A = PC_6; // ENC1: 測定輪X方向エンコーダA相
    constexpr PinName ODO_X_ENC_B = PA_7; // ENC1: 測定輪X方向エンコーダB相
    constexpr PinName ODO_Y_ENC_A = PB_7; // ENC2: 測定輪Y方向エンコーダA相
    constexpr PinName ODO_Y_ENC_B = PB_6; // ENC2: 測定輪Y方向エンコーダB相
    constexpr PinName IMU_SDA = PC_12;
    constexpr PinName IMU_SCL = PB_10;

    constexpr PinName ARM1_LIFT_PWM = PC_7; // DC3: 石炭アーム昇降用モーターPWM
    constexpr PinName ARM1_LIFT_DIR = PB_1; // DC3: 石炭アーム昇降用モーター方向
    constexpr PinName ARM1_LIFT_ENC_A = PC_2; // ENC6: 石炭アーム昇降用エンコーダA相
    constexpr PinName ARM1_LIFT_ENC_B = PC_3; // ENC6: 石炭アーム昇降用エンコーダB相
    constexpr PinName ARM1_GRIPPER_PWM = PA_10; // 石炭アーム掴み用サーボPWM
}



namespace DigitalOutPins
{
    constexpr PinName PSEUDO_SERVO2_DIR = PB_8 ;     // 疑似サーボ(石油上下)用DC5の方向
    constexpr PinName PSEUDO_SERVO3_DIR = PB_9 ;     // 疑似サーボ(石油掴む)用DC6の方向
}

namespace PwmOutPins
{
    constexpr PinName PSEUDO_SERVO2_PWM = PC_8 ;     // 疑似サーボ(石油上下)用DC5のPWM 
    constexpr PinName PSEUDO_SERVO3_PWM = PC_9 ;     // 疑似サーボ(石油掴む)用DC6のPWM

    constexpr PinName SERVO_PWM         = PA_10;     // サーボ(石炭掴む)用PWM
}

namespace InterruptInPins
{
    constexpr PinName PSEUDO_SERVO_ENCODER2_A   = PB_5 ;     // エンコーダー7：疑似サーボ(石油上下)用エンコーダー2のA相
    constexpr PinName PSEUDO_SERVO_ENCODER3_A   = PA_12 ;     // エンコーダー8：疑似サーボ(石油掴む)用エンコーダー3のA相
}

namespace DigitalInPins
{
    constexpr PinName PSEUDO_SERVO_ENCODER2_B   = PB_4;     // エンコーダー7：疑似サーボ(石油上下)用エンコーダー2のB相
    constexpr PinName PSEUDO_SERVO_ENCODER3_B   = PC_5;     // エンコーダー8：疑似サーボ(石油掴む)用エンコーダー3のB相
}

namespace PinsForSensor
{
    constexpr PinName TOF1      = PC_1 ;       // 前方TOFセンサー
    constexpr PinName TOF2      = PC_0 ;       // 側方TOFセンサー
}

#endif // PINCONFIG_HPP
