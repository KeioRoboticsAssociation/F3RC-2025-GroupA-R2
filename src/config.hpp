#pragma once
#include <PinNames.h>

namespace DigitalOutPins
{
    constexpr PinName OMUNI_MOTOR1_DIR  = PA_4 ;     // 左前方オムニホイールDC1の方向
    constexpr PinName OMUNI_MOTOR2_DIR  = PB_0 ;     // 右前方オムニホイールDC2の方向
    constexpr PinName OMUNI_MOTOR3_DIR  = PB_15;     // 後方オムニホイールDC4の方向

    constexpr PinName PSEUDO_SERVO1_DIR = PB_1 ;     // 疑似サーボ(石炭上下)用DC3の方向
    constexpr PinName PSEUDO_SERVO2_DIR = PB_8 ;     // 疑似サーボ(石油上下)用DC5の方向
    constexpr PinName PSEUDO_SERVO3_DIR = PB_9 ;     // 疑似サーボ(石油掴む)用DC6の方向
}



namespace PwmOutPins
{
    constexpr PinName OMUNI_MOTOR1_PWM  = PA_0 ;     // オムニホイールDC1のPWM
    constexpr PinName OMUNI_MOTOR2_PWM  = PA_1 ;     // オムニホイールDC2のPWM
    constexpr PinName OMUNI_MOTOR3_PWM  = PB_2 ;     // オムニホイールDC3のPWM

    constexpr PinName PSEUDO_SERVO1_PWM = PC_7 ;     // 疑似サーボ用DC1のPWM 
    constexpr PinName PSEUDO_SERVO2_PWM = PC_8 ;     // 疑似サーボ用DC2のPWM 
    constexpr PinName PSEUDO_SERVO3_PWM = PC_9 ;     // 疑似サーボ用DC3のPWM

    constexpr PinName SERVO_PWM         = PA_10;     // サーボ(石炭掴む)用PWM
}



namespace InterruptInPins
{
    constexpr PinName MEASURING_ENCODER1_A      = PC_6 ;     // エンコーダー1：測定輪エンコーダ1(前後)のA相
    constexpr PinName MEASURING_ENCODER2_A      = PB_6 ;     // エンコーダー2：測定輪エンコーダ2(左右)のA相

    constexpr PinName OMUNI_ENCODER1_A          = PA_9 ;     // エンコーダー3：オムニ(左前方)用エンコーダー1のA相
    constexpr PinName OMUNI_ENCODER2_A          = PB_13;     // エンコーダー4：オムニ(右前方)用エンコーダー2のA相
    constexpr PinName OMUNI_ENCODER3_A          = PC_12;     // エンコーダー5：オムニ(後方)用エンコーダー3のA相

    constexpr PinName PSEUDO_SERVO_ENCODER1_A   = PC_8 ;     // エンコーダー6：疑似サーボ(石炭上下)用エンコーダー1のA相
    constexpr PinName PSEUDO_SERVO_ENCODER2_A   = PA_12;     // エンコーダー7：疑似サーボ(石油上下)用エンコーダー2のA相
    constexpr PinName PSEUDO_SERVO_ENCODER3_A   = PB_2 ;     // エンコーダー8：疑似サーボ(石油掴む)用エンコーダー3のA相
}



namespace DigitalInPins
{
    constexpr PinName MEASURING_ENCODER1_B      = PC_12;     // エンコーダー1：測定輪エンコーダ1()のB相
    constexpr PinName MEASURING_ENCODER2_B      = PD_2 ;     // エンコーダー2：測定輪エンコーダ2()のB相

    constexpr PinName OMUNI_ENCODER1_B          = PC_3 ;     // エンコーダー3：オムニ用(左前方)エンコーダー1のB相
    constexpr PinName OMUNI_ENCODER2_B          = PC_1 ;     // エンコーダー4：オムニ用(右前方)エンコーダー2のB相
    constexpr PinName OMUNI_ENCODER3_B          = PB_8 ;     // エンコーダー5：オムニ用(後方)エンコーダー3のB相

    constexpr PinName PSEUDO_SERVO_ENCODER1_B   = PC_6 ;     // エンコーダー6：疑似サーボ(石炭上下)用エンコーダー1のB相
    constexpr PinName PSEUDO_SERVO_ENCODER2_B   = PA_11;     // エンコーダー7：疑似サーボ(石油上下)用エンコーダー2のB相
    constexpr PinName PSEUDO_SERVO_ENCODER3_B   = PB_1 ;     // エンコーダー8：疑似サーボ(石油掴む)用エンコーダー3のB相
}



namespace LimitSwitchPins
{
    constexpr PinName FRONT_LIMIT           = PA_15;
    constexpr PinName SIDE_LIMIT            = PA_11;

    constexpr PinName PSEUDO_SERVO_LIMIT1   = PB_12;        // リミットスイッチ1：疑似サーボ(石炭上下)用リミットスイッチ
    constexpr PinName PSEUDO_SERVO_LIMIT2   = PC_4 ;        // リミットスイッチ2：疑似サーボ(石油上下)用リミットスイッチ
    constexpr PinName PSEUDO_SERVO_LIMIT3   = PA_6 ;        // リミットスイッチ3：疑似サーボ(石炭掴む)用リミットスイッチ
}



namespace PinsForSensor
{
    constexpr PinName IMU_RX    = PC_10 ;
    constexpr PinName IMU_TX    = PC_11 ;

    constexpr PinName IMU_SDA   = PC_12 ;
    constexpr PinName IMU_SCL   = PB_10 ;

    constexpr PinName TOF1      = PC_1 ;       // 前方TOFセンサー
    constexpr PinName TOF2      = PC_0 ;       // 側方TOFセンサー
}