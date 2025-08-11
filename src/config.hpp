#pragma once
#include <PinNames.h>

namespace DigitalOutPins
{
    constexpr PinName OMUNI_MOTOR1_DIR  = PB_10;     // オムニホイールDC1の方向
    constexpr PinName OMUNI_MOTOR2_DIR  = PC_7 ;     // オムニホイールDC2の方向
    constexpr PinName OMUNI_MOTOR3_DIR  = PB_4 ;     // オムニホイールDC3の方向

    constexpr PinName PSEUDO_SERVO1_DIR = PB_9 ;     // 疑似サーボ用DC1の方向
    constexpr PinName PSEUDO_SERVO2_DIR = PB_6 ;     // 疑似サーボ用DC2の方向
    constexpr PinName PSEUDO_SERVO3_DIR = PA_4 ;     // 疑似サーボ用DC3の方向
}



namespace PwmOutPins
{
    constexpr PinName OMUNI_MOTOR1_PWM  = PA_8 ;     // オムニホイールDC1のPWM (Tim1)
    constexpr PinName OMUNI_MOTOR2_PWM  = PA_9 ;     // オムニホイールDC2のPWM (Tim1)
    constexpr PinName OMUNI_MOTOR3_PWM  = PA_10;     // オムニホイールDC3のPWM (Tim1)

    constexpr PinName PSEUDO_SERVO1_PWM = PA_6 ;     // 疑似サーボ用DC1のPWM (Tim3)
    constexpr PinName PSEUDO_SERVO2_PWM = PA_7 ;     // 疑似サーボ用DC2のPWM (Tim3)
    constexpr PinName PSEUDO_SERVO3_PWM = PB_0 ;     // 疑似サーボ用DC3のPWM (Tim3)

    constexpr PinName SERVO_PWM         = PA_15;     // サーボ用PWM (Tim2)
}



namespace InterruptInPins
{
    constexpr PinName MEASURING_ENCODER1_A      = PC_10;     // エンコーダー1：測定輪エンコーダ1のA相
    constexpr PinName MEASURING_ENCODER2_A      = PC_11;     // エンコーダー2：測定輪エンコーダ2のA相

    constexpr PinName OMUNI_ENCODER1_A          = PC_2 ;     // エンコーダー3：オムニ用エンコーダー1のA相
    constexpr PinName OMUNI_ENCODER2_A          = PC_0 ;     // エンコーダー4：オムニ用エンコーダー2のA相
    constexpr PinName OMUNI_ENCODER3_A          = PC_9 ;     // エンコーダー5：オムニ用エンコーダー3のA相

    constexpr PinName PSEUDO_SERVO_ENCODER1_A   = PC_8 ;     // エンコーダー6：疑似サーボ用エンコーダー1のA相
    constexpr PinName PSEUDO_SERVO_ENCODER2_A   = PA_12;     // エンコーダー7：疑似サーボ用エンコーダー2のA相
    constexpr PinName PSEUDO_SERVO_ENCODER3_A   = PB_2 ;     // エンコーダー8：疑似サーボ用エンコーダー3のA相
}



namespace DigitalInPins
{
    constexpr PinName MEASURING_ENCODER1_B      = PC_12;     // エンコーダー1：測定輪エンコーダ1のB相
    constexpr PinName MEASURING_ENCODER2_B      = PD_2 ;     // エンコーダー2：測定輪エンコーダ2のB相

    constexpr PinName OMUNI_ENCODER1_B          = PC_3 ;     // エンコーダー3：オムニ用エンコーダー1のB相
    constexpr PinName OMUNI_ENCODER2_B          = PC_1 ;     // エンコーダー4：オムニ用エンコーダー2のB相
    constexpr PinName OMUNI_ENCODER3_B          = PB_8 ;     // エンコーダー5：オムニ用エンコーダー3のB相

    constexpr PinName PSEUDO_SERVO_ENCODER1_B   = PC_6 ;     // エンコーダー6：疑似サーボ用エンコーダー1のB相
    constexpr PinName PSEUDO_SERVO_ENCODER2_B   = PA_11;     // エンコーダー7：疑似サーボ用エンコーダー2のB相
    constexpr PinName PSEUDO_SERVO_ENCODER3_B   = PB_1 ;     // エンコーダー8：疑似サーボ用エンコーダー3のB相
}



namespace PinsForSensor
{
    constexpr PinName IMU_RX = PA_0 ;
    constexpr PinName IMU_TX = PA_1 ;

    constexpr PinName IR1    = PB_14;
    constexpr PinName IR2    = PB_13;

    constexpr PinName FRONT_LIMIT  = PB_7 ;
    constexpr PinName SIDE_LIMIT   = PB_5 ;

    constexpr PinName PSEUDO_SERVO_LIMIT1 = PC_5 ;
    constexpr PinName PSEUDO_SERVO_LIMIT2 = PB_12;
    constexpr PinName PSEUDO_SERVO_LIMIT3 = PB_15;
}