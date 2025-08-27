#pragma once
#include "mbed.h"

// 3次元ベクトル用のシンプルな構造体
struct Vector3
{
    double x, y, z;
};

class Imu
{
public:
    Imu(PinName sda, PinName scl);
    ~Imu();

    bool init();
    double getYaw();
    double getPitch();
    double getRoll();
    Vector3 getAngularVelocity();
    Vector3 getLinearAcceleration();
    bool isCalibrated();
    void resetYaw();

private:
    // I2C通信とデータ
    I2C i2c_;
    double yaw_, pitch_, roll_;
    Vector3 angular_velocity_;
    Vector3 linear_acceleration_;
    uint8_t calibration_status_;
    double yaw_offset_;

    // 同期制御用
    Mutex data_mutex_;
    Ticker data_update_ticker_;

    // データ更新をISRコンテキストから分離するためのスレッドとEventFlags
    Thread data_update_thread_;
    EventFlags event_flags_;
    static const uint32_t UPDATE_SIGNAL = 0x1;

    // データ更新用の関数
    void ticker_isr();
    void update_thread_worker();

    // I2C通信ヘルパー関数
    uint8_t readByte(uint8_t reg_addr);
    bool writeByte(uint8_t reg_addr, uint8_t data);
    bool readBytes(uint8_t reg_addr, uint8_t *buffer, int length);
    int16_t combineBytes(uint8_t lsb, uint8_t msb);

    // 設定用関数
    bool setOperationMode(uint8_t mode);
    bool setupAxisMapping();
};
