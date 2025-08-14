#include "Driver/Imu.hpp"

Imu::Imu(PinName sda, PinName scl) 
    : i2c_(sda, scl)
    , yaw_(0.0)
    , pitch_(0.0)
    , roll_(0.0)
    , angular_velocity_()
    , linear_acceleration_()
    , calibration_status_(0)
    , yaw_offset_(0.0)
{
    // I2C通信速度を400kHzに設定
    i2c_.frequency(400000);
}

Imu::~Imu() {
    // Tickerを停止
    data_update_ticker_.detach();
}

bool Imu::init() {
    // BNO055との通信確認（チップIDの確認）
    uint8_t chip_id = readByte(BNO055_CHIP_ID_ADDR);
    if (chip_id != BNO055_ID) {
        printf("BNO055 not found. Chip ID: 0x%02X (expected: 0x%02X)\n", chip_id, BNO055_ID);
        return false;
    }
    
    printf("BNO055 detected. Chip ID: 0x%02X\n", chip_id);
    
    // コンフィグモードに設定
    if (!setOperationMode(OPERATION_MODE_CONFIG)) {
        printf("Failed to set config mode\n");
        return false;
    }
    
    // システムリセット
    writeByte(BNO055_SYS_TRIGGER_ADDR, 0x20);
    ThisThread::sleep_for(650ms);  // リセット待機
    
    // 再度チップIDを確認
    chip_id = readByte(BNO055_CHIP_ID_ADDR);
    if (chip_id != BNO055_ID) {
        printf("BNO055 communication failed after reset\n");
        return false;
    }
    
    // パワーモードをNORMALに設定
    if (!writeByte(BNO055_PWR_MODE_ADDR, 0x00)) {
        printf("Failed to set power mode\n");
        return false;
    }
    ThisThread::sleep_for(10ms);
    
    // 軸マッピングを設定
    if (!setupAxisMapping()) {
        printf("Failed to setup axis mapping\n");
        return false;
    }
    
    // NDOFモードに設定
    if (!setOperationMode(OPERATION_MODE_NDOF)) {
        printf("Failed to set NDOF mode\n");
        return false;
    }
    
    // データ更新処理を開始（100Hz）
    data_update_ticker_.attach(callback(this, &Imu::updateData), 
                               chrono::microseconds(UPDATE_INTERVAL_US));
    
    printf("BNO055 initialization completed\n");
    return true;
}

double Imu::getYaw() {
    data_mutex_.lock();
    double yaw = yaw_ - yaw_offset_;
    data_mutex_.unlock();
    
    // -180度から180度の範囲に正規化
    while (yaw > 180.0) yaw -= 360.0;
    while (yaw <= -180.0) yaw += 360.0;
    
    return yaw;
}

double Imu::getPitch() {
    data_mutex_.lock();
    double pitch = pitch_;
    data_mutex_.unlock();
    return pitch;
}

double Imu::getRoll() {
    data_mutex_.lock();
    double roll = roll_;
    data_mutex_.unlock();
    return roll;
}

Vector3 Imu::getAngularVelocity() {
    data_mutex_.lock();
    Vector3 angular_velocity = angular_velocity_;
    data_mutex_.unlock();
    return angular_velocity;
}

Vector3 Imu::getLinearAcceleration() {
    data_mutex_.lock();
    Vector3 linear_acceleration = linear_acceleration_;
    data_mutex_.unlock();
    return linear_acceleration;
}

bool Imu::isCalibrated() {
    data_mutex_.lock();
    uint8_t status = calibration_status_;
    data_mutex_.unlock();
    
    // 各センサーのキャリブレーションレベルを抽出
    uint8_t sys_cal = (status >> 6) & 0x03;  // システム
    uint8_t gyr_cal = (status >> 4) & 0x03;  // ジャイロ
    uint8_t acc_cal = (status >> 2) & 0x03;  // 加速度
    uint8_t mag_cal = status & 0x03;         // 磁気
    
    // 全てのキャリブレーションレベルが3（最大値）の場合にtrueを返す
    return (sys_cal == 3) && (gyr_cal == 3) && (acc_cal == 3) && (mag_cal == 3);
}

void Imu::resetYaw() {
    data_mutex_.lock();
    yaw_offset_ = yaw_;
    data_mutex_.unlock();
    printf("Yaw offset reset to: %.2f degrees\n", yaw_offset_);
}

uint8_t Imu::readByte(uint8_t reg_addr) {
    char cmd[1] = {reg_addr};
    char data[1];
    
    if (i2c_.write(BNO055_ADDRESS, cmd, 1, true) != 0) {
        return 0;
    }
    
    if (i2c_.read(BNO055_ADDRESS, data, 1) != 0) {
        return 0;
    }
    
    return data[0];
}

bool Imu::writeByte(uint8_t reg_addr, uint8_t data) {
    char cmd[2] = {reg_addr, data};
    return i2c_.write(BNO055_ADDRESS, cmd, 2) == 0;
}

bool Imu::readBytes(uint8_t reg_addr, uint8_t* buffer, int length) {
    char cmd[1] = {reg_addr};
    
    if (i2c_.write(BNO055_ADDRESS, cmd, 1, true) != 0) {
        return false;
    }
    
    return i2c_.read(BNO055_ADDRESS, (char*)buffer, length) == 0;
}

int16_t Imu::combineBytes(uint8_t lsb, uint8_t msb) {
    return (int16_t)((msb << 8) | lsb);
}

void Imu::updateData() {
    uint8_t buffer[18];  // オイラー角(6) + 角速度(6) + 線形加速度(6)のデータ
    
    // オイラー角データを読み取り (6バイト)
    if (readBytes(BNO055_EULER_H_LSB_ADDR, buffer, 6)) {
        int16_t yaw_raw = combineBytes(buffer[0], buffer[1]);
        int16_t roll_raw = combineBytes(buffer[2], buffer[3]);
        int16_t pitch_raw = combineBytes(buffer[4], buffer[5]);
        
        data_mutex_.lock();
        yaw_ = yaw_raw / 16.0;      // LSB = 1/16度
        roll_ = roll_raw / 16.0;
        pitch_ = pitch_raw / 16.0;
        data_mutex_.unlock();
    }
    
    // 角速度データを読み取り (6バイト)
    if (readBytes(BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6)) {
        int16_t x_raw = combineBytes(buffer[0], buffer[1]);
        int16_t y_raw = combineBytes(buffer[2], buffer[3]);
        int16_t z_raw = combineBytes(buffer[4], buffer[5]);
        
        data_mutex_.lock();
        angular_velocity_.x = x_raw / 16.0;  // LSB = 1/16 deg/s
        angular_velocity_.y = y_raw / 16.0;
        angular_velocity_.z = z_raw / 16.0;
        data_mutex_.unlock();
    }
    
    // 線形加速度データを読み取り (6バイト)
    if (readBytes(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6)) {
        int16_t x_raw = combineBytes(buffer[0], buffer[1]);
        int16_t y_raw = combineBytes(buffer[2], buffer[3]);
        int16_t z_raw = combineBytes(buffer[4], buffer[5]);
        
        data_mutex_.lock();
        linear_acceleration_.x = x_raw / 100.0;  // LSB = 1m/s² = 100 LSB
        linear_acceleration_.y = y_raw / 100.0;
        linear_acceleration_.z = z_raw / 100.0;
        data_mutex_.unlock();
    }
    
    // キャリブレーション状態を読み取り
    data_mutex_.lock();
    calibration_status_ = readByte(BNO055_CALIB_STAT_ADDR);
    data_mutex_.unlock();
}

bool Imu::setOperationMode(uint8_t mode) {
    if (!writeByte(BNO055_OPR_MODE_ADDR, mode)) {
        return false;
    }
    
    // モード変更待機時間
    if (mode == OPERATION_MODE_CONFIG) {
        ThisThread::sleep_for(19ms);  // コンフィグモードへの変更は19ms
    } else {
        ThisThread::sleep_for(7ms);   // その他のモードは7ms
    }
    
    return true;
}

bool Imu::setupAxisMapping() {
    // デフォルトの軸マッピングを使用（必要に応じて変更）
    // X軸:前方, Y軸:左方, Z軸:上方
    
    // 軸マッピング設定 (デフォルト: X=X, Y=Y, Z=Z)
    if (!writeByte(BNO055_AXIS_MAP_CONFIG_ADDR, 0x24)) {
        return false;
    }
    
    // 軸の方向設定 (デフォルト: 全て正方向)
    if (!writeByte(BNO055_AXIS_MAP_SIGN_ADDR, 0x00)) {
        return false;
    }
    
    return true;
}