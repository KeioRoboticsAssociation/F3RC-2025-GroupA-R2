#ifndef IMU_HPP
#define IMU_HPP

#include "mbed.h"

/**
 * @brief 3軸データを格納する構造体
 */
struct Vector3 {
    double x;
    double y;
    double z;
    
    Vector3() : x(0.0), y(0.0), z(0.0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief BNO055 IMUセンサードライバクラス
 * 
 * 9軸フュージョンモード（NDOFモード）を使用して、姿勢角、角速度、
 * 線形加速度、キャリブレーション状態を提供する。
 * 100Hzでのデータ更新により、リアルタイム性を確保する。
 */
class Imu {
public:
    /**
     * @brief コンストラクタ
     * @param sda I2CのSDAピン
     * @param scl I2CのSCLピン
     */
    Imu(PinName sda, PinName scl);
    
    /**
     * @brief デストラクタ
     */
    ~Imu();
    
    /**
     * @brief BNO055の初期化とデータ更新処理を開始する
     * @return true: 初期化成功, false: 初期化失敗
     */
    bool init();
    
    /**
     * @brief 機体のヨー角（Z軸周りの回転角度）を取得する
     * @return ヨー角 [deg] (-180.0 ~ 180.0)
     */
    double getYaw();
    
    /**
     * @brief 機体のピッチ角（Y軸周りの回転角度）を取得する
     * @return ピッチ角 [deg] (-90.0 ~ 90.0)
     */
    double getPitch();
    
    /**
     * @brief 機体のロール角（X軸周りの回転角度）を取得する
     * @return ロール角 [deg] (-180.0 ~ 180.0)
     */
    double getRoll();
    
    /**
     * @brief 機体の角速度（3軸）を取得する
     * @return 角速度 [deg/s] (x:ロール速度, y:ピッチ速度, z:ヨー速度)
     */
    Vector3 getAngularVelocity();
    
    /**
     * @brief 機体の線形加速度（3軸、重力除去済み）を取得する
     * @return 線形加速度 [m/s^2]
     */
    Vector3 getLinearAcceleration();
    
    /**
     * @brief センサが完全にキャリブレーションされているかを確認する
     * @return true: キャリブレーション完了, false: キャリブレーション未完了
     */
    bool isCalibrated();
    
    /**
     * @brief 現在のヨー角を0度とするオフセットを設定する
     */
    void resetYaw();

private:
    // BNO055レジスタアドレス定義
    static const uint8_t BNO055_ADDRESS = 0x28 << 1;  // I2Cアドレス（7bitを8bitに変換）
    
    // レジスタアドレス
    static const uint8_t BNO055_CHIP_ID_ADDR = 0x00;
    static const uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
    static const uint8_t BNO055_PWR_MODE_ADDR = 0x3E;
    static const uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3F;
    static const uint8_t BNO055_AXIS_MAP_CONFIG_ADDR = 0x41;
    static const uint8_t BNO055_AXIS_MAP_SIGN_ADDR = 0x42;
    
    // データレジスタアドレス
    static const uint8_t BNO055_EULER_H_LSB_ADDR = 0x1A;
    static const uint8_t BNO055_GYRO_DATA_X_LSB_ADDR = 0x14;
    static const uint8_t BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;
    static const uint8_t BNO055_CALIB_STAT_ADDR = 0x35;
    
    // 動作モード定義
    static const uint8_t OPERATION_MODE_CONFIG = 0x00;
    static const uint8_t OPERATION_MODE_NDOF = 0x0C;
    
    // 期待されるチップID
    static const uint8_t BNO055_ID = 0xA0;
    
    // データ更新周期
    static const int UPDATE_FREQUENCY_HZ = 100;
    static const int UPDATE_INTERVAL_US = 1000000 / UPDATE_FREQUENCY_HZ;
    
    I2C i2c_;                    // I2C通信オブジェクト
    Ticker data_update_ticker_;  // データ更新用タイマー
    
    // センサーデータを保持するメンバ変数
    double yaw_;           // ヨー角 [deg]
    double pitch_;         // ピッチ角 [deg]
    double roll_;          // ロール角 [deg]
    Vector3 angular_velocity_;    // 角速度 [deg/s]
    Vector3 linear_acceleration_; // 線形加速度 [m/s^2]
    uint8_t calibration_status_;  // キャリブレーション状態
    
    double yaw_offset_;    // ヨー角のオフセット値
    
    // 排他制御用ミューテックス
    Mutex data_mutex_;
    
    /**
     * @brief レジスタから1バイト読み取り
     * @param reg_addr レジスタアドレス
     * @return 読み取った値
     */
    uint8_t readByte(uint8_t reg_addr);
    
    /**
     * @brief レジスタに1バイト書き込み
     * @param reg_addr レジスタアドレス
     * @param data 書き込むデータ
     * @return true: 成功, false: 失敗
     */
    bool writeByte(uint8_t reg_addr, uint8_t data);
    
    /**
     * @brief レジスタから複数バイト読み取り
     * @param reg_addr 開始レジスタアドレス
     * @param buffer 読み取りデータを格納するバッファ
     * @param length 読み取りバイト数
     * @return true: 成功, false: 失敗
     */
    bool readBytes(uint8_t reg_addr, uint8_t* buffer, int length);
    
    /**
     * @brief 16bit符号付き整数に変換
     * @param lsb 下位バイト
     * @param msb 上位バイト
     * @return 16bit符号付き整数
     */
    int16_t combineBytes(uint8_t lsb, uint8_t msb);
    
    /**
     * @brief データ更新処理（Ticker割り込みから呼び出し）
     */
    void updateData();
    
    /**
     * @brief BNO055の動作モードを設定
     * @param mode 動作モード
     * @return true: 成功, false: 失敗
     */
    bool setOperationMode(uint8_t mode);
    
    /**
     * @brief 軸マッピングを設定（座標系の統一）
     * @return true: 成功, false: 失敗
     */
    bool setupAxisMapping();
};

#endif // IMU_HPP