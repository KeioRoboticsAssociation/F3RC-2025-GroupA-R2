#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "Encoder.hpp" // (ご自身のEncoderライブラリのヘッダ名に置き換えてください)
#include "Imu.hpp"     // (ご自身のImuライブラリのヘッダ名に置き換えてください)

class Odometry {
public:
    /**
     * @brief コンストラクタ
     * @param encoder_x 前後方向の移動を計測する測定輪のエンコーダ
     * @param encoder_y 左右方向の移動を計測する測定輪のエンコーダ
     * @param imu ロボットの姿勢を計測するIMU
     */
    Odometry(Encoder& encoder_x, Encoder& encoder_y, Imu& imu);

    /**
     * @brief 自己位置推定を更新する関数。メインループで定期的に呼び出す。
     */
    void update();
    
    /**
     * @brief 現在の自己位置をリセットする
     * @param x 新しいX座標
     * @param y 新しいY座標
     * @param theta_rad 新しい向き (ラジアン)
     */
    void reset(float x = 0.0f, float y = 0.0f, float theta_rad = 0.0f);

    // --- ゲッター関数 ---
    float getX() const { return m_x; }
    float getY() const { return m_y; }
    float getThetaRad() const { return m_theta; }
    float getThetaDeg() const;

private:
    // 参照メンバー
    Encoder& m_encoder_x;
    Encoder& m_encoder_y;
    Imu& m_imu;

    // ロボットのグローバル座標と向き
    float m_x;
    float m_y;
    float m_theta; // [rad]

    // 前回のエンコーダ値
    long m_prev_count_x;
    long m_prev_count_y;

    // 物理パラメータ
    float m_meters_per_count;
};

#endif // ODOMETRY_HPP
