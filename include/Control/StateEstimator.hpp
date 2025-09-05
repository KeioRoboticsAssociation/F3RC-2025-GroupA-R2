#ifndef F3RC_2025_GROUPA_R2_FEEDBACK_HPP
#define F3RC_2025_GROUPA_R2_FEEDBACK_HPP

#include <utility>
#include <vector>
#include <string>
#include "Provider/Database.hpp" // Databaseのメソッドを直接呼び出すため、完全な定義をインクルード
#include "Provider/ImuOdometry.hpp"
#include "Provider/WheelOdometry.hpp"
#include "Common/Style.hpp"

struct TOFStatus {
    bool front;
    bool side;
};

struct LimitSwitchStatus {
    bool front;
    bool side;
};



class StateEstimator {
public:
    StateEstimator(Database& db, WheelOdometry& wheel_odom, ImuOdometry& imu_odom);

    //=====================================================================================
    //　 以下、使用するメソッド一覧        ※これらメソッドは原則ループ内で呼び出し
    //=====================================================================================

    /**
     * @brief センサー情報・Odometryの更新を行い、自己位置推定を更新する
     * @param dt 更新周期[s]
     */
    void update(double dt);

    /**
     * @brief ワールド座標系での絶対姿勢を取得
     * @return Pose2D (x, y, theta)
     */
    Pose2D getAbsolutePosition() const;

    /**
     * @brief 相対座標系での姿勢を取得（指定フレーム番号、未指定の場合は現在座標系）
     * @param frame_num 座標系番号
     * @return Pose2D (x, y, theta)
     */
    Pose2D getRelativePosition(int frame_num = -1) const;

    /**
     * @brief 融合済みの速度を取得
     * @return std::pair<double,double> (vx, vy) [m/s]
     */
    std::pair<double,double> getVelocity() const;

    /**
     * @brief 融合済みの加速度を取得（WheelOdometry由来）
     * @return std::pair<double,double> (ax, ay) [m/s^2]
     */
    std::pair<double,double> getAcceleration() const;

    /**
     * @brief IMU由来の角速度を取得
     * @return double z方向角速度 [rad/s]
     */
    double getAngularVelocity() const;

    /**
     * @brief ToFセンサーの状態を取得
     * @return TOFStatus 前方・側方の検知状態
     */
    TOFStatus getTOFStatus() const;

    /**
     * @brief リミットスイッチの状態を取得
     * @return LimitSwitchStatus 前方・側方の押下状態
     */
    LimitSwitchStatus getLimitSwitchStatus() const;

    /**
     * @brief 一回微分（速度）を取得
     * @return std::pair<double,double> (dx/dt, dy/dt)
     */
    std::pair<double,double> getOneDerivative() const;

    /**
     * @brief 二回微分（加速度）を取得
     * @return std::pair<double,double> (d^2x/dt^2, d^2y/dt^2)
     */
    std::pair<double,double> getTwoDerivative() const;

    /**
     * @brief 現在の座標系番号を取得
     * @return int 座標系番号
     */
    int getCoordinateSystemNum() const;

    /**
     * @brief 現在の自己位置を原点として新しい座標系を追加（リセット）
     */
    void resetCoordinateSystem();


    // void setAngularAcceleration(double aa);
    // void setIRSensorRaw(float raw);

    // float getTheta() const { return current_theta; }
    // float getOmega() const { return current_omega; }
    float getAlpha() const { return (current_omega - prev_omega); }

private:
    // Databaseへの参照
    Database& db_;// getAcceleration()のために保持
    // 測定輪の速度・加速度を担当するクラス
    WheelOdometry& wheel_odom_;
    // 速度・位置・姿勢の計算を担当するクラス
    ImuOdometry& imu_odom_;

    // 融合済み状態
    Pose2D fused_pose_;                            ///< 融合済み絶対姿勢
    std::pair<double,double> fused_velocity_;      ///< 融合済み速度
    std::pair<double,double> fused_acceleration_;  ///< 融合済み加速度
    std::pair<double,double> one_derivative_;      ///< 1回微分
    std::pair<double,double> two_derivative_;      ///< 2回微分
    // ワールド座標系から見た各座標系の原点
    std::vector<Pose2D> coordinate_systems_;
    int current_coordinate_system_num;


    // 回転運動に関する追加メンバ
    float current_theta = 0.0f;
    float current_omega = 0.0f;
    float prev_omega = 0.0f;
};


#endif // F3RC_2025_GROUPA_R2_FEEDBACK_HPP