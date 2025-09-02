#ifndef F3RC_2025_GROUPA_R2_FEEDBACK_HPP
#define F3RC_2025_GROUPA_R2_FEEDBACK_HPP

#include <utility>
#include <vector>
#include <string>
#include "Provider/Database.hpp" // Databaseのメソッドを直接呼び出すため、完全な定義をインクルード
#include "Provider/ImuOdometry.hpp"


// 2D姿勢（位置と向き）を表す構造体
struct Pose2D {
    double x;      // X座標 [m]
    double y;      // Y座標 [m]  
    double theta;  // 角度（ラジアン、右方0度、反時計回り正）
};

class Feedback {
public:
    Feedback(Database& db, ImuOdometry& odometry_calculator);
    Pose2D getPosition(int frame_num);
    int getCoordinateSystemNum() const;

    std::pair<double, double> getVelocity() const {
    return odometry_calculator_.getVelocity();
    }

    std::pair<double, double> getAcceleration() const {
        return database.getAcceleration();
    }

    double getRotation() const {
    // 角度はVelocityクラスが計算した絶対姿勢から取得
        return odometry_calculator_.getPose().theta;
    }

    double getAngularVelocity() const {
        return database.getAngularVelocity();
    }


    bool getFrontTofStatus() const {
        return database.getFrontTofStatus();
    }

    bool getSideTofStatus() const {
        return database.getSideTofStatus();
    }

    void resetCoordinateSystem();


    void update(double dt);

    void setAbsolutePose(const Pose2D& pose);
    // void setAngularAcceleration(double aa);
    // void setIRSensorRaw(float raw);

private:
    // Databaseへの参照
    Database& database; // getAcceleration()のために保持



    // 速度・位置・姿勢の計算を担当するクラス
    ImuOdometry& odometry_calculator_;
   

    // ワールド座標系から見た各座標系の原点
    std::vector<Pose2D> coordinate_systems;
    int current_coordinate_system_num;
};



#endif // F3RC_2025_GROUPA_R2_FEEDBACK_HPP