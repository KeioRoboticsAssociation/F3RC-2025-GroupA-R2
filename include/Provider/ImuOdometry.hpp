#ifndef F3RC_2025_GROUPA_R2_IMUODOMETRY_HPP
#define F3RC_2025_GROUPA_R2_IMUODOMETRY_HPP

#include "Provider/Database.hpp"

// 2D姿勢（位置と向き）を表す構造体
struct Pose2D {
    double x;      // X座標 [m]
    double y;      // Y座標 [m]  
    double theta;  // 角度（ラジアン、右方0度、反時計回り正）
};



class ImuOdometry {
public:
    ImuOdometry(Database& db);
    void update(double dt);
    std::pair<double, double> getVelocity() const;
    Pose2D getPose() const;
    void setPose(const Pose2D& pose);

private:
    Database& database_;
    Pose2D pose_; // ワールド座標系での絶対姿勢 (x, y, theta)
    std::pair<double, double> velocity_; // ワールド座標系での速度 (vx, vy)
};

#endif // F3RC_2025_GROUPA_R2_IMUODOMETRY_HPP
