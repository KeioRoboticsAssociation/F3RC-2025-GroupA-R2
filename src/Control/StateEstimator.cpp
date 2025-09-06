#include "Control/StateEstimator.hpp"
#include <cmath> // M_PI, cos, sin


StateEstimator::StateEstimator(Database& db, WheelOdometry& wheel_odom, ImuOdometry& imu_odom)
    : db_(db), wheel_odom_(wheel_odom), imu_odom_(imu_odom),
      fused_pose_{0,0,0}, fused_velocity_{0,0}, fused_acceleration_{0,0},
      one_derivative_{0,0}, two_derivative_{0,0}
{
    // ワールド座標系（番号0）を原点(0,0,0)で初期化.
    coordinate_systems_.push_back({0,0,0});
    current_coordinate_system_num = 0;
}

//情報を回収する
void StateEstimator::update(double dt) {
    // 1. WheelOdometryの速度・加速度
    auto wheel_vel = wheel_odom_.getVelocity();
    auto wheel_acc = wheel_odom_.getAcceleration();

    // 1.5 駆動輪エンコーダーの速度・加速度

    // 2. IMUによる速度・位置更新
    imu_odom_.update(dt);
    auto imu_vel = imu_odom_.getVelocity();
    auto imu_pose = imu_odom_.getPose();
    double imu_ang_vel = db_.getAngularVelocity();

    // 3. 融合（簡易相補フィルタ）
    fused_velocity_.first  = 0.0 * imu_vel.first  + 1.0 * wheel_vel.vx;
    fused_velocity_.second = 0.0 * imu_vel.second + 1.0 * wheel_vel.vy;
    fused_acceleration_ = { wheel_acc.ax, wheel_acc.ay };
    fused_pose_ = imu_pose;

    // 4. 1回微分・2回微分
    one_derivative_ = fused_velocity_;
    two_derivative_ = fused_acceleration_;

    // 5. リセット判定（両方押下でリセット）
    if(db_.getFrontLimitSwitchStatus() && db_.getSideLimitSwitchStatus()) {
        resetCoordinateSystem();
    }
}

Pose2D StateEstimator::getAbsolutePosition() const {
    return fused_pose_; 
}

/**
 * @brief 指定された座標系(frame_num)での相対位置を返します。
 * @param frame_num 基準となる座標系の番号
 * @return Pose2D 指定された座標系から見たロボットの姿勢
 */
Pose2D StateEstimator::getRelativePosition(int frame_num) const {
    if (frame_num < 0 || static_cast<size_t>(frame_num) >= coordinate_systems_.size()) {
        // エラー処理: 不正な座標系番号の場合は、現在の座標系番号を使用
        frame_num = current_coordinate_system_num;
    }

    // ✅ 修正: imu_odom_から直接取得するのではなく、このクラスが保持する融合後の絶対姿勢を使う
    const Pose2D absolute_pose = getAbsolutePosition();

    // 目標座標系の原点を取得
    const Pose2D& frame_origin = coordinate_systems_[frame_num];

    // ワールド座標系における相対位置ベクトルを計算
    double rel_x_world = absolute_pose.x - frame_origin.x;
    double rel_y_world = absolute_pose.y - frame_origin.y;

    // 目標座標系の向きに合わせて、相対位置ベクトルを回転
    const double rotation_angle = -frame_origin.theta;
    const double cos_rot = cos(rotation_angle);
    const double sin_rot = sin(rotation_angle);

    Pose2D relative_pose;
    relative_pose.x = rel_x_world * cos_rot - rel_y_world * sin_rot;
    relative_pose.y = rel_x_world * sin_rot + rel_y_world * cos_rot;

    // 相対的な向きを計算
    relative_pose.theta = absolute_pose.theta - frame_origin.theta;

    // 角度を -PI から PI の範囲に正規化 
    relative_pose.theta = fmod(relative_pose.theta + M_PI, 2.0 * M_PI);
    if (relative_pose.theta < 0) {
        relative_pose.theta += 2.0 * M_PI;
    }
    relative_pose.theta -= M_PI;

    return relative_pose;
}

int StateEstimator::getCoordinateSystemNum() const {
    return current_coordinate_system_num;
}

void StateEstimator::resetCoordinateSystem() {
    // 現在の絶対姿勢を新しい座標系の原点として追加
    coordinate_systems_.push_back(fused_pose_);
    // 新しく作成した座標系に切り替え
    current_coordinate_system_num = coordinate_systems_.size() - 1;
}

std::pair<double,double> StateEstimator::getVelocity() const { return fused_velocity_; }
std::pair<double,double> StateEstimator::getAcceleration() const { return fused_acceleration_; }
double StateEstimator::getAngularVelocity() const { return db_.getAngularVelocity(); }
TOFStatus StateEstimator::getTOFStatus() const {
    return { db_.getFrontTofStatus(), db_.getSideTofStatus() };
}
LimitSwitchStatus StateEstimator::getLimitSwitchStatus() const {
    return { db_.getFrontLimitSwitchStatus(), db_.getSideLimitSwitchStatus() };
}
std::pair<double,double> StateEstimator::getOneDerivative() const { return one_derivative_; }
std::pair<double,double> StateEstimator::getTwoDerivative() const { return two_derivative_; }
