#include "Provider/Feedback.hpp"
#include <cmath> // M_PI, cos, sin


 
Feedback::Feedback(Database& db, ImuOdometry& odometry_calculator) 
    : database(db), odometry_calculator_(odometry_calculator) {
    // ワールド座標系（番号0）を原点(0,0,0)で初期化.
    coordinate_systems.push_back({0.0, 0.0, 0.0});
    current_coordinate_system_num = 0;
}

Pose2D Feedback::getPosition(int frame_num) {
    if (frame_num < 0 || static_cast<size_t>(frame_num) >= coordinate_systems.size()) {
        // エラー処理: 不正な座標系番号の場合は、現在の座標系での姿勢を返す
        frame_num = current_coordinate_system_num;
    }

    // Velocityクラスからワールド座標系での絶対姿勢を取得
    const Pose2D absolute_pose = odometry_calculator_.getPose();

    // absolute_poseはワールド座標系におけるロボットの絶対姿勢
    // 目標座標系の原点を取得
    const Pose2D& frame_origin = coordinate_systems[frame_num];

    // ワールド座標系における相対位置を計算
    double rel_x_world = absolute_pose.x - frame_origin.x;
    double rel_y_world = absolute_pose.y - frame_origin.y;

    // 目標座標系の向き(frame_origin.theta)を打ち消すために、-frame_origin.thetaで回転させる
    const double rotation_angle = -frame_origin.theta;
    const double cos_rot = cos(rotation_angle);
    const double sin_rot = sin(rotation_angle);

    Pose2D relative_pose;
    relative_pose.x = rel_x_world * cos_rot - rel_y_world * sin_rot;
    relative_pose.y = rel_x_world * sin_rot + rel_y_world * cos_rot;
    //https://youta-blog.com/rotate_point/

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

// Pose2D Feedback::getPosition(const std::string& frame_name) {
//     // "WorldFrame" は座標系0とする
//     if (frame_name == "WorldFrame") {
//         return getPosition(0);
//     }
//     // それ以外の名前付き座標系は未実装のため、現在の座標系を返す
//     return getPosition(current_coordinate_system_num);
// }

int Feedback::getCoordinateSystemNum() const {
    return current_coordinate_system_num;
}

void Feedback::resetCoordinateSystem() {
    // 現在の絶対姿勢を新しい座標系の原点として追加
    coordinate_systems.push_back(odometry_calculator_.getPose());
    // 新しく作成した座標系に切り替え
    current_coordinate_system_num = coordinate_systems.size() - 1;
}

//角度、位置、速度の更新
void Feedback::update(double dt) {
    odometry_calculator_.update(dt);
}

void Feedback::setAbsolutePose(const Pose2D& pose) {
    odometry_calculator_.setPose(pose);
}
