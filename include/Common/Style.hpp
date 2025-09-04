#pragma once

// 2D姿勢（位置と向き）を表す構造体
struct Pose2D {
    double x;      // X座標 [m]
    double y;      // Y座標 [m]  
    double theta;  // 角度（ラジアン、右方0度、反時計回り正）
};