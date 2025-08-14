#pragma once

#include "mbed.h"

class LimitSwitch {
public:
    // 定数（ms単位）
    static constexpr int STABILIZE_TIME_MS = 50;  // SS-10GL13安定化待機時間（例）
    static constexpr int DEBOUNCE_TIME_MS  = 30;  // デバウンス時間

    // コンストラクタ
    explicit LimitSwitch(PinName pin);

    // 初期化（プルアップ設定・安定化処理）
    void init();

    // 現在押されているか（デバウンス済み）
    bool isPressed();

    // 押下の立ち上がりエッジを検出
    bool isPressedEdge();

    // 生の入力値（論理反転済み）を取得
    bool getRawInput();

private:
    // デバウンス処理（内部専用）
    bool debounce(bool rawState);

    // メンバ変数
    DigitalIn digitalIn;
    Timer debounceTimer;
    bool lastState;
    bool currentState;
    bool lastEdgeState;
};
