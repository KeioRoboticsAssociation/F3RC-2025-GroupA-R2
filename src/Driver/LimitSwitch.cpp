#include "Driver/LimitSwitch.hpp"

LimitSwitch::LimitSwitch(PinName pin) : digitalIn(pin), lastState(false), currentState(false), lastEdgeState(false) {
    // タイマー開始
    debounceTimer.start();
}

void LimitSwitch::init() {
    // SS-10GL13専用プルアップ設定
    // COM → Mbedピン, NO → GND の配線構成
    // 非押下時: プルアップでHIGH (3.3V)
    // 押下時: NO-COM間導通でLOW (0V)
    digitalIn.mode(PullUp);
    
    // SS-10GL13の安定化待機
    // 機械的応答と電気的安定性を確保
    ThisThread::sleep_for(chrono::milliseconds(STABILIZE_TIME_MS));
    
    // 初期状態を安定的に取得
    bool initialRawState = !digitalIn.read();  // プルアップ構成なので論理反転
    currentState = initialRawState;
    lastState = initialRawState;
    lastEdgeState = initialRawState;
    
    // デバウンスタイマーリセット
    debounceTimer.reset();
}

bool LimitSwitch::isPressed() {
    // 生入力値取得 (プルアップなので論理反転)
    bool rawState = !digitalIn.read();
    
    // SS-10GL13専用デバウンス処理
    return debounce(rawState);
}

bool LimitSwitch::isPressedEdge() {
    bool currentPressed = isPressed();
    
    // 立ち上がりエッジ検出 (false → true)
    bool edge = currentPressed && !lastEdgeState;
    
    // エッジ状態更新
    lastEdgeState = currentPressed;
    
    return edge;
}

bool LimitSwitch::getRawInput() {
    // デバッグ用: 生の入力値 (論理反転済み)
    return !digitalIn.read();
}

bool LimitSwitch::debounce(bool rawState) {
    // SS-10GL13の物理特性に最適化されたデバウンスアルゴリズム
    
    // 状態変化検出
    if (rawState != lastState) {
        // 新しい状態変化を検出 → タイマーリセット
        debounceTimer.reset();
        lastState = rawState;
    }
    
    // SS-10GL13に最適化された30msデバウンス時間
    // この時間はSS-10GL13のSimulated Roller Leverの
    // 機械的応答時間と接点バウンス特性を考慮
    if (debounceTimer.read_ms() > DEBOUNCE_TIME_MS) {
        // 十分な時間が経過 → 状態を確定
        if (rawState != currentState) {
            currentState = rawState;
        }
    }
    
    return currentState;
}