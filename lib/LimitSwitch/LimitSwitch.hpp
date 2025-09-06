#ifndef LIMITSWITCH_HPP
#define LIMITSWITCH_HPP

#include <mbed.h>
#include <chrono>

/**
 * @brief OMRON SS-10GL13 マイクロスイッチドライバクラス
 * 
 * R2ロボット用リミットスイッチドライバ
 * - OMRON SS-10GL13 (Simulated Roller Lever)専用最適化
 * - 動作力: 0.49N (50gf)
 * - SPDT接点 (SPST-NOとして使用)
 * - 250VAC 10.1A定格
 * - 機械的耐久性: 30,000,000回
 * 
 * @author R2開発チーム
 * @date 2025-08-11
 */
class LimitSwitch {
public:
    /**
     * @brief コンストラクタ
     * @param pin SS-10GL13のCOM端子を接続するMbedピン
     */
    LimitSwitch(PinName pin);
    
    /**
     * @brief 初期化処理
     * 
     * SS-10GL13の特性に最適化された設定:
     * - プルアップ抵抗有効 (NO接点→GND接続構成)
     * - デバウンス時間: 物理特性に合わせて30ms
     * - 初期状態の安定化処理
     */
    void init();
    
    /**
     * @brief スイッチ押下状態取得
     * 
     * SS-10GL13のSimulated Roller Leverが壁に接触している状態を検知
     * 
     * @return true: レバーが押されている(壁接触), false: 非接触
     */
    bool isPressed();
    
    /**
     * @brief エッジ検出機能
     * 
     * 押下された瞬間を検知 (立ち上がりエッジ)
     * 連続で押されている間は最初の1回のみtrueを返す
     * 
     * @return true: 押下された瞬間, false: その他
     */
    bool isPressedEdge();
    
    /**
     * @brief 生の入力値取得 (デバッグ用)
     * @return デバウンス処理前の生の入力値
     */
    bool getRawInput();

private:
    DigitalIn digitalIn;              // デジタル入力ピン
    bool lastState;                   // 前回のデバウンス後状態
    bool currentState;                // 現在のデバウンス後状態
    bool lastEdgeState;               // エッジ検出用前回状態
    Timer debounceTimer;              // デバウンス用タイマー
    
    // SS-10GL13に最適化された定数
    static const int DEBOUNCE_TIME_MS = 30;    // デバウンス時間 (30ms)
    static const int STABILIZE_TIME_MS = 100;  // 初期化時安定化時間
    
    /**
     * @brief SS-10GL13専用デバウンス処理
     * 
     * SS-10GL13の物理特性 (動作力0.49N, 機械的応答時間) に
     * 最適化されたデバウンスアルゴリズム
     * 
     * @param rawState 生の入力状態
     * @return デバウンス処理後の安定した状態
     */
    bool debounce(bool rawState);
};

#endif // LIMITSWITCH_HPP