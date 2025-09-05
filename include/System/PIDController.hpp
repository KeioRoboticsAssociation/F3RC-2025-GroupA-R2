#pragma once

struct PIDGain
{
    float kp;      // Pゲイン
    float ki;      // Iゲイン
    float kd;      // Dゲイン
    int frequency; // 制御頻度
};

template <typename T>
class PIDController
{
public:
    PIDController() : gain({0.0f, 0.0f, 0.0f, 0}) {};          // デフォルトコンストラクタ
    PIDController(PIDGain pid_gain) : gain(pid_gain) {};       // ゲインを与えて初期化
    PIDController(float kp, float ki, float kd, int frequency) // ゲインを与えて初期化
    {
        gain.kp = kp;
        gain.ki = ki;
        gain.kd = kd;
        gain.frequency = frequency;
    };

    // 偏差を与えると操作量を返す。
    T calculate(T error)
    {
        printf("frequency: %d\n", gain.frequency);
        // 周波数を考慮
        T output = error * gain.kp +
                   integral * (gain.ki / gain.frequency) +
                   (error - prevError) * gain.kd * gain.frequency;
        prevError = error; // 前回の偏差を更新
        integral += error; // 積分値を更新

        // 操作量を返す
        return output;
    };

    // 積分値をリセット
    void reset()
    {
        integral = T{}; // 積分値をリセット
    };

    // 周波数を変更
    void setFrequency(int frequency)
    {
        gain.frequency = frequency; // 周波数を変更
    };

    // 周波数を取得
    int getFrequency()
    {
        return gain.frequency; // 周波数を取得
    };

    // ゲインを変更
    void setGain(float kp, float ki, float kd)
    {
        gain.kp = kp;
        gain.ki = ki;
        gain.kd = kd;
    };

    // ゲインを変更
    void setGain(PIDGain pid_gain)
    {
        gain = pid_gain;
    };

private:
    T integral = T{};  // 積分値
    T prevError = T{}; // 前回の偏差
    PIDGain gain;      // ゲイン
};
