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
    PIDController() : gain({1.0f, 0.0f, 0.0f, 1}) {}           // デフォルトコンストラクタ
    PIDController(PIDGain pid_gain) : gain(pid_gain) {}        // ゲインを与えて初期化
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
        int freq = (gain.frequency > 0) ? gain.frequency : 1;

        T output = error * gain.kp +
                   integral * (gain.ki / freq) +
                   (error - prevError) * gain.kd * freq;

        prevError = error;
        integral += error;

        return output;
    }

    void reset()
    {
        integral = T{};
        prevError = T{};
    }

    void setFrequency(int frequency)
    {
        gain.frequency = frequency;
    }

    int getFrequency()
    {
        return gain.frequency;
    }

    // ゲインを変更
    void setGain(float kp, float ki, float kd)
    {
        gain.kp = kp;
        gain.ki = ki;
        gain.kd = kd;
    }

    void setGain(PIDGain pid_gain)
    {
        gain = pid_gain;
    }

    // Pゲインだけ変更
    void setP(float p) { gain.kp = p; }

private:
    T integral = T{};
    T prevError = T{};
    PIDGain gain;
};