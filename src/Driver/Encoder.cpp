#include "Driver/Encoder.hpp"

Encoder::Encoder(PinName interrupt_in_pin, PinName digital_in_pin, int resolution, bool reverse_dir, bool is_dual) : digital_in(digital_in_pin), interrupt_in(interrupt_in_pin)
{
    count = 0;
    converted_resolution = resolution * (is_dual ? 2 : 1) * (reverse_dir ? -1 : 1);

    // 割り込みハンドラの設定
    interrupt_in.rise([this]()
                      { interrupt(1); });
    if (is_dual)
    {
        interrupt_in.fall([this]()
                          { interrupt(0); });
    }
}

// 割り込みハンドラ
void Encoder::interrupt(int sgn)
{
    if (digital_in.read() == sgn)
    {
        count++;
    }
    else
    {
        count--;
    }
}

// カウント数を取得
int Encoder::getCount()
{
    return count;
}

// 回転数を取得
float Encoder::getRotations()
{
    return (float)count / converted_resolution;
}

// 角度[rad]を取得
float Encoder::getRadians()
{
    return getRotations() * 2 * M_PI;
}

// 角度[deg]を取得
float Encoder::getDegrees()
{
    return getRotations() * 360;
}

// カウント数から回転数に変換
float Encoder::countToRotations(int count)
{
    return (float)count / converted_resolution;
}

// カウント数から角度[rad]に変換
float Encoder::countToRadians(int count)
{
    return countToRotations(count) * 2 * M_PI;
}

// カウント数から角度[deg]に変換
float Encoder::countToDegrees(int count)
{
    return countToRotations(count) * 360;
}

// 回転数からカウント数に変換
int Encoder::rotationsToCount(float rotations)
{
    return (int)(converted_resolution * rotations);
}

void Encoder::reset()
{
    count = 0;
}

// カウント数を加算 (シミュレーション用)
void Encoder::addCount(int count)
{
    this->count += count;
}
