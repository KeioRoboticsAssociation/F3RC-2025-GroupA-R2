#pragma once
#include <mbed.h>

// 参考: https://keiorogiken.wordpress.com/2017/12/08/ロータリーエンコーダの話/
// A相がinterrupt_in, B相がdigital_in
class Encoder
{
public:
    Encoder(PinName interrupt_in_pin, PinName digital_in_pin, int resolution = 2048, bool reverse_dir = false, bool is_dual = false);

    int getCount();
    float getRadians();
    float getDegrees();
    float getRotations();
    float countToRotations(int count);
    float countToRadians(int count);
    float countToDegrees(int count);
    int rotationsToCount(float rotation);
    void reset();

    void addCount(int count);

private:
    DigitalIn digital_in;
    InterruptIn interrupt_in;

    // 1回転あたりのカウント数
    int converted_resolution;
    int count;

    void interrupt(int sgn);
};
