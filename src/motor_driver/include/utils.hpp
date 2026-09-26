#pragma once

#include <chrono>
#include <thread>
#include <cstdint>

// float 转 uint - 将浮点数映射到指定位数的无符号整数
inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1u << bits) - 1)) / span);
}

// uint 转 float - 将无符号整数映射到指定范围的浮点数
inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1u << bits) - 1)) + offset;
}

// 延时函数
inline void delay_ms(int time)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

inline void delay_us(int time)
{
    std::this_thread::sleep_for(std::chrono::microseconds(time));
}

// 限幅函数
inline float clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    else if (value > max_value)
        return max_value;
    else
        return value;
}
