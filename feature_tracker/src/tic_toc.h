#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

// 计时器类，用于测量代码执行时间（单位：毫秒）
class TicToc
{
public:
    // 构造函数：创建对象时自动开始计时
    TicToc()
    {
        tic();  // 对象构造时立即开始计时
    }

    // 开始计时：记录当前时间点
    void tic()
    {
        // 获取当前系统时钟的时间点并保存到start
        start = std::chrono::system_clock::now();
    }

    // 结束计时并返回经过的时间（毫秒）
    double toc()
    {
        // 获取当前系统时钟的时间点并保存到end
        end = std::chrono::system_clock::now();

        // 计算时间间隔，elapsed_seconds的类型是std::chrono::duration<double>
        // 表示以秒为单位的浮点数时间间隔
        std::chrono::duration<double> elapsed_seconds = end - start;

        // 将秒转换为毫秒并返回
        // count()返回时间间隔的数值，乘以1000将秒转换为毫秒
        return elapsed_seconds.count() * 1000;
    }

private:
    // 私有成员变量：存储开始和结束的时间点
    // std::chrono::system_clock::time_point 表示系统时钟的时间点
    // system_clock 是系统范围的实时时钟，可以转换为日历时间
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
