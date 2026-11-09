//
// Created by Salmo on 2026/11/5.
//

#include <cmath>
#include "motor.hpp"
#include "stm32f4xx_hal.h"

// 来自 callback.cpp 的 C ABI 封装
extern "C" void motor1_set_position(float angle_deg);
extern "C" void motor2_set_position(float angle_deg);

// 连杆长度 (mm)
static constexpr float L1 = 100.0f;
static constexpr float L2 = 75.0f;

// 肘部构型: true = 肘朝上 (theta2 > 0), false = 肘朝下 (theta2 < 0)
static constexpr bool kElbowUp = true;

// 输入目标位姿 (x, y) [mm], 解算两个关节角并下发.
// 返回 1 表示可达并已下发目标; 0 表示不可达, 电机目标不变.
extern "C" int arm_set_xy(float x, float y) {
    const float r2 = x * x + y * y;
    const float r_max = L1 + L2;
    const float r_min = std::fabs(L1 - L2);
    if (r2 > r_max * r_max || r2 < r_min * r_min) {
        return 0;
    }

    float cos_t2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cos_t2 >  1.0f) cos_t2 =  1.0f;
    if (cos_t2 < -1.0f) cos_t2 = -1.0f;

    const float sin_t2 = (kElbowUp ? 1.0f : -1.0f) * std::sqrt(1.0f - cos_t2 * cos_t2);
    const float theta2 = std::atan2(sin_t2, cos_t2);
    const float theta1 = std::atan2(y, x) - std::atan2(L2 * sin_t2, L1 + L2 * cos_t2);

    const float kRad2Deg = 180.0f / 3.14159265f;

    // theta1 的 atan2 在 ±180° 会跳变(画到 -x 轴那一侧时), 解绕保证目标连续, 否则电机会整圈甩
    float theta1_deg = theta1 * kRad2Deg;
    static float last_theta1_deg = 0.0f;
    static bool  has_last_theta1 = false;
    if (has_last_theta1) {
        while (theta1_deg - last_theta1_deg >  180.0f) theta1_deg -= 360.0f;
        while (theta1_deg - last_theta1_deg < -180.0f) theta1_deg += 360.0f;
    }
    last_theta1_deg = theta1_deg;
    has_last_theta1 = true;

    motor1_set_position(theta1_deg);
    motor2_set_position(theta2 * kRad2Deg);
    return 1;
}

// 画 200*200正方形
// update_period_ms: 位置更新周期(ms), 频率 = 1000/此值.
extern "C" void arm_draw_square(uint32_t update_period_ms) {
    constexpr float kHalfSide     = 100.0f;  //边长的一半
    constexpr int   kStepsPerSide = 100;      // 每条边插值点数

    const float cx[4] = {  kHalfSide, -kHalfSide, -kHalfSide,  kHalfSide };
    const float cy[4] = {  kHalfSide,  kHalfSide, -kHalfSide, -kHalfSide };

    for (int side = 0; side < 4; ++side) {
        int next = (side + 1) % 4;
        for (int i = 0; i <= kStepsPerSide; ++i) {
            float t = (float)i / (float)kStepsPerSide;
            float x = cx[side] + (cx[next] - cx[side]) * t;
            float y = cy[side] + (cy[next] - cy[side]) * t;
            arm_set_xy(x, y);
            HAL_Delay(update_period_ms);
        }
    }
}
extern "C" float target_angle=0.0f;
