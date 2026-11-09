//
// Created by Salmo on 2026/11/5.
//

#include "pid.hpp"

PID::PID(const volatile PidGains& gains)
    : gains_(gains)
{
    reset();
}

void PID::reset() {
    data_i = 0.0f;
    data_p = 0.0f;
    data_d = 0.0f;
    last_error = 0.0f;
}

float PID::calc(float ref, float fdb) {
    // 每次都现读 gains_, Keil Watch 修改立即生效
    const float kp         = gains_.kp;
    const float ki         = gains_.ki;
    const float kd         = gains_.kd;
    const float i_max      = gains_.i_max;
    const float out_max    = gains_.out_max;
    const float d_filter_k = gains_.d_filter_k;

    float error = ref - fdb;
    data_p = error;

    // 积分 (带限幅)
    data_i += ki * error;
    if (data_i >  i_max) data_i =  i_max;
    if (data_i < -i_max) data_i = -i_max;

    // 微分 (低通滤波)
    float raw_d_term = kd * (error - last_error);
    data_d = (1.0f - d_filter_k) * data_d + d_filter_k * raw_d_term;

    // 输出 + 限幅
    float output = (kp * data_p) + data_i + data_d;
    if (output >  out_max) output =  out_max;
    if (output < -out_max) output = -out_max;

    last_error = error;
    return output;
}
