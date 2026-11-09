//
// Created by Salmo on 2026/11/5.
//

#ifndef MINIROBOTARM_PID_HPP
#define MINIROBOTARM_PID_HPP

// ---- PID 参数组 ----
// 定义成 volatile 全局, 配合 Keil Watch (勾 Periodic Window Update) 可以在线调参.
struct PidGains {
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float d_filter_k;
};

class PID {
public:
    // 持有 PidGains 的引用, calc() 每次都现读, 支持在线调参
    explicit PID(const volatile PidGains& gains);

    void reset();
    float calc(float ref, float fdb);

private:
    const volatile PidGains& gains_;

    // 状态
    float data_i;      // 积分累积值
    float data_p;      // 当前误差 (比例项)
    float data_d;      // 当前滤波后的微分项
    float last_error;  // 上一次的误差 (用于计算微分)
};
#endif //MINIROBOTARM_PID_HPP
