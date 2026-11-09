//
// Created by Salmo on 2026/11/5.
//

#ifndef MINIROBOTARM_MOTOR_HPP
#define MINIROBOTARM_MOTOR_HPP
#include <cstdint>
#include "pid.hpp"
enum class MotorMode {
    STOP = 0,
    SPEED_CTRL = 1,
    POSITION_CTRL = 2,
};

// 每个电机一套: 速度环 + 位置环. 定义在 motor.cpp, 可在 Keil Watch 在线改.
extern volatile PidGains gSpeedGainsM1;
extern volatile PidGains gPosGainsM1;
extern volatile PidGains gSpeedGainsM2;
extern volatile PidGains gPosGainsM2;


class Motor {
public:
    Motor(float ratio,
          const volatile PidGains& speed_gains,
          const volatile PidGains& pos_gains);

    // 入口：CAN 接收中断调用
    void canRxMsgCallback(const uint8_t rx_data[8]);

    // 出口：控制逻辑读取
    float angle() const;           // 输出轴角度 (度)
    float speedRpm() const;        // 转子转速 (RPM)
    float currentAmps() const;     // 转矩电流 (A)
    float temperatureC() const;    // 温度 (℃)
    bool  hasFeedback() const;     // 收到过帧？

    // 命令：控制逻辑调用，内部限幅
    void setTxCurrent(float amperes, uint8_t motor_id);
    void pid_speed_control(float target_rpm, uint8_t motor_id);
    void pid_position_control(float target_angle, uint8_t motor_id);

    // 命令出口：TIM6 中断取用（HAL 形参非 const）
    uint8_t* getTxData();
    void resetPid();

private:
    // ---- 配置 ----
    const float ratio_;          // 减速比

    // ---- PID 控制器 ----
    PID speed_pid_;
    PID pos_pid_;

    // ---- 这一帧解出来的 ----
    float ecd_angle_ = 0;        // 机械角度 (0~8191)
    float speedRpm_ = 0;         // 转速 (RPM)
    float currentA_ = 0;         // 转矩电流 (A)
    float tempC_ = 0;            // 温度 (℃)

    // ---- 连续角度 ----
    float angle_ = 0;            // 输出轴累计角度
    float last_ecd_ = 0;         // 上一帧角度，用来求差
    bool  received_ = false;     // 第一帧特殊处理

    // ---- 待发送 ----
    uint8_t tx_data_[8] = {};

    static constexpr uint16_t kEncoderRange = 8192;
};
#endif //MINIROBOTARM_MOTOR_HPP
