//
// Created by Salmo on 2026/11/5.
//

#include "motor.hpp"
#include "pid.hpp"
#include "can_user.h"
//1号电机
volatile PidGains gSpeedGainsM1 = {
    /*kp*/        0.03f,
    /*ki*/        0.0f,
    /*kd*/        0.003f,
    /*i_max*/     3.0f,
    /*out_max*/   5.0f,
    /*d_filter_k*/0.1f,
};
volatile PidGains gPosGainsM1 = {
    /*kp*/        2.0f,
    /*ki*/        0.008f,
    /*kd*/        1.0f,
    /*i_max*/     10.0f,
    /*out_max*/   3000.0f,
    /*d_filter_k*/0.1f,
};

//2号电机
volatile PidGains gSpeedGainsM2 = {
    /*kp*/        0.02f,
    /*ki*/        0.0f,
    /*kd*/        0.002f,
    /*i_max*/     3.0f,
    /*out_max*/   5.0f,
    /*d_filter_k*/0.1f,
};
volatile PidGains gPosGainsM2 = {
    /*kp*/        2.8f,
    /*ki*/        0.0f,
    /*kd*/        0.8f,
    /*i_max*/     10.0f,
    /*out_max*/   3000.0f,
    /*d_filter_k*/0.1f,
};

Motor::Motor(float ratio,
             const volatile PidGains& speed_gains,
             const volatile PidGains& pos_gains)
    : ratio_(ratio),
      speed_pid_(speed_gains),
      pos_pid_(pos_gains)
{
}

// 接收回调：解析 3508 (C620 电调) 反馈报文
void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    // 1. 转子机械角度 (0~8191 对应 0~360°)
    uint16_t raw_ecd = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    // 2. 转子转速 (RPM)
    int16_t raw_speed = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    // 3. 实际转矩电流 (-16384~16384 对应 -20A~20A)
    int16_t raw_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    // 4. 温度 (°C, uint8)
    uint8_t raw_temp = rx_data[6];

    ecd_angle_ = (float)raw_ecd * 360.0f / 8192.0f;
    speedRpm_  = (float)raw_speed;
    currentA_  = (float)raw_current * 20.0f / 16384.0f;
    tempC_     = (float)raw_temp;

    // 连续角度计算（累积角度）
    if (!received_) {
        received_ = true;
        last_ecd_ = (float)raw_ecd;
    } else {
        float delta = (float)raw_ecd - last_ecd_;
        if (delta > 4096.0f) {
            delta -= 8192.0f;
        } else if (delta < -4096.0f) {
            delta += 8192.0f;
        }
        angle_ += (delta * 360.0f / 8192.0f) / ratio_;
        last_ecd_ = (float)raw_ecd;
    }
}

// 设置发送电流
void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    // 3508: -20A~20A 对应 -16384~16384
    if (amperes >  20.0f) amperes =  20.0f;
    else if (amperes < -20.0f) amperes = -20.0f;
    int16_t raw_current = (int16_t)(amperes * 16384.0f / 20.0f);

    // 根据电机 ID (1~4) 计算偏移量
    uint8_t offset = (motor_id - 1) * 2;

    // 大端序写入
    tx_data_[offset]     = (raw_current >> 8) & 0xFF;
    tx_data_[offset + 1] = raw_current & 0xFF;
}

// 速度环控制：输入目标速度，输出电流
void Motor::pid_speed_control(float target_rpm, uint8_t motor_id) {
    if (!received_) return; // 没有反馈时不控制

    float output_current = speed_pid_.calc(target_rpm, speedRpm_);
    setTxCurrent(output_current, motor_id);
}

// 位置环控制：输入目标角度，内部串级速度环
void Motor::pid_position_control(float target_angle, uint8_t motor_id) {
    if (!received_) return;

    float target_speed = pos_pid_.calc(target_angle, angle_);
    pid_speed_control(target_speed, motor_id);
}

uint8_t* Motor::getTxData() {
    return tx_data_;
}
void Motor::resetPid() {
    speed_pid_.reset();
    pos_pid_.reset();
}
Motor motor1(1.0f, gSpeedGainsM1, gPosGainsM1);
Motor motor2(1.0f, gSpeedGainsM2, gPosGainsM2);