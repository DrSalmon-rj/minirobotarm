//
// Created by Salmo on 2026/11/5.
//
#include "can_user.h"
#include "motor.hpp"
#include "can.h"
#include "tim.h"

// motor1 / motor2 定义在 motor.cpp 末尾
extern Motor motor1;
extern Motor motor2;

// ================= 运行时控制目标 =================
// 主循环(或调试器)写入, TIM6 ISR 读取
// extern "C" 让 main.c 能直接链上

extern "C" {
    volatile MotorMode motor1_mode = MotorMode::STOP;
    volatile MotorMode motor2_mode = MotorMode::STOP;
    volatile float motor1_target_rpm   = 0.0f;
    volatile float motor1_target_angle = 0.0f;
    volatile float motor2_target_rpm   = 0.0f;
    volatile float motor2_target_angle = 0.0f;
}

// 供 C 代码调用的小封装
extern "C" void motor1_set_speed(float rpm) {
    motor1_target_rpm = rpm;
    motor1_mode = MotorMode::SPEED_CTRL;
}
extern "C" void motor1_set_position(float angle_deg) {
    motor1_target_angle = angle_deg;
    motor1_mode = MotorMode::POSITION_CTRL;
}
extern "C" void motor2_set_speed(float rpm) {
    motor2_target_rpm = rpm;
    motor2_mode = MotorMode::SPEED_CTRL;
}
extern "C" void motor2_set_position(float angle_deg) {
    motor2_target_angle = angle_deg;
    motor2_mode = MotorMode::POSITION_CTRL;
}
extern "C" void motors_stop(void) {
    motor1_mode = MotorMode::STOP;
    motor2_mode = MotorMode::STOP;
    motor1.resetPid();
    motor2.resetPid();
}

// ================= CAN RX: 按 ID 分发 =================
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    switch (rx_header.StdId) {
        case 0x201:
            motor1.canRxMsgCallback(rx_data);
            break;
        case 0x202:
            motor2.canRxMsgCallback(rx_data);
            break;
        default:
            break;
    }
}

// ================= TIM6 1kHz: 跑 PID + 合帧发送 =================
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM6) return;

    // Motor1: 速度环 / 位置环
    if (motor1_mode == MotorMode::SPEED_CTRL) {
        motor1.pid_speed_control(motor1_target_rpm, 1);
    } else if (motor1_mode == MotorMode::POSITION_CTRL) {
        motor1.pid_position_control(motor1_target_angle, 1);
    } else {
        motor1.setTxCurrent(0.0f, 1);
    }

    // Motor2: 速度环 / 位置环
    if (motor2_mode == MotorMode::SPEED_CTRL) {
        motor2.pid_speed_control(motor2_target_rpm, 2);
    } else if (motor2_mode == MotorMode::POSITION_CTRL) {
        motor2.pid_position_control(motor2_target_angle, 2);
    } else {
        motor2.setTxCurrent(0.0f, 2);
    }
    uint8_t frame[8];
    const uint8_t* d1 = motor1.getTxData();
    const uint8_t* d2 = motor2.getTxData();
    for (int i = 0; i < 8; ++i) frame[i] = d1[i] | d2[i];

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, frame, &can_tx_mailbox);
    }
}
