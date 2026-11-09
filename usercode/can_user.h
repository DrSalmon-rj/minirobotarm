//
// Created by Salmo on 2026/11/5.
//

#ifndef MINIROBOTARM_CAN_USER_H
#define MINIROBOTARM_CAN_USER_H

#include "stm32f4xx_hal.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;
#endif //MINIROBOTARM_CAN_USER_H
