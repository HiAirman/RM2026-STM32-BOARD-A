//
// Created by HiAir on 2025/10/3.
//

#include "M3508_motor.h"

extern uint16_t angle;
extern uint16_t speed;
extern uint16_t current;
extern uint8_t temperature;

M3508_Motor::M3508_Motor(const float kratio) : kratio_(kratio)
{}

void M3508_Motor::canRxMsgCallBack(const uint8_t rx_data_[8]){
    angle = (rx_data_[0] << 8) | rx_data_[1];
    speed = (rx_data_[2] << 8) | rx_data_[3];
    current = (rx_data_[4] << 8) | rx_data_[5];
    temperature = rx_data_[6];

}

M3508_Motor motor = M3508_Motor(3591/187);