//
// Created by HiAir on 2025/10/3.
//

#include "M3508_motor.h"

extern uint16_t angle;
extern uint16_t speed;
extern uint16_t current;
extern uint8_t temperature;

M3508_Motor::M3508_Motor(const float kratio):
    kratio_(kratio) {}

void M3508_Motor::CanRxMsgCallBack(const uint8_t rx_data_[8]) {
    angle = (int16_t)((rx_data_[0] << 8) | rx_data_[1]) * 360.f / 8191.f; //单位 °
    speed = (int16_t)((rx_data_[2] << 8) | rx_data_[3]); //单位 RPM
    current = (int16_t)((rx_data_[4] << 8) | rx_data_[5]) * 20.f / 16384.f; //单位 A
    temperature = (int8_t)rx_data_[6]; //单位 ℃

    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = (int16_t)((rx_data_[0] << 8) | rx_data_[1]) * 360.f / 8191.f; //单位 °
    rotate_speed_ = (int16_t)((rx_data_[2] << 8) | rx_data_[3]) * 360 / 60; //单位 dps
    current_ = (int16_t)((rx_data_[4] << 8) | rx_data_[5]) * 20.f / 16384.f; //单位 A
    temp_ = (int8_t)rx_data_[6]; //单位 ℃

    //假定两次采集之间 转动角度小于360
    if (rotate_speed_ >= 0.f) {
        if (ecd_angle_ > last_ecd_angle_) {
            delta_angle_ = (ecd_angle_ - last_ecd_angle_) / kratio_;
        } else {
            delta_angle_ = (ecd_angle_ + 360.f - last_ecd_angle_) / kratio_;
        }
    } else {
        if (ecd_angle_ < last_ecd_angle_) {
            delta_angle_ = (ecd_angle_ - last_ecd_angle_) / kratio_;
        } else {
            delta_angle_ = (ecd_angle_ - 360.f - last_ecd_angle_) / kratio_;
        }
    }

    angle_ += delta_angle_;
}

M3508_Motor motor = M3508_Motor(3591 / 187);