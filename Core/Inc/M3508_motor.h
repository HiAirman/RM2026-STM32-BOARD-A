//
// Created by HiAir on 2025/10/3.
//

#ifndef RM_A_BOARD_TEST_DEV_M3508_MOTOR_H
#define RM_A_BOARD_TEST_DEV_M3508_MOTOR_H

#include "main.h"

class M3508_Motor {
public:
    explicit M3508_Motor(float kratio);
    void CanRxMsgCallBack(const uint8_t rx_data_[8]);

private :
    const float kratio_ = 0.f; //电机减速比

    float angle_ = 0.f; // deg 输出端累计转动角度
    float delta_angle_ = 0.f; // deg 输出端新转动角度
    float ecd_angle_ = 0.f; // deg 当前电机编码器角度
    float last_ecd_angle_ = 0.f; // deg 上次电机编码器角度
    float delta_ecd_angle_ = 0.f; // deg 编码器端新转动角度
    float rotate_speed_ = 0.f; // dps 反馈转子转速
    float current_ = 0.f; // A 反馈转矩电流
    float temp_ = 0.f; // ℃ 反馈电机角度
};

extern M3508_Motor motor;

#endif //RM_A_BOARD_TEST_DEV_M3508_MOTOR_H