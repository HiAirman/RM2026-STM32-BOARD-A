//
// Created by HiAir on 2025/10/3.
// 文件包括M3508_motor类
// 类成员
//  公共
//      初始化（安全帧）
//      收到CAN信息的同步解析回调函数
//      控制电机转矩函数（电流与输出转矩是一次函数关系）
//      （暂时）保护flag，按下按钮改变
//      getter
//      setter
//  私有
// 成员函数：监测电机温度
//         监测转速、电流
// 成员变量：const StdId
//         电机参数
//         输出参数
//         flag
//

#ifndef RM_A_BOARD_TEST_DEV_M3508_MOTOR_H
#define RM_A_BOARD_TEST_DEV_M3508_MOTOR_H

#include "main.h"

class M3508_Motor {
public:
    explicit M3508_Motor(float kratio);
    void MotorInitialization();
    void CanRxMsgCallBack(const uint8_t rx_data_[8]);
    void MotorOutput(const float torque);
    void SetFlag(bool flag);
    //getters
    void angle();
    void rotate_speed();
    void temperature();
    //setters
    void set_output_torque();

private :
    const float kratio_ = 0.f; //电机减速比

    float angle_ = 0.f; // deg 输出端累计转动角度
    float delta_angle_ = 0.f; // deg 输出端新转动角度
    float ecd_angle_ = 0.f; // deg 当前电机编码器角度
    float last_ecd_angle_ = 0.f; // deg 上次电机编码器角度
    float delta_ecd_angle_ = 0.f; // deg 编码器端新转动角度
    float rotate_speed_ = 0.f; // dps 反馈转子转速
    float current_ = 0.f; // A 反馈转矩电流
    float temp_ = 0.f; // ℃ 反馈电机温度

    float output_torque = 0.f; // N·m 设置电机输出扭矩
};

extern M3508_Motor motor;

#endif //RM_A_BOARD_TEST_DEV_M3508_MOTOR_H