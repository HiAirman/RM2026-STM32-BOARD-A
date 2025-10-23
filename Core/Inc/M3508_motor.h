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
//         CAN类对象
//

#ifndef RM_A_BOARD_TEST_DEV_M3508_MOTOR_H
#define RM_A_BOARD_TEST_DEV_M3508_MOTOR_H

#include "main.h"

class M3508_Motor {
public:
    explicit M3508_Motor(float kratio, const int motor_rx_ID);
    //初始化应紧跟CAN初始化，在tim初始化之前
    void MotorInitialization();
    void CanRxMsgCallBack(const uint8_t rx_data_[8], const int rx_ID);
    void MotorOutput(const float torque);
    void SetFlag(bool flag);
    //getters
    void get_angle();
    void get_rotate_speed();
    void get_temperature();
    //setters
    void set_output_torque(const float torque);

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

    //can对应结构体
    CAN_RxHeaderTypeDef rx_header;
    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x200, //标准ID
        .ExtId = 0, //扩展ID
        .IDE = CAN_ID_STD, //标准ID
        .RTR = CAN_RTR_DATA, //数据帧
        .DLC = 8, //数据长度
        .TransmitGlobalTime = DISABLE
    };
    CAN_FilterTypeDef filter_config = {
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterActivation = ENABLE
    };
    //can对应变量
    int rx_ID_ = 0x000;
};

extern M3508_Motor motor;

#endif //RM_A_BOARD_TEST_DEV_M3508_MOTOR_H