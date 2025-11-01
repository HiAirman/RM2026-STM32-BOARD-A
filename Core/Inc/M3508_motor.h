//
// Created by HiAir on 2025/10/3.
// 文件包括M3508_motor类
// 类成员
//  公共
//      初始化（安全帧）
//      收到CAN信息的同步解析回调函数
//      控制电机转矩函数（电流与输出转矩是一次函数关系）
//
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
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

void Motor_Init(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class M3508_Motor {
public:
    explicit M3508_Motor(
        float kratio,
        const int motor_rx_ID,
        const float kp_ppid,
        const float ki_ppid,
        const float kd_ppid,
        const float kp_spid,
        const float ki_spid,
        const float kd_spi
    );
    //初始化应紧跟CAN初始化，在tim初始化和Rx中断初始化之前
    void MotorInitialization();
    //CAN RX 回调函数中调用
    void CanRxMsgCallBack(const uint8_t rx_data_[8], const int rx_ID);
    //TIM触发回调函数：计算输出值并调用output
    void TimerCallback();
    //控制电机输出对应扭矩
    void MotorOutput();
    void MotorOutput(const float torque);
    //前馈pid控制: 设置目标
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    //前馈力矩计算
    float FeedforwardIntensityCalc(float current_angle);
    //将flag设为对应值
    void SetFlag(const uint8_t flag);
    //监测按钮置反flag
    void MonitorButton();
    //getters
    uint8_t get_flag();
    float get_angle();
    float get_rotate_speed();
    float get_temperature();
    //setters
    void set_output_torque(const float torque);

private:
    const float kratio_ = 0.f; //电机减速比
    const int rx_ID_; //Can接收时对应ID
    //读
    float angle_ = 0.f; // deg 输出端累计转动角度
    float delta_angle_ = 0.f; // deg 输出端新转动角度
    float ecd_angle_ = 0.f; // deg 当前电机编码器角度
    float last_ecd_angle_ = 0.f; // deg 上次电机编码器角度
    float delta_ecd_angle_ = 0.f; // deg 编码器端新转动角度
    float rotate_speed_ = 0.f; // dps 反馈转子转速
    float current_ = 0.f; // A 反馈转矩电流
    float temp_ = 0.f; // ℃ 反馈电机温度
    //写
    float output_torque_ = 0.f; // N·m 设置电机输出扭矩
    //flag
    uint8_t flag_ = 1;
    uint8_t is_button_pressed_ = 0, is_flag_reversed_ = 0;
    uint16_t sum_pressed_ = 0, sum_unpressed_ = 0;

    //can对应结构体
    CAN_TxHeaderTypeDef tx_header = { .StdId = 0x200, //标准ID
                                      .ExtId = 0, //扩展ID
                                      .IDE = CAN_ID_STD, //标准ID
                                      .RTR = CAN_RTR_DATA, //数据帧
                                      .DLC = 8, //数据长度
                                      .TransmitGlobalTime = DISABLE };

    //PID器
    PID spid_, ppid_; //speed pid and position pid
    float target_angle_, fdb_angle_; //目标角度和反馈角度
    float target_speed_, fdb_speed_, feedforward_speed_; //目标速度、电机反馈速度、前馈速度
    float feedforward_intensity_, output_intensity_; //前馈力矩强度、输出力矩强度
    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;
    //私有方法
    void MonitorMotorTemperature();
    void MonitorMotorCurrent();
};

extern M3508_Motor motor1;

#endif

#endif //RM_A_BOARD_TEST_DEV_M3508_MOTOR_H