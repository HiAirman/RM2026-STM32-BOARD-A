//
// Created by HiAir on 2025/10/3.
//

#include "M3508_motor.h"

#include <math.h>

#include "can.h"

extern uint16_t angle;
extern uint16_t speed;
extern uint16_t current;
extern uint8_t temperature;

M3508_Motor::M3508_Motor(const float kratio, const int motor_rx_ID):
    kratio_(kratio),
    rx_ID_(motor_rx_ID) {}

void M3508_Motor::MotorInitialization() {
    //初始化

    //保护输出：0
    MotorOutput(0);
}


void M3508_Motor::CanRxMsgCallBack(const uint8_t rx_data_[8], const int rx_ID) {
    if (rx_ID != rx_ID_) {
        return;
    }
    //outdated
    angle = (int16_t)((rx_data_[0] << 8) | rx_data_[1]) * 360.f / 8191.f; //单位 °
    speed = (int16_t)((rx_data_[2] << 8) | rx_data_[3]); //单位 RPM
    current = (int16_t)((rx_data_[4] << 8) | rx_data_[5]) * 20.f / 16384.f; //单位 A
    temperature = (int8_t)rx_data_[6]; //单位 ℃
    //解析can收到的数据
    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = (int16_t)((rx_data_[0] << 8) | rx_data_[1]) * 360.f / 8191.f; //单位 °
    rotate_speed_ = (int16_t)((rx_data_[2] << 8) | rx_data_[3]) * 360 / 60; //单位 dps
    current_ = (int16_t)((rx_data_[4] << 8) | rx_data_[5]) * 20.f / 16384.f; //单位 A
    temp_ = (int8_t)rx_data_[6]; //单位 ℃

    //假定两次采集之间 转动角度小于360 计算angle
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

void M3508_Motor::TimerCallback() {
    //分模式计算
    switch (control_method_) {
        case TORQUE:
            break;
        case SPEED:
            break;
        case POSITION_SPEED:
            break;
        default:
            break;
    }
}

// from torque to current current = 2.7 * torque + 0.2
void M3508_Motor::MotorOutput() {
    if (flag_ == false) {
        //如果flag置0，直接输出0
        uint8_t motor_tx_data[8] = { 0 };
        uint32_t txMailBox;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor_tx_data, &txMailBox);
    }
    float output_current = (2.7f * abs(output_torque_) + 0.2f) * output_torque_ / abs(output_torque_); //保持方向
    int16_t output_value;
    if (abs(output_current) <= 20.0f) {
        output_value = (int16_t)(output_current * 16383.f / 20.f);
    } else {
        output_value = 0;
    }

    uint8_t motor_tx_data[8] = { 0 };

    motor_tx_data[2] = ((uint16_t)output_value) & 0xFF; //低字节
    motor_tx_data[3] = ((uint16_t)output_value >> 8) & 0xFF; //高字节
    //发送tx包
    uint32_t txMailBox;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor_tx_data, &txMailBox);
}

void M3508_Motor::MotorOutput(const float torque) {
    set_output_torque(torque);
    MotorOutput();
}
void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void M3508_Motor::SetIntensity(float intensity) {
    output_torque_ = intensity;
}

void M3508_Motor::SetFlag(bool flag) {
    if (output_torque_ < 4.5f) {
        flag_ = flag;
    } else {
        flag_ = false;
    }
}

void M3508_Motor::monitor_motor_temperature() {
    if (temp_ > 125) {
        flag_ = false;
    }
}

void M3508_Motor::monitor_motor_current() {
    if (current_ > 14.5) {
        flag_ = false;
    }
}


M3508_Motor motor1 = M3508_Motor(3591 / 187, 0x201);