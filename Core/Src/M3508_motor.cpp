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

void Motor_Init() {
    motor1.MotorInitialization();
}


M3508_Motor::M3508_Motor(const float kratio, const int motor_rx_ID,
                        const float kp_ppid, const float ki_ppid,const float kd_ppid,
                        const float kp_spid, const float ki_spid, const float kd_spid):
    kratio_(kratio),
    rx_ID_(motor_rx_ID),
    target_angle_(0.0f), fdb_angle_(0.0f),
    target_speed_(0.0f), fdb_speed_(0.0f), feedforward_speed_(0.0f),
    feedforward_intensity_(0.0f), output_intensity_(0.0f),
    control_method_(TORQUE) {
    //initialize pid
    spid_ = PID(kp_spid, ki_spid, kd_spid, 10, 4.7, 1, 0.001); //输出Torque
    ppid_ = PID(kp_ppid, ki_ppid, kd_ppid, 0, 2400, 1, 0.001); //输出speed
}

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
    rotate_speed_ = (int16_t)((rx_data_[2] << 8) | rx_data_[3]) * 360.f / 60.f; //单位 dps
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
    angle_ += delta_angle_; //angle_ 是从0开始累加的

    //为pid传送数据
    fdb_angle_ = angle_;
    fdb_speed_ = rotate_speed_ / kratio_;
}

void M3508_Motor::TimerCallback() {
    //计算前馈值
    //feedforward_intensity_ = FeedforwardIntensityCalc(angle_);
    //分模式计算
    switch (control_method_) {
        case TORQUE:
            break;
        case SPEED:
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_)
                                + feedforward_intensity_;
            break;
        case POSITION_SPEED:
            target_speed_ = ppid_.calc(target_angle_, fdb_angle_)
                            + feedforward_speed_;
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_)
                            + feedforward_intensity_;
            break;
        default:
            break;
    }
    //监测motor状态
    MonitorMotorCurrent();
    MonitorMotorTemperature();
    //输出发包
    MotorOutput(output_intensity_);
}

// from torque to current current = 2.7 * torque + 0.2
void M3508_Motor::MotorOutput() {
    if (flag_ == 1) {
        //如果flag置1，直接输出0
        uint8_t motor_tx_data[8] = { 0 };
        uint32_t txMailBox;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor_tx_data, &txMailBox);
    } else {
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
}

void M3508_Motor::MotorOutput(const float torque) {
    set_output_torque(torque);
    MotorOutput();
}
void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void M3508_Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    output_torque_ = intensity;
}

//Intensity = 500g * G * 55.24*0.001m * sin(angle)
float M3508_Motor::FeedforwardIntensityCalc(float current_angle) {
    return 0.5 * 9.8 * 0.05524 * sin(current_angle);
}

void M3508_Motor::SetFlag(const uint8_t flag) {
    if (output_torque_ < 4.5f) {
        flag_ = flag;
    } else {
        flag_ = 1;
    }
}

uint8_t M3508_Motor::get_flag() {
    return flag_;
}
float M3508_Motor::get_angle() {
    return angle_;
}
float M3508_Motor::get_rotate_speed() {
    return rotate_speed_;
}
float M3508_Motor::get_temperature() {
    return temp_;
}
void M3508_Motor::set_output_torque(const float torque) {}

void M3508_Motor::MonitorMotorTemperature() {
    if (temp_ > 125) { // 高于125℃
        flag_ = 1;
    }
}

void M3508_Motor::MonitorMotorCurrent() {
    if (current_ > 14.5) { //电流值高于14.5A
        flag_ = 1;
    }
}


M3508_Motor motor1 = M3508_Motor(3591 / 187, 0x201,
                                0.01, 0.0, 0.0,
                                0.01, 0.0, 0.0);