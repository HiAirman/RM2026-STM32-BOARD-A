//
// Created by HiAir on 2025/10/3.
//

#include "M3508_motor.h"
#include "can.h"
#include "tim.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        //读包
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        //自定义函数解析rx_data
        motor1.CanRxMsgCallBack(rx_data, rx_header.StdId);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM6) {
        //Test1
        //空，看rx数值
        //Test2
        //不开前馈，纯pid
        // motor1.TimerCallback();
        //Test3
        //开前馈
        // motor1.TimerCallback();
    }
    //按键置反flag
    if (htim->Instance == TIM7) {
        //按键监测
        motor1.MonitorButton();
    }
}