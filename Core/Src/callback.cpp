//
// Created by HiAir on 2025/10/3.
//

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"
#include "tim.h"
#include "M3508_Motor.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];
extern M3508_Motor Motor;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1 )
    {
        //读包
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        //自定义函数解析rx_data
        if (rx_header.StdId == 0x202)
        {
            Motor.canRxMsgCallBack(rx_data);
        }

    }
}

uint32_t txMailBox;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {
        //发包
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &txMailBox);
    }
}