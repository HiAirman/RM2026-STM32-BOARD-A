//
// Created by HiAir on 2025/10/2.
//

#include <math.h>
#include "tim.h"
#include "usart.h"


uint32_t cnt = 0;
uint32_t brt = 0;
uint32_t timems = 0;

extern uint8_t rx_msg[4];

//处理timer2中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    cnt++;

    if (htim == &htim2){
        uint32_t arr_value = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
        uint32_t brightness = arr_value * sinf(4 * HAL_GetTick() / 1000.f) - 1;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
        brt = brightness;
        timems = HAL_GetTick();
    }

}
//处理UART RX接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7)
    {
        //收到R开灯，收到M灭灯
        if (rx_msg[0] == 'R')
        {
            HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
        }
        else if (rx_msg[0] == 'M')
        {
            HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
        }
        HAL_UART_Receive_IT(&huart7, rx_msg, 1);//重新接收下一条message
    }

}