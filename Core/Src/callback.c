//
// Created by HiAir on 2025/10/2.
//

#include <math.h>
#include "tim.h"


uint32_t cnt = 0;
uint32_t brt = 0;
uint32_t timems = 0;

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
