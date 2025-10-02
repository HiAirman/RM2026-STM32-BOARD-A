//
// Created by HiAir on 2025/10/2.
//

#include "timer_interrupt_handler.h"
#include <math.h>
#include "tim.h"
#include "gpio.h"


uint32_t cnt = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    cnt++;
    if (htim == &htim1){
        uint32_t arr_value = __HAL_TIM_GetCounter(&htim1) + 1;
        uint32_t brightness = arr_value * sinf(4 * HAL_GetTick() / 1000.f) - 1;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
    }

}
