//
// Created by HiAir on 2025/10/2.
//

#include <math.h>
#include "tim.h"
#include "gpio.h"


uint32_t cnt = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    cnt++;

}
