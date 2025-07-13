#include "TIM_func.h"

void TIM_Base_Start_IT(TIM_HandleTypeDef *htim, Period_TIM Period)
{
    if (Period == PERIOD_1S)
    {
        TIM2->ARR = 62499;
        TIM2->PSC = 575;
    }

    if (Period == PERIOD_60S)
    {
        TIM2->ARR = 65501;
        TIM2->PSC = 29311;
    }
    HAL_TIM_Base_Start_IT(htim);
}