#ifndef __TIM_FUNC_h
#define __TIM_FUNC_h

#include "main.h"

typedef enum{
    PERIOD_1S = 1,
    PERIOD_60S = 2
} Period_TIM;

void TIM_Base_Start_IT(TIM_HandleTypeDef *htim, uint8_t Period);

#endif