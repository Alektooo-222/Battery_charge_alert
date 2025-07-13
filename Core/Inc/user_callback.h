#ifndef USER_CALLBACK_
#define USER_CALLBACK_

#include "main.h"

#define TURN_ECO 1

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif //USER_CALLBACK_