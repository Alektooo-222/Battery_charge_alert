/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "swoTrace.h"

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "SIM800.h"
#include "user_callback.h"
#include "TIM_func.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define RX_BUFFER_SIZE 100

  extern UART_HandleTypeDef huart1;
  extern TIM_HandleTypeDef htim2;

  extern volatile char rx_buffer[RX_BUFFER_SIZE];
  extern uint8_t flag_tx_cplt;
  extern ADC_HandleTypeDef hadc1;
  extern uint8_t flag_adc_cplt;
  extern uint8_t flag_rx_cplt;
  extern uint8_t flag_n;
  extern uint8_t flag_tim2;

  extern int rx_index;
  extern uint8_t rx_byte;
  extern uint8_t newline_count;
  extern uint8_t recording;
  extern uint8_t ctrl_z;

#define START_ADC(hadc, massive, CONVERSIONS_ADC) \
                                                  do{          \
                                                    HAL_ADC_Start_DMA(hadc, massive, CONVERSIONS_ADC); \
                                                    while (flag_adc_cplt == 0); \
                                                    flag_adc_cplt = 0; \
                                                  }while (0);

#define SLEEP_1S() \
                do{   \
                  HAL_TIM_Base_Start_IT(&htim2);  \
                  HAL_SuspendTick();              \
                  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);  \
                  HAL_ResumeTick();              \
                  HAL_TIM_Base_Stop_IT(&htim2);  \
                }while(0)
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
