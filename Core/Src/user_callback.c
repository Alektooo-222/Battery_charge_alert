#include "user_callback.h"

/* extern ADC_HandleTypeDef hadc1;
extern uint8_t flag;
extern uint8_t flag_rx_cplt;

uint8_t flag_n = 0;
int index = 0; */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        flag_adc_cplt = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        flag_tx_cplt = 1;
    }
}

/* void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (flag_n == 1)
        {
            HAL_UART_Transmit_IT(huart, rx_buff + rx_index, 1);
            ++rx_index;
        }

        if (huart->Instance->DR == '\n')
        {
            if (flag_n == 1)
            {
                flag_n = 0;
                flag_rx_cplt = 1;
            }

            flag_n = 1;
            HAL_UART_Transmit_IT(huart, rx_buff, 1);
        }
        else
        {
            HAL_UART_Transmit_IT(huart, rx_buff, 1);
        }
    }
} */

// Если эхо включено в SIM800L
#if TURN_ECO == 1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) 
    {
        if (rx_byte == '>')
        {
            flag_rx_cplt = 1;

            return;
        }

        if (huart->RxXferSize == 26)
        {
            flag_rx_cplt = 1;
        }

        if (rx_byte == '\n')
        {
            newline_count++;

            if (newline_count == 1)
            {
                // Первый \n — начинаем запись со следующего байта
                recording = 1;
                rx_index = 0; // очищаем буфер для новых данных
            }
            else if (newline_count == 2)
            {
                // Второй \n — завершение записи
                recording = 0;
                rx_buffer[rx_index] = '\n'; // завершаем строку
                rx_buffer[rx_index + 1] = '\0';
                flag_rx_cplt = 1;
                newline_count = 0;
                rx_index = 0;

                return;
            }
        }
        else if (recording)
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = rx_byte;
            }
            else
            {
                // переполнение
                recording = 0;
                rx_index = 0;
                newline_count = 0;
            }
        }

        // Повторный приём
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}
#else
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_index < RX_BUFFER_SIZE - 1)
        {
            rx_buffer[rx_index++] = rx_byte;
            if (rx_byte == '\n' )  
            {
                rx_buffer[++rx_index] = '\0'; // null-terminate
                flag_rx_cplt = 1;
                rx_index = 0;

                return;
            }
        }
        else
        {
            rx_index = 0; // переполнение — сброс
        }

        // Готов к следующему байту
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1);
    }
}
#endif //TURN_ECO

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {

    }
}
