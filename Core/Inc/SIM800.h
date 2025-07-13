#ifndef __SIM800_H
#define __SIM800_H

#include "main.h"
//#include "string.h"

#define PHONE_NUMBER ""

#define SEND_COMMAND_SIM800L(command, size_message)                                                                               \
                                                do {                                                                \
                                                    HAL_UART_Transmit_IT(&huart1, command, size_message); \
                                                    while (flag_tx_cplt == 0);                                      \
                                                    flag_tx_cplt = 0;                                               \
                                                } while(0)
#define RECEIVE_RESPONSE_SIM800L()                                                                                  \
                                                do {                                                                \
                                                    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);                      \
                                                    while (flag_rx_cplt == 0);                                      \
                                                    flag_rx_cplt = 0;                                               \                                                
                                                } while(0)

void send_SMS(float V);
void call_start_SIM800();

#endif