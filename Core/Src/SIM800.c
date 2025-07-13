#include "SIM800.h"

uint8_t tx_buff[] = "AT+CPIN?\r\n";
uint8_t ATOK[] = "AT\r\n";
uint8_t Turn_text[] = "AT+CMGF=1\r\n";
uint8_t Set_number[] = "AT+CMGS=\"+79604490040\"\r\n";
uint8_t SMS[] = "Where are you?";
uint8_t Call[] = "ATD+79604490040;\r\n";
uint8_t Call_end[] = "ATH\r\n";
uint8_t Echo_off[] = "ATE0\r\n";

void send_SMS(float V)
{
    char SMS[50];

    sprintf(SMS, "%s %d.%02d", "Voltage: ", (int32_t)V, (int32_t)((V - (int32_t)V) * 100));
    SEND_COMMAND_SIM800L(ATOK, strlen(ATOK));
    RECEIVE_RESPONSE_SIM800L();

    //make sms
    SEND_COMMAND_SIM800L(Turn_text, strlen(Turn_text));
    RECEIVE_RESPONSE_SIM800L();

    SEND_COMMAND_SIM800L(Set_number, strlen(Set_number));
    RECEIVE_RESPONSE_SIM800L();

    SEND_COMMAND_SIM800L(SMS, strlen(SMS));
    SEND_COMMAND_SIM800L(&ctrl_z, 1);
    RECEIVE_RESPONSE_SIM800L();

}

void call_start_SIM800()
{
    SEND_COMMAND_SIM800L(ATOK, strlen(ATOK));
    RECEIVE_RESPONSE_SIM800L();

    SEND_COMMAND_SIM800L(Call, strlen(Call));
    RECEIVE_RESPONSE_SIM800L();
    HAL_Delay(30000);
    call_end_SIM800();
}

void call_end_SIM800()
{
    SEND_COMMAND_SIM800L(ATOK, strlen(ATOK));
    RECEIVE_RESPONSE_SIM800L();

    SEND_COMMAND_SIM800L(Call_end, strlen(Call_end));
    RECEIVE_RESPONSE_SIM800L();
}