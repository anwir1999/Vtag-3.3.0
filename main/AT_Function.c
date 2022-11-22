/*
 * AT_Function.c
 *
 *  Created on: Aug 11, 2021
 *      Author: QuangDan
 */
#include "../MyLib/AT_Function.h"

extern bool Flag_Wait_Exit;
extern bool Flag_Device_Ready;
extern const char *TAG;

void SendATCommand()
{
	uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) SIMCOM_ATCommand.CMD, strlen(SIMCOM_ATCommand.CMD));
	ESP_LOGI(TAG,"Send: %s", SIMCOM_ATCommand.CMD);
}

void ATC_SendATCommand(const char *Command, char *ExpectResponse, uint32_t Timeout, uint8_t RetryCount, SIMCOM_SendATCallBack_t CallBackFunction)
{
	strcpy(SIMCOM_ATCommand.CMD, Command);
	SIMCOM_ATCommand.lenCMD = strlen(SIMCOM_ATCommand.CMD);
	strcpy(SIMCOM_ATCommand.ExpectResponseFromATC, ExpectResponse);
	SIMCOM_ATCommand.RetryCountATC = RetryCount;
	SIMCOM_ATCommand.SendATCallBack = CallBackFunction;
	SIMCOM_ATCommand.TimeoutATC = Timeout;
	SIMCOM_ATCommand.CurrentTimeoutATC = 0;

	SendATCommand();
}
void RetrySendATC()
{
	SendATCommand();
}
//--------------------------------------------------------------------------------------------------------------// Send AT OPTIMIZE
void ATC_SendATCommandOptimize(const char *Command, char *ExpectResponse, uint32_t Timeout, uint8_t RetryCount, SIMCOM_SendATCallBack_t CallBackFunction)
{
	Flag_Wait_Exit = false;
	ATC_SendATCommand(Command, ExpectResponse, Timeout, RetryCount, CallBackFunction);
	WaitandExitLoop(&Flag_Wait_Exit);
}
//------------------------------------------------------------------------------------------------------------------------//
void WaitandExitLoop(bool *Flag)
{
	while(1)
	{
		if(*Flag == true)
		{
			*Flag = false;
			break;
		}
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}
void ATResponse_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		ESP_LOGW(TAG, "Device is ready to use\r\n");
		Flag_Wait_Exit = true;
		Flag_Device_Ready = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		ESP_LOGE(TAG, "Device is not ready \r\n");
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		ESP_LOGE(TAG, "AT Check Error \r\n");
		Flag_Wait_Exit = true;
	}
}

void SMSResponseCallBack(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		printf((char *)ResponseBuffer);
	}
}
