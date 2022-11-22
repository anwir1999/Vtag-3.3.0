/*
 * AT_Function.h
 *
 *  Created on: Aug 11, 2021
 *      Author: QuangDan
 */

#ifndef AT_FUNCTION_H_
#define AT_FUNCTION_H_

#include "../MyLib/Common.h"

#define TIMER_ATC_PERIOD 100

typedef struct{
	char CMD[DAM_BUF_TX];
	uint32_t lenCMD;
	char ExpectResponseFromATC[20];
	uint32_t TimeoutATC;
	uint32_t CurrentTimeoutATC;
	uint8_t RetryCountATC;
	SIMCOM_SendATCallBack_t SendATCallBack;
}ATCommand_t;

ATCommand_t SIMCOM_ATCommand;

void SendATCommand(void);
void RetrySendATC();
void ATC_SendATCommand(const char *Command, char *ExpectResponse, uint32_t Timeout, uint8_t RetryCount, SIMCOM_SendATCallBack_t CallBackFunction);
void ATC_SendATCommandOptimize(const char *Command, char *ExpectResponse, uint32_t Timeout, uint8_t RetryCount, SIMCOM_SendATCallBack_t CallBackFunction);
void WaitandExitLoop(bool *Flag);
void ATResponse_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void SMSResponseCallBack(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
#endif /* AT_FUNCTION_H_ */
