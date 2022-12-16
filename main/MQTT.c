/*
 * MQTT.c
 *
 *  Created on: Aug 25, 2021
 *      Author: QuangDan
 */

#include "../MyLib/MQTT.h"

char AT_Command_Client_ID_Buf[128];
extern const char *TAG;

extern bool Flag_MQTT_Stop_OK;
extern bool Flag_Cycle_Completed;
extern bool Flag_MQTT_Connected;
extern bool Flag_MQTT_Sub_OK;
extern bool Flag_MQTT_Publish_OK;
extern bool Flag_MQTT_SubMessage_Processing;
extern bool Flag_sms_receive;
extern uint8_t stepConn;
extern uint8_t stepDisconn;
extern uint8_t stepSendData;
extern uint8_t Network_Type;
//extern char DeviceID_TW_Str[50];
extern uint16_t RTOS_TICK_PERIOD_MS;

extern CFG VTAG_Configure;
extern Network_Signal VTAG_NetworkSignal;
extern Device_Param VTAG_DeviceParameter;
extern bool Flag_backup_data;
extern VTAG_MessageType VTAG_MessType_G;
extern void BackUp_UnsentMessage(VTAG_MessageType Mess_Type);
extern bool Flag_sending_backup;
extern RTC_DATA_ATTR char DeviceID_TW_Str[50];
extern RTC_DATA_ATTR uint8_t Reboot_reason;
extern RTC_DATA_ATTR BU_reason Backup_reason;
extern bool Flag_test_connectSimcom;
extern bool Flag_motion_detected;
extern int counter_test_connectSimcom;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_Connect_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		switch (stepConn)
		{
		case 0:
			ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_CONFIG_URL].content, "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 1:
			ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_CONFIG_USN].content, "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 2:
			ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_CONFIG_PAS].content, "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 3:
			sprintf(AT_Command_Client_ID_Buf, "AT+SMCONF=\"CLIENTID\",\"%s\"\r\n", DeviceID_TW_Str);
			ATC_SendATCommand(AT_Command_Client_ID_Buf, "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 4:
			ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_CONFIG_KTA].content, "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 5:
			ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_CONFIG_CLE].content, "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 6:
			ATC_SendATCommand("AT+SMSTATE?\r\n", "OK", 2000, 2, MQTT_Connect_Callback);
			break;
		case 7:
			//			if(Flag_test_connectSimcom)
			//			{
			//				Flag_test_connectSimcom = false;
			//				ATC_SendATCommand("AT+SMCONN1\r\n", "OK", 10000, 1, MQTT_Connect_Callback);
			//			}
			//			else
			//			{
			ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_CONN].content, "OK", 10000, 1, MQTT_Connect_Callback);
			//			}
			break;
		case 8:
			ESP_LOGW(TAG, "MQTT Connected ok\r\n");
			Flag_MQTT_Connected = true;
			break;
		default:
			break;
		}
		stepConn++;
	}
	else if(event == EVEN_TIMEOUT || event == EVEN_ERROR)
	{
		Flag_MQTT_Connected = true;
		stepConn = 0;
		ESP_LOGW(TAG, "MQTT connect timeout or error\r\n");
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_Disconnected_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		switch (stepDisconn)
		{
		case 0:
			ESP_LOGW(TAG, "MQTT disconnect ok\r\n");
			//			if(VTAG_Configure.MA == 0 && Flag_motion_detected == true)
			//			{
			//				ATC_SendATCommand("AT+CNACT=0,0\r\n", "DEACTIVE", 3000, 3, MQTT_Disconnected_Callback);
			//				break;
			//			}
		case 1:
			ESP_LOGW(TAG, "Stop network ok\r\n");
			Flag_MQTT_Stop_OK = true;
			break;
		default:
			break;
		}
		stepDisconn++;
	}
	else if(event == EVEN_TIMEOUT || event == EVEN_ERROR)
	{
		stepConn = 0;
		Flag_MQTT_Stop_OK = true;
		ESP_LOGW(TAG, "MQTT disconnect timeout or error\r\n");
		ESP_sleep(1);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_Publish_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		if(stepSendData == 0)
		{
			ESP_LOGW(TAG, "Pub data: %s", MQTTdataType.dataTrans);
			ATC_SendATCommand(MQTTdataType.dataTrans, "OK", 20000, 3, MQTT_Publish_Callback);
		}
		else if(stepSendData == 1)
		{
			ESP_LOGW(TAG, "MQTT Publish ok\r\n");
			stepConn = 0;
			Flag_MQTT_SubMessage_Processing = false;
			Flag_MQTT_Publish_OK = true;
		}
		stepSendData++;
	}
	else if(event == EVEN_TIMEOUT || event == EVEN_ERROR)
	{
		ESP_LOGW(TAG, "MQTT Publish timeout \r\n");
		Flag_Cycle_Completed = true;
		Flag_backup_data = true;
		Backup_reason = MQTT_PUB;
		BackUp_UnsentMessage(VTAG_MessType_G);
		//TurnOff7070G();
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_Subcribe_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		Flag_MQTT_Sub_OK = true;
		ESP_LOGW(TAG, "MQTT Subcribe ok \r\n");
	}
	else if(event == EVEN_TIMEOUT || event == EVEN_ERROR)
	{
		ESP_LOGW(TAG, "MQTT Subcribe timeout \r\n");
		Flag_MQTT_Sub_OK = true;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_Unsubcribe_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	// MQTT_SubTopic(CONTROL_TOPIC, 1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_SubTopic(char* topic, int qos)
{
	char buf[100];
	uint32_t len;
	len = sprintf(buf, "AT+SMSUB=\"%s\",%d\r\n", topic, qos);
	buf[len] = 0;
	ESP_LOGI(TAG, "Sub topic: %s\r\n", topic);
	ATC_SendATCommand(buf, "OK", 1000, 3, MQTT_Subcribe_Callback);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_UnsubTopic(char* topic)
{
	char buf[100];
	uint32_t len;
	len = sprintf(buf, "AT+SMUNSUB=\"%s\"\r\n", topic);
	buf[len] = 0;
	ATC_SendATCommand(buf, "OK", 1000, 3, MQTT_Unsubcribe_Callback);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MQTT_PubDataToTopic(char* topic, char* data, int dataLen, int qos, int retain)
{
	char buf[1024];
	uint32_t len;
	len = sprintf(buf, "AT+SMPUB=\"%s\",%d,%d,%d\r\n", topic, dataLen+2, qos, retain);
	buf[len] = 0;
	memset(MQTTdataType.dataTrans, 0, strlen(MQTTdataType.dataTrans));
	sprintf(MQTTdataType.dataTrans, "%s\r\n", data);
	MQTTdataType.dataTrans[dataLen+2] = 0;
	stepSendData = 0;
	ATC_SendATCommand(buf, ">", 1000, 3, MQTT_Publish_Callback);
	return;
}

void MQTT_SubReceive_Callback(void *ResponseBuffer)
{
	char Tok_Buffer[200];
	char *tok = strtok(ResponseBuffer, ",");

	if(tok != NULL)
	{
		tok = strtok(NULL, "\r\n");
		strcpy(Tok_Buffer, tok);
		ESP_LOGW(TAG, "%s\r\n", Tok_Buffer);
	}

	if(tok != NULL)
	{
		tok = strtok(Tok_Buffer, ",");
		ESP_LOGW(TAG,"%s\r\n", tok);
		CheckConfigure(tok);
	}

	while(1)
	{
		tok = strtok(NULL, ",");
		if(tok == NULL) break;
		ESP_LOGW(TAG,"%s\r\n", tok);
		CheckConfigure(tok);
	}
	Flag_MQTT_SubMessage_Processing = true;
}

uint16_t MQTT_Sub_Timeout_Counter = 0;
void MQTT_SubReceive_Wait(uint16_t Timeout)
{
	while(1)
	{
		MQTT_Sub_Timeout_Counter++;
		if(Flag_MQTT_SubMessage_Processing == true || MQTT_Sub_Timeout_Counter >= Timeout)
		{
			//printf("111\r\n");
			//Flag_MQTT_SubMessage_Processing = false;
			if(MQTT_Sub_Timeout_Counter >= Timeout)
			{
				Flag_backup_data = true;
				Backup_reason = MQTT_SUB_REC;
				BackUp_UnsentMessage(VTAG_MessType_G);
				if(Flag_sms_receive == true || VTAG_MessType_G == FOTA_START|| VTAG_MessType_G == FOTA_FAIL ||  VTAG_MessType_G == FOTA_SUCCESS)
				{
					return;
				}
				//TurnOff7070G();
			}
			MQTT_Sub_Timeout_Counter = 0;
			break;
		}
		vTaskDelay(100 / RTOS_TICK_PERIOD_MS);
	}
}

void CheckConfigure(char *tok)
{
	if(strstr(tok, "\"Type\""))
	{
		ESP_LOGW(TAG,"Type configure\r\n");
	}
	else if(strstr(tok, "\"M\""))
	{
		ESP_LOGW(TAG,"Mode configure\r\n");
	}
	else if(strstr(tok, "\"P\""))
	{
		ESP_LOGW(TAG,"Period configure\r\n");
	}
	else if(strstr(tok, "\"a\""))
	{
		ESP_LOGW(TAG,"Accuracy configure\r\n");
	}
	else if(strstr(tok, "\"CC\""))
	{
		ESP_LOGW(TAG,"Country/city configure\r\n");
	}
	else if(strstr(tok, "\"N\""))
	{
		ESP_LOGW(TAG,"Network configure\r\n");
	}
	else if(strstr(tok, "\"T\""))
	{
		ESP_LOGW(TAG,"Time stamp configure\r\n");
	}
}

void MQTT_BatteryAlert_Payload_Convert(char *type)
{
	char Network_Type_Str[10];
	if(Network_Type == NB_IoT)
	{
		sprintf(Network_Type_Str, "nb");
	}
	else if(Network_Type == GSM)
	{
		sprintf(Network_Type_Str, "2g");
	}

	MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, type, VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc , VTAG_Configure.MA);
}

void MQTT_DevConf_FOTA_Convert(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, char *cn, uint8_t n, int WM, int MA, int _lc)
{
	memset(str, 0, MQTT_TX_Str_Buf_Lenght);
	sprintf(str,"{\"ss\":%d,\"CC\":%d,\"Type\":\"%s\",\"r\":%d,\"MMC\":{\"P\":%d,\"M\":%d},\"T\":%ld,\"V\":\"%s\",\"Cn\":\"%s\",\"N\":%d,\"WM\":%d,\"MA\":%d,\"lc\":%d,\"RR\":%d%d}", ss, cc, type, rq, p, m, ts, version, cn, n, WM, MA, _lc, Reboot_reason, Backup_reason);
}

void MQTT_DevConf_Payload_Convert(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, uint8_t bat_level, char *cn, uint8_t n, int _ss, int WM, int _lc, int MA)
{
	memset(str, 0, MQTT_TX_Str_Buf_Lenght);
	sprintf(str,"{\"ss\":%d,\"CC\":%d,\"Type\":\"%s\",\"r\":%d,\"MMC\":{\"P\":%d,\"M\":%d},\"T\":%ld,\"V\":\"%s\",\"B\":%d,\"Cn\":\"%s\",\"N\":%d,\"_ss\":%d,\"WM\":%d, \"lc\":%d,\"MA\":%d,\"RR\":%d%d}", ss, cc, type, rq, p, m, ts, version, bat_level, cn, n, _ss, WM,_lc,MA, Reboot_reason, Backup_reason);
}
void MQTT_DonDevConf_Payload_Convert(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, uint8_t bat_level, char *cn, uint8_t n, int _ss, int WM, int _lc, int MA, char *loopReason)
{
	memset(str, 0, MQTT_TX_Str_Buf_Lenght);
	sprintf(str,"{\"ss\":%d,\"CC\":%d,\"Type\":\"%s\",\"r\":%d,\"MMC\":{\"P\":%d,\"M\":%d},\"T\":%ld,\"V\":\"%s\",\"B\":%d,\"Cn\":\"%s\",\"N\":%d,\"_ss\":%d,\"WM\":%d, \"lc\":%d,\"MA\":%d,\"RR\":%d%d,\"LR\":\"%s\"}", ss, cc, type, rq, p, m, ts, version, bat_level, cn, n, _ss, WM,_lc,MA, Reboot_reason, Backup_reason, loopReason);
}
void MQTT_DevConf_Payload_Convert_Startup(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, uint8_t bat_level, char *IMSI, uint8_t n)
{
	memset(str, 0, MQTT_TX_Str_Buf_Lenght);
	sprintf(str,"{\"ss\":%d,\"CC\":%d,\"Type\":\"%s\",\"r\":%d,\"MMC\":{\"P\":%d,\"M\":%d},\"T\":%ld,\"V\":\"%s\",\"B\":%d,\"Cn\":\"%s\",\"N\":%d,\"RR\":%d%d}", ss, cc, type, rq, p, m, ts, version, bat_level, IMSI, n, Reboot_reason, Backup_reason);
}
