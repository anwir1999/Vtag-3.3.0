/*
 * Common.h
 *
 *  Created on: Aug 11, 2021
 *      Author: QuangDan
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_clk.h"
#include "sdkconfig.h"
#include "string.h"
#include "pthread.h"
#include "cJSON.h"
#include "time.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_task_wdt.h"
#include "soc/rtc_wdt.h"

#define ECHO_TEST_TXD  17
#define ECHO_TEST_RXD  16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      2
#define ECHO_UART_BAUD_RATE     115200
#define uart_rx_task_STACK_SIZE    2048

#define ACCURACY_THR			0
#define AP_COUNT_THR			0

#define BUF_SIZE (4096)

#define NB_IoT					 	0
#define GSM							1
#define Both_NB_GSM					51
#define MQTT_SUB_TIMEOUT			100

char Network_Type_Str[10];

#define DAM_BUF_TX 512
#define DAM_BUF_RX 2049
#define MAX_RETRY	10

//RTC_DATA_ATTR char VTAG_Vesion[10]		    =		"S3.0.2x";
extern RTC_DATA_ATTR char VTAG_Vesion[10];
//RTC_DATA_ATTR char VTAG_Version_next[10] 	=		"S3.0.2y";
extern RTC_DATA_ATTR char VTAG_Version_next[10];

#define CALIB_FACTOR	1.048
//#define BU_Arr_Max_Num 	12
#define INNOWAY_LIVE	1
#define INNOWAY_TEST 	0
#define SERVER_TEST 	0
#define VTT_LIVE_TEST	1
typedef enum
{
	NORMAL = 0,
	CFUN,
	SCAN_NET,
	REG_NET,
	CHECK_ACT_NET,
	ACT_NET,
	MQTT_CON,
	MQTT_SUB,
	MQTT_PUB,
	MQTT_SUB_REC,
} BU_reason;
typedef enum
{
	NO_AC = 0, //  do nothing with pin activity
	Update_BP, // update buffer pin
	Send_BP, // send message buffer pin
} PIN_activity;
typedef struct
{
	char MAC_Serial[150];
}mac_serial_st;
typedef enum
{
	PAIR = 0,
	UNPAIR,
	STARTUP,
	LOCATION,
	GPS_FAIL,
	LOWBATTERY,
	FULLBATTERY,
	STARTMOTION,
	STOPMOTION,
	SOS,
	DCF_ACK,
	UNPAIR_GET,
	OFF_ACK,
	FOTA_START,
	FOTA_SUCCESS,
	FOTA_FAIL,
	SEND_BACKUP,
} VTAG_MessageType;
typedef enum
{
	NO_LOOPMAIN,		//0
	SEND_BK,			//1
	CHECK_PIN,			//2
	MAIN_ENL,			//3
	WIFI_DETECT_1,		//4
	WD1_DASP,			//5
	WD1_SLEEP,			//5
	WD1_BACKUP,			//7
	WAKEUP_EXT0,		//8
	WAKEUP_EXT1,		//9
	WAKEUP_TIMER,		//10
	WAKEUP_UNDEFINED,	//11
	PIN_FULL,			//12
	WIFI_DETECT_2,		//13
	WD2_DASP,			//14
	WD2_SLEEP,			//15
	WD2_BACKUP,			//16
	MODE_CC2,			//17
	MODE_CC3,			//18
	MODE_CC1,			//19
	SMS_RECEIVE,		//20
	SMS_RECEIVE_PAST, 	//21
	BACKUP_MESS			//22
}loopmain_reason_t;
typedef enum
{
	NO_LOOPBUTTON,		//0
	BT_RESTART,			//1
	BT_UNPAIR,			//2
	BT_FOTA,			//3
	BT_SOS,				//4
	BT_NOTHING,			//5
	BT_ONE,				//6
	BT_DEFAULT,			//7

}loopbutton_reason_t;
typedef enum
{
	NOR = 0,
	STR_PRE,
	STR_CUR,
	CHANGE_RSSI,
	WIFI_DISAPPEARE,
}wifi_detect_reason;
typedef enum
{
	B_UNPAIR = 2,
	B_SHUTDOWN = 3,
	B_FOTA = 4,
	B_RESTART = 5,
	B_UNPAIR_TEST = 6,
} Button_Task;

typedef enum
{
	MESS_NONE = 0,
	MESS_UNPAIR = 1,
	MESS_PAIR = 2,
	MESS_DOF = 4,
	MESS_DCF = 5,
	MESS_DOFA = 6,
} ACK_MESS;
typedef enum
{
	WAIT_ACK = 0,
	ACK_DONE = 1,
};
typedef struct _CFG
{
	int _lc		;
	int FLR;
	int F2G;
	int BT		;
	int MA		;
	int WM		;
	int _SS		;
	int Mode 	;
	int Period;
	char Type[5];
	int CC		;
	int Network ;
	int Accuracy;
	char Server_Timestamp[15];
} CFG;

typedef struct
{
	int16_t RSRP;
	int16_t RSRQ;
	int16_t RSSI;
}Network_Signal;

typedef struct
{
	uint64_t Device_Timestamp;
	uint8_t Bat_Level;
	uint16_t Bat_Voltage;
	uint16_t Pre_Bat_Voltage; // store bat value previous
}Device_Param;

typedef struct
{
	bool Flag_ScanNetwok;
	bool Flag_Cycle_Completed;
	bool Flag_ActiveNetwork;
	bool Flag_DeactiveNetwork;
	bool Flag_Network_Active;
	bool Flag_Network_Check_OK;
	bool Flag_Wait_Exit;
	bool Flag_GPS_Started;
	bool Flag_GPS_Stopped;
	bool Flag_Timer_GPS_Run;
	bool Flag_Device_Ready;
	bool Flag_WifiScan_Request;
	bool Flag_WifiScan_End;
	bool Flag_WifiCell_OK;
	bool Flag_Restart7070G_OK;
	bool Flag_CFUN_0_OK;
	bool Flag_CFUN_1_OK;
	bool Flag_Control_7070G_GPIO_OK;
	bool Flag_SelectNetwork_OK;
	bool Flag_NeedToProcess_GPS;
	bool Flag_MQTT_Stop_OK;
	bool Flag_Reboot7070_OK;
	bool Flag_NeedToReboot7070;
	// Need adding to reset parameters funtion
	bool Flag_MQTT_Connected;
	bool Flag_MQTT_Sub_OK;
	bool Flag_MQTT_Publish_OK;
	bool Flag_MQTT_SubMessage_Processing;
}Device_Flag;

typedef struct
{
	/* data */
	const char content[500];
} ATC_List_t;

typedef enum
{
	EVEN_OK = 0,
	EVEN_TIMEOUT,
	EVEN_ERROR,
} SIMCOM_ResponseEvent_t;
typedef void (*SIMCOM_SendATCallBack_t)(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);

//-----------------------------------------------------------------------------------------------------// MQTT
// Device ID
//#define Device_ID_TW 			"c6118176-b3a9-4c70-8a46-2685edb35d57"									// TW_1
//#define Device_ID_TW			"ee5db00a-b105-49cf-aeab-c4d0a34bc7e9"									// TW_5
#define Device_ID_TW			"MAN02ND00160"									// TW_KIT

#define MQTT_TX_Str_Buf_Lenght	1000

typedef enum
{
	MQTT_STATE_CONFIG_URL = 0,
	MQTT_STATE_CONFIG_USN,
	MQTT_STATE_CONFIG_PAS,
	MQTT_STATE_CONFIG_CID,
	MQTT_STATE_CONFIG_KTA,
	MQTT_STATE_CONFIG_CLE,
	MQTT_STATE_START_NETWORK,
	MQTT_STATE_CONN,
	MQTT_STATE_DISC,
	MQTT_STATE_STOP_NETWORK,
	MQTT_STATE_CHECK_NETWORK,

} MQTT_State_t;

typedef struct
{
	/* data */
	char dataRec[1024];
	char dataTrans[1024];
} MQTTdataType_t;

MQTTdataType_t MQTTdataType;

static ATC_List_t AT_MQTT_List[] =
{
#if SERVER_TEST
		{"AT+SMCONF=\"URL\",203.113.138.18,4445\r\n"}, 					        // URL, port TCP
		{"AT+SMCONF=\"USERNAME\",\"CHUNG97\"\r\n"}, 							// username
		{"AT+SMCONF=\"PASSWORD\",\"TESTPASS\"\r\n"}, 							// pass
		{"AT+SMCONF=\"CLIENTID\",\"TEST\"\r\n"},								// client id
		{"AT+SMCONF=\"KEEPTIME\",120\r\n"},										// kepptime alive
		{"AT+SMCONF=\"CLEANSS\",1\r\n"},										// cleanssession flag
#elif INNOWAY_TEST
		{"AT+SMCONF=\"URL\",116.101.122.190,1883\r\n"}, 					        // URL, port TCP
		{"AT+SMCONF=\"USERNAME\",\"vtag\"\r\n"},								// username
		{"AT+SMCONF=\"PASSWORD\",\"abMthkHU3UOZ7T5eICcGrVvjPbya17ER\"\r\n"},	// pass
		{"AT+SMCONF=\"CLIENTID\",\"8344\"\r\n"},								// client id
		{"AT+SMCONF=\"KEEPTIME\",120\r\n"},										// kepptime alive
		{"AT+SMCONF=\"CLEANSS\",1\r\n"},										// cleanssession flag
#elif INNOWAY_LIVE
		{"AT+SMCONF=\"URL\",171.244.133.251,1883\r\n"}, 					        // URL, port TCP
		{"AT+SMCONF=\"USERNAME\",\"vtag\"\r\n"},								// username
		{"AT+SMCONF=\"PASSWORD\",\"\"\r\n"},	// pass
		{"AT+SMCONF=\"CLIENTID\",\"8344\"\r\n"},								// client id
		{"AT+SMCONF=\"KEEPTIME\",120\r\n"},										// kepptime alive
		{"AT+SMCONF=\"CLEANSS\",1\r\n"},
#else
		{"AT+SMCONF=\"URL\",171.244.133.251,1883\r\n"},
		{"AT+SMCONF=\"USERNAME\",\"VTAG_admin\"\r\n"},							// username
		{"AT+SMCONF=\"PASSWORD\",\"\"\r\n"},									// pass
		{"AT+SMCONF=\"CLIENTID\",\"8344\"\r\n"},								// client id
		{"AT+SMCONF=\"KEEPTIME\",120\r\n"},										// kepptime alive
		{"AT+SMCONF=\"CLEANSS\",1\r\n"},										// cleanssession flag
#endif
		{"AT+CNACT=0,1\r\n"},
		{"AT+SMCONN\r\n"},
		{"AT+SMDISC\r\n"},
		{"AT+CNACT=0,0\r\n"},
		{"AT+CPSI?\r\n"},
};

char Mqtt_TX_Str[MQTT_TX_Str_Buf_Lenght];
char MQTT_ID_Topic[75];

//--------------------------------------------------------------------------------------------------------------// Define for cell
int MCC, MNC, LAC, cell_ID, RSSI;
//--------------------------------------------------------------------------------------------------------------// Define for simcom response event
SIMCOM_ResponseEvent_t AT_RX_event;
//--------------------------------------------------------------------------------------------------------------// Define for Flag
#endif /* COMMON_H_ */
