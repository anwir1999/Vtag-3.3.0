/*
 * MQTT.h
 *
 *  Created on: Aug 25, 2021
 *      Author: QuangDan
 */

#ifndef MYLIB_MQTT_H_
#define MYLIB_MQTT_H_

#include "Common.h"
#include "../MyLib/AT_Function.h"
#include "../MyLib/Sim7070G_General_Control.h"
#include "../MyLib/Common.h"
extern SIMCOM_ResponseEvent_t AT_RX_event;

void MQTT_Connect_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void MQTT_Disconnected_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void MQTT_Publish_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void MQTT_Subcribe_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);

void MQTT_UnsubTopic(char* topic);
void MQTT_PubDataToTopic(char* topic, char* data, int dataLen, int qos, int retain);
void MQTT_SubTopic(char* topic, int qos);
void MQTT_SubReceive_Callback(void *ResponseBuffer);

void CheckConfigure(char *tok);
void MQTT_SubReceive_Wait(uint16_t Timeout);
void MQTT_DevConf_Payload_Convert(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, uint8_t bat_level, char *cn, uint8_t n, int _ss, int WM, int _lc, int MA);
void MQTT_DonDevConf_Payload_Convert(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, uint8_t bat_level, char *cn, uint8_t n, int _ss, int WM, int _lc, int MA, char *loopReason);
void MQTT_BatteryAlert_Payload_Convert(char *type);
void MQTT_DevConf_FOTA_Convert(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, char *cn, uint8_t n);
void MQTT_DevConf_Payload_Convert_Startup(char *str, int16_t ss, uint8_t cc, char *type, int16_t rq, uint16_t p, uint8_t m, long ts, char *version, uint8_t bat_level, char *IMSI, uint8_t n);
#endif /* MYLIB_MQTT_H_ */
