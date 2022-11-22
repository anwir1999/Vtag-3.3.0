/*
 * BaterryRead_7070.h
 *
 *  Created on: Aug 29, 2021
 *      Author: QuangDan
 */

#ifndef MYLIB_SIM7070G_BATTERY_H_
#define MYLIB_SIM7070G_BATTERY_H_


#include "Common.h"
#include "../MyLib/AT_Function.h"
#include "../MyLib/Sim7070G_General_Control.h"

void Bat_GetPercent(void);
void Bat_GetVoltage(void);
void Battery_Decode(char *buffer, uint8_t *level, uint16_t *voltage);
void Battery_Calibrate(uint8_t *level);
void MQTT_LowBatteryAlert_Payload_Convert(void);
void Bat_Process(void);
void Bat_Process_(void);
#endif /* MYLIB_SIM7070G_BATTERY_H_ */
