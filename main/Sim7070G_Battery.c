/*
 * BaterryRead_7070.c
 *
 *  Created on: Aug 29, 2021
 *      Author: QuangDan
 */
#include "../MyLib/Sim7070G_Battery.h"
#include "../MyLib/MQTT.h"
#include "../MyLib/ESP32_GPIO.h"
#include <math.h>
extern void WaitandExitLoop(bool *Flag);

extern const char *TAG;
extern bool Flag_Wait_Exit;
extern bool Flag_LowBattery;
extern Device_Param VTAG_DeviceParameter;
extern RTC_DATA_ATTR uint8_t Bat_raw_pre;
extern RTC_DATA_ATTR bool DBL_15;
extern RTC_DATA_ATTR bool DBL_10;
extern RTC_DATA_ATTR bool DBL_5;
extern bool Flag_DBL_Task;
extern RTC_DATA_ATTR float Calib_Pin_factor;
extern RTC_DATA_ATTR bool Flag_charged;
float Bat_CalibFactor_a_b [10][2] =
{
		{0.909090909, 20.90909091},
		{1.428571429,-18.57142857},
		{1.111111111,3.333333333},
		{1.666666667, -30},
		{2, -48},
		{3.333333333, -113.3333333},
		{3.333333333, -113.3333333},
		{3.333333333, -113.3333333},
		{1.666666667, -46.66666667},
		{0.294117647, 0},
};
int Raw_bat [11] =
{
		87,
		76,
		69,
		60,
		54,
		49,
		46,
		43,
		40,
		34,
		0,
};
void Bat_Process_(void)
{
   if(VTAG_DeviceParameter.Bat_Voltage > 3970)
   {
	   VTAG_DeviceParameter.Bat_Level = 100;
   }
   else if(VTAG_DeviceParameter.Bat_Voltage < 3390)
   {
	   VTAG_DeviceParameter.Bat_Level = 1;
   }
   else
   {
	  float a1 = 1493,a2 = 147.8,a3 = 51.31,a4 = 15.7;
	  float b1 = 0.0008793, b2 = 0.00475, b3 = 0.009529, b4=0.01551;
	  float c1 = 5.913, c2 = -6.446, c3 = -9.181, c4 = 16.32, time_con;
	  time_con = a1 * sin(b1 * (VTAG_DeviceParameter.Bat_Voltage) + c1) + a2 * sin(b2 * (VTAG_DeviceParameter.Bat_Voltage) + c2) + a3 * sin(b3 * (VTAG_DeviceParameter.Bat_Voltage) + c3) + a4 * sin(b4 * (VTAG_DeviceParameter.Bat_Voltage) + c4);
	  time_con = (670.36 - time_con)/(670.36);
	  int bat_percent = time_con * 100;
	  if(bat_percent > 99)
	  {
	     bat_percent = 100;
	  }
	  else if(bat_percent < 2)
	  {
	     bat_percent = 1;
	  }
	   //   else if(1 < bat_percent && bat_percent < 100)
	   //   {
	   //	   if(bat_percent % 5 <= 2)
	   //	   {
	   //			  bat_percent = bat_percent - bat_percent%5;
	   //	   }
	   //	   else bat_percent = bat_percent + (5 - bat_percent%5);
	   //   }
	   VTAG_DeviceParameter.Bat_Level = bat_percent;
   }
   ESP_LOGI(TAG,"VTAG_DeviceParameter.Bat_Level processed: %d\r\n",  VTAG_DeviceParameter.Bat_Level);
   if( VTAG_DeviceParameter.Bat_Level > 99)	 VTAG_DeviceParameter.Bat_Level = 100;
   if( VTAG_DeviceParameter.Bat_Level < Bat_raw_pre)
   {
	   Bat_raw_pre =  VTAG_DeviceParameter.Bat_Level;
   }
   else
   {
		if(gpio_get_level(CHARGE) == 1 && Flag_charged == false)
		{
			 VTAG_DeviceParameter.Bat_Level = Bat_raw_pre;
		}
		else
		{
			Bat_raw_pre =  VTAG_DeviceParameter.Bat_Level;
		}
   }
   Bat_raw_pre = VTAG_DeviceParameter.Bat_Level;

   VTAG_DeviceParameter.Bat_Level =(uint16_t) ceil(Calib_Pin_factor * VTAG_DeviceParameter.Bat_Level);
   if(VTAG_DeviceParameter.Bat_Level > 100)
	{
		VTAG_DeviceParameter.Bat_Level = 100;
	}
	if(VTAG_DeviceParameter.Bat_Level <= 15 && DBL_15 == false)
	{
		Flag_DBL_Task = true;
		Flag_LowBattery = true;
		DBL_15 = true;
	}
	if(VTAG_DeviceParameter.Bat_Level <= 10 && DBL_10 == false)
	{
		Flag_DBL_Task = true;
		Flag_LowBattery = true;
		DBL_10 = true;
	}
	if(VTAG_DeviceParameter.Bat_Level <= 5 && DBL_5 == false)
	{
		Flag_DBL_Task = true;
		Flag_LowBattery = true;
		DBL_5 = true;
	}
	if(VTAG_DeviceParameter.Bat_Level > 15)
	{
		DBL_15 = DBL_10 = DBL_5 = false;
	}
	Flag_charged = false;
}

void Bat_Process(void)
{
	Battery_Calibrate(&VTAG_DeviceParameter.Bat_Level);
	VTAG_DeviceParameter.Bat_Level =(uint16_t) ceil(Calib_Pin_factor * VTAG_DeviceParameter.Bat_Level);
	if(VTAG_DeviceParameter.Bat_Level > 100)
	{
		VTAG_DeviceParameter.Bat_Level = 100;
	}
	if(VTAG_DeviceParameter.Bat_Level <= 15 && DBL_15 == false)
	{
		Flag_DBL_Task = true;
		Flag_LowBattery = true;
		DBL_15 = true;
	}
	if(VTAG_DeviceParameter.Bat_Level <= 10 && DBL_10 == false)
	{
		Flag_DBL_Task = true;
		Flag_LowBattery = true;
		DBL_10 = true;
	}
	if(VTAG_DeviceParameter.Bat_Level <= 5 && DBL_5 == false)
	{
		Flag_DBL_Task = true;
		Flag_LowBattery = true;
		DBL_5 = true;
	}
	if(VTAG_DeviceParameter.Bat_Level > 15)
	{
		DBL_15 = DBL_10 = DBL_5 = false;
	}
}

static void Batterry_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	Flag_Wait_Exit = true;
	if(event == EVEN_OK)
	{
		ESP_LOGW(TAG,"Battery read ok\r\n");
		Battery_Decode(ResponseBuffer, &VTAG_DeviceParameter.Bat_Level, &VTAG_DeviceParameter.Bat_Voltage);
	}
	else if(event == EVEN_TIMEOUT)
	{
		ESP_LOGW(TAG,"Battery read timeout\r\n");
	}
	else if(event == EVEN_ERROR)
	{
		ESP_LOGW(TAG,"Battery read error\r\n");
	}
}
void Battery_Calibrate(uint8_t *level)
{
	float calib_bat_level = 0;
	if(*level > 87)
	{
		Bat_raw_pre = calib_bat_level = 100;
	}
	else
	{
		if(*level < Bat_raw_pre)
		{
			Bat_raw_pre = *level;
		}
		else
		{
			if(gpio_get_level(CHARGE) == 1)
			{
				*level = Bat_raw_pre;
			}
			else
			{
				Bat_raw_pre = *level;
			}
		}
		Bat_raw_pre = *level;
		calib_bat_level = (float) *level;

		for(int i = 0; i < 10; i++)
		{
			if(calib_bat_level <= Raw_bat[i] && calib_bat_level > Raw_bat[i+1])
			{
				ESP_LOGI(TAG,"i: %d, a: %f, b: %f \r\n", i, Bat_CalibFactor_a_b[i][0], Bat_CalibFactor_a_b[i][1]);
				calib_bat_level =  roundf(calib_bat_level * Bat_CalibFactor_a_b[i][0] + Bat_CalibFactor_a_b[i][1]);
				break;
			}
		}
	}

	if(calib_bat_level >= 100) calib_bat_level = 100;
//	ESP_LOGW(TAG,"Calib battery level: %f\r\n", calib_bat_level);

	*level = (uint8_t) calib_bat_level;
	ESP_LOGW(TAG,"battery level: %d\r\n", *level);
}

void Battery_Decode(char *buffer, uint8_t *level, uint16_t *voltage)
{
	char Bat_buffer[50];
	strlcpy(Bat_buffer, buffer, 50);

	// skip charge status
	char *tok = strtok(Bat_buffer, ",");

	// skip GPS run status
	tok = strtok(NULL, ",");
	if (tok)
	{
		*level = atoi(tok);
	}

	// skip fix status
	tok = strtok(NULL, "\r\n");
	if ( tok)
	{
		*voltage = atoi(tok);
	}

}
void Battery_Str_Convert(char *str, uint8_t level)
{
	sprintf(str,"{\"Type\":\"DAD\",\"B\":%d,\"MMC\":{\"P\":2,\"M\":1},\"T\":77,\"V\":\"1.1\",\"Cn\":\"2g\"}", level);
}

void Bat_GetPercent(void)
{
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CBC\r\n", "OK", 1000, 2, Batterry_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);
}

void Bat_GetVoltage(void)
{
	ATC_SendATCommand("AT+CBC\r\n", "OK", 1000, 2, Batterry_Callback);
}

