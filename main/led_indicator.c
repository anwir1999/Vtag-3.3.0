#include "../MyLib/ESP32_GPIO.h"

extern bool Flag_Fota_led;
extern bool Flag_Unpair_led;
extern bool Flag_led_ble;
extern bool Flag_test_unpair;
extern bool Flag_wifi_got_led;
extern bool Flag_check_run;
extern bool Flag_new_firmware_led;
extern bool Flag_button_cycle_start;
extern SemaphoreHandle_t xMutex_LED;
extern RTC_DATA_ATTR char Device_PairStatus[5];
extern const char *TAG;
void LED_TurnOn()
{
	while(Flag_check_run == false) vTaskDelay(50/portTICK_PERIOD_MS);
	if(strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0)
	{
		for(int i = 0; i < 3; i++)
		{
			gpio_set_level(LED_1, 1);
//			gpio_set_level(LED_2, 1);
			vTaskDelay(150/portTICK_PERIOD_MS);
			gpio_set_level(LED_1, 0);
//			gpio_set_level(LED_2, 0);
			vTaskDelay(150/portTICK_PERIOD_MS);
		}
	}
	else
	{
		for(int i = 0; i < 3; i++)
		{
			gpio_set_level(LED_1, 1);
			gpio_set_level(LED_2, 1);
			vTaskDelay(150/portTICK_PERIOD_MS);
			gpio_set_level(LED_1, 0);
			gpio_set_level(LED_2, 0);
			vTaskDelay(150/portTICK_PERIOD_MS);
		}
	}
	vTaskDelay(1000/portTICK_PERIOD_MS);
}

void LED_TurnOff()
{
	if(gpio_get_level(CHARGE) == 1)
	{
		if(strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0)
		{
			for(int i = 0; i < 3; i++)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(300/portTICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				vTaskDelay(300/portTICK_PERIOD_MS);
			}
		}
		else
		{
			for(int i = 0; i < 3; i++)
			{
				gpio_set_level(LED_2, 1);
				vTaskDelay(300/portTICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				vTaskDelay(300/portTICK_PERIOD_MS);
			}
		}
	}
	else
	{
		if(strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0)
		{
			for(int i = 0; i < 3; i++)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(300/portTICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				vTaskDelay(300/portTICK_PERIOD_MS);
			}
		}
		else
		{
			for(int i = 0; i < 3; i++)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(300/portTICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				vTaskDelay(300/portTICK_PERIOD_MS);
			}
		}
	}
}

void LED_ChangeMode()
{
	if(gpio_get_level(CHARGE) == 1)
	{
		for(int i = 0; i < 10; i++)
		{
			gpio_set_level(LED_2, 1);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_2, 0);
			vTaskDelay(100/portTICK_PERIOD_MS);
		}

	}
	else
	{
		for(int i = 0; i < 10; i++)
		{
			gpio_set_level(LED_1, 1);
			vTaskDelay(100/portTICK_PERIOD_MS);
			gpio_set_level(LED_1, 0);
			vTaskDelay(100/portTICK_PERIOD_MS);
		}
	}
}

void LED_SendSOS()
{
	if(gpio_get_level(CHARGE) == 1)
	{
		vTaskDelay(200/portTICK_PERIOD_MS);
		for(int i = 0; i < 2; i++)
		{
			gpio_set_level(LED_2, 1);
			vTaskDelay(150/portTICK_PERIOD_MS);
			gpio_set_level(LED_2, 0);
			vTaskDelay(150/portTICK_PERIOD_MS);
		}
	}
	else
	{
		vTaskDelay(200/portTICK_PERIOD_MS);
		for(int i = 0; i < 2; i++)
		{
			gpio_set_level(LED_1, 1);
			vTaskDelay(150/portTICK_PERIOD_MS);
			gpio_set_level(LED_1, 0);
			vTaskDelay(150/portTICK_PERIOD_MS);
		}
	}

}

void LED_Fota()
{
	if(gpio_get_level(CHARGE) == 1)
	{
		while(Flag_Fota_led == true)
		{
			if(Flag_wifi_got_led == false)
			{
				for(int i = 0; i < 4; i++)
				{
					gpio_set_level(LED_2, 1);
					vTaskDelay(150/portTICK_PERIOD_MS);
					gpio_set_level(LED_2, 0);
					vTaskDelay(150/portTICK_PERIOD_MS);
				}
				vTaskDelay(2000/portTICK_PERIOD_MS);
			}
			else
			{
				for(int i = 0; i < 4; i++)
				{
					gpio_set_level(LED_2, 1);
					vTaskDelay(75/portTICK_PERIOD_MS);
					gpio_set_level(LED_2, 0);
					vTaskDelay(75/portTICK_PERIOD_MS);
				}
				vTaskDelay(1000/portTICK_PERIOD_MS);
			}
		}
	}
	else
	{
		while(Flag_Fota_led == true)
		{
			if(Flag_wifi_got_led == false)
			{
				for(int i = 0; i < 4; i++)
				{
					gpio_set_level(LED_2, 1);
					vTaskDelay(150/portTICK_PERIOD_MS);
					gpio_set_level(LED_2, 0);
					vTaskDelay(150/portTICK_PERIOD_MS);
				}
				vTaskDelay(2000/portTICK_PERIOD_MS);
			}
			else
			{
				for(int i = 0; i < 4; i++)
				{
					gpio_set_level(LED_2, 1);
					vTaskDelay(75/portTICK_PERIOD_MS);
					gpio_set_level(LED_2, 0);
					vTaskDelay(75/portTICK_PERIOD_MS);
				}
				vTaskDelay(1000/portTICK_PERIOD_MS);
			}
		}
	}
}

void LED_UnpairAndCfg()
{
	if(gpio_get_level(CHARGE) == 1)
	{
		while(Flag_Unpair_led == true)
		{
			xSemaphoreTake(xMutex_LED, portMAX_DELAY);
			for(int i = 0; i < 2; i++)
			{
				gpio_set_level(LED_2, 1);
				vTaskDelay(200/portTICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				vTaskDelay(200/portTICK_PERIOD_MS);
			}
			xSemaphoreGive(xMutex_LED);
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
	else
	{
		while(Flag_Unpair_led == true)
		{
			xSemaphoreTake(xMutex_LED, portMAX_DELAY);
			for(int i = 0; i < 2; i++)
			{
				ESP_LOGE(TAG, "Lock_xMutex_LED_blik\r\n");
				gpio_set_level(LED_1, 1);
				vTaskDelay(200/portTICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				vTaskDelay(200/portTICK_PERIOD_MS);
				ESP_LOGE(TAG, "Unlock_xMutex_LED_blik\r\n");
			}
			xSemaphoreGive(xMutex_LED);
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
}
void LED_StartMove()
{
	if(gpio_get_level(CHARGE) == 1)
	{
		for(int i = 0; i < 3; i++)
		{
			gpio_set_level(LED_2, 1);
			vTaskDelay(300/portTICK_PERIOD_MS);
			gpio_set_level(LED_2, 0);
			vTaskDelay(300/portTICK_PERIOD_MS);
		}
	}
	else
	{
		for(int i = 0; i < 3; i++)
		{
			gpio_set_level(LED_1, 1);
			vTaskDelay(300/portTICK_PERIOD_MS);
			gpio_set_level(LED_1, 0);
			vTaskDelay(300/portTICK_PERIOD_MS);
		}
	}
}

void LED_StopMove()
{
	for(int i = 0; i < 3; i++)
	{
		gpio_set_level(LED_1, 1);
		vTaskDelay(200/portTICK_PERIOD_MS);
		gpio_set_level(LED_1, 0);
		vTaskDelay(200/portTICK_PERIOD_MS);
	}
}

void LED_Pair()
{
	while(strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0)
	{
		while(Flag_button_cycle_start == true);
		if(Flag_new_firmware_led == true)
		{
			for(int i = 0; i < 10; i++)
			{
				gpio_set_level(LED_1, 1);
				gpio_set_level(LED_2, 1);
				vTaskDelay(150/portTICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				gpio_set_level(LED_2, 0);
				vTaskDelay(150/portTICK_PERIOD_MS);
			}
			Flag_new_firmware_led = false;
		}
		for(int i = 0; i < 3; i++)
		{
			if(Flag_new_firmware_led == true)
			{
				break;
			}
			gpio_set_level(LED_1, 1);
			vTaskDelay(200/portTICK_PERIOD_MS);
			gpio_set_level(LED_1, 0);
			vTaskDelay(200/portTICK_PERIOD_MS);
		}
		if(Flag_Fota_led == true || Flag_test_unpair == true) break;
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
