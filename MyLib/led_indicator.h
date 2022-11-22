#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void LED_TurnOn();
void LED_TurnOff();
void LED_ChangeMode();
void LED_SendSOS();
void LED_Fota();
void LED_UnpairAndCfg();
void LED_StartMove();
void LED_StopMove();
void LED_Pair();
