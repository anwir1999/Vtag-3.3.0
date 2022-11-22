/*
 * ESP32_GPIO.h
 *
 *  Created on: Aug 31, 2021
 *      Author: QuangDan
 */

#ifndef MYLIB_ESP32_GPIO_H_
#define MYLIB_ESP32_GPIO_H_

#include "../MyLib/Common.h"


#define PowerLatch     	26   // Cap nguon toan mach
#define PowerKey     	32	 // PWR 7070
#define VCC_7070_EN		19	 // Cap nguon 7070
#define VCC_GPS_EN		18   // Cap nguon anten GPS
#define UART_SW			23
#define LED_1			5  	 // led xanh
#define LED_2			15	 // led do
#define DTR_Sim7070_3V3	27

#define BUTTON  		25
#define ACC_INT			2
#define RI_Sim7070_3V3	34
#define CHARGE			35

#define SW_UART_AT		0
#define SW_UART_GNSS	1

#define ESP32_GPIO_OUTPUT_PIN_SEL 	((1ULL<<PowerLatch) | (1ULL<<PowerKey) | (1ULL<<VCC_7070_EN) | (1ULL<<VCC_GPS_EN) | (1ULL<<UART_SW) | (1ULL<<LED_1) | (1ULL<<LED_2) | (1ULL<<DTR_Sim7070_3V3))
#define ESP32_GPIO_INPUT_PIN_SEL 	((1ULL << BUTTON) | (1ULL << RI_Sim7070_3V3) | (1ULL << CHARGE))

#endif /* MYLIB_ESP32_GPIO_H_ */
