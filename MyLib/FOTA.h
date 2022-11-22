/*
 * FOTA.h
 *
 *  Created on: Sep 16, 2021
 *      Author: HAOHV6
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_smartconfig.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "../Mylib/MQTT.h"
#ifndef MYLIB_FOTA_H_
#define MYLIB_FOTA_H_

void init_OTA_component();
void check_update_task(void *pvParameter);
void ini_wifi();
void smartconfig_task(void *parm);
void forcedReset();
void HTTPFSesponse_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void readFSCallback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
esp_err_t  update_2G_handle();
#endif /* MYLIB_FOTA_H_ */
