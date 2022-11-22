/*
 * sensor.h
 *
 *  Created on: Sep 6, 2021
 *      Author: HAOHV6
 */

#ifndef MAIN_INCLUDE_SENSOR_H_
#define MAIN_INCLUDE_SENSOR_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"

#define TAG_SENSOR "SENSOR"
#define THRES 100
#define T	  10
/*************I2C DEFINE*****************/
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)
#define CONFIG_I2C_MASTER_SCL 22
#define CONFIG_I2C_MASTER_SDA 21
#define CONFIG_I2C_MASTER_PORT_NUM 1
#define CONFIG_I2C_MASTER_FREQUENCY 100000
#define CONFIG_I2C_SLAVE_SCL 5
#define CONFIG_I2C_SLAVE_SDA 4
#define CONFIG_I2C_SLAVE_PORT_NUM 0
#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/*************GPIO DEFINE*****************/
#define GPIO_INPUT_IO_0     0
//#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define ESP_INTR_FLAG_DEFAULT 0

#define MC3416
/*************MC3416 REGISTER***************/
#ifdef MC3416
#define MC3413_I2C_ADDR         0x4C
#define DEV_STAT                0x05
#define INTR_CTRL               0x06
#define MODE                    0x07
#define SR                      0x08
#define MOTION_CTRL             0x09
#define XOUT_EX_L               0x0D
#define XOUT_EX_H               0x0E
#define YOUT_EX_L               0x0F
#define YOUT_EX_H               0x10
#define ZOUT_EX_L               0x11
#define ZOUT_EX_H               0x12
#define STATUS_2                0x13
#define INTR_STAT_2             0x14
#define RANGE                   0x20
#define XOFFL                   0x21
#define XOFFH                   0x22
#define YOFFL                   0x23
#define YOFFH                   0x24
#define ZOFFL                   0x25
#define ZOFFH                   0x26
#define XGAIN                   0x27
#define YGAIN                   0x28
#define ZGAIN                   0x29
#define TF_THRESH_LSB           0x40
#define TF_THRESH_MSB           0x41
#define TF_DB                   0x42
#define AM_THRESH_LSB           0x43
#define AM_THRESH_MSB           0x44
#define AM_DB                   0x45
#define SHK_THRESH_LSB          0x46
#define SHK_THRESH_MSB          0x47
#define PK_P2P_DUR_THRESH_LSB   0x48
#define PK_P2P_DUR_THRESH_MSB   0x49
#define TIMER_CTRL              0x4A
#endif

/*************MC3413 REGISTER***************/
#ifdef MC3413
#define MC3413_I2C_ADDR         0x4C
#define SR                      0x03
#define OPSTAT                  0x04
#define INTEN                   0x06
#define MODE                    0x07
#define SRTFR                   0x08
#define TAPEN                   0x09
#define TTTRX                   0x0A
#define TTTRY                   0x0B
#define TTTRZ                   0x0C
#define XOUT_EX_L               0x0D
#define XOUT_EX_H               0x0E
#define YOUT_EX_L               0x0F
#define YOUT_EX_H               0x10
#define ZOUT_EX_L               0x11
#define ZOUT_EX_H               0x12
#define OUTCFG                  0x20
#define XOFFL                   0x21
#define XOFFH                   0x22
#define YOFFL                   0x23
#define YOFFH                   0x24
#define ZOFFL                   0x25
#define ZOFFH                   0x26
#define XGAIN                   0x27
#define YGAIN                   0x28
#define ZGAIN                   0x29
#define PCODE                   0x3B
#endif

esp_err_t i2c_master_init(void);
static esp_err_t i2c_WriteByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t Data);
static esp_err_t i2c_ReadByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t* Data);
 void clear_interrupt_source(void);
void acc_power_down(void);
void acc_power_up(void);
static void clear_ISR_task(void *arg);
static void enter_deep_sleep(int wake_pin);
static void IRAM_ATTR gpio_isr_handler(void* arg);
static void check_motion(void* arg);
void gpio_init(void);
void acc_config(uint8_t gain, uint16_t thres);
void read_3axis(void * arg);
#endif /* MAIN_INCLUDE_SENSOR_H_ */
