/*
 * sensor.c
 *
 *  Created on: Sep 3, 2021
 *      Author: ASUS
 */
#include "../MyLib/sensor.h"
#include "math.h"
#include "soc/rtc.h"
#include "../MyLib/ESP32_GPIO.h"
#include "../MyLib/led_indicator.h"
#include "../MyLib/Sim7070G_General_Control.h"

SemaphoreHandle_t print_mux = NULL;
xQueueHandle gpio_evt_queue = NULL;
TaskHandle_t xClear_ISR_Handle;
TaskHandle_t xHandle_main_running;

uint16_t count_ISR_P = 0;
uint16_t count_ISR_I = 0;
extern RTC_DATA_ATTR bool Flag_motion_detected;
extern uint16_t acc_counter;
extern RTC_DATA_ATTR uint64_t acc_capture;
extern bool Flag_send_DAST;
extern bool Flag_send_DASP;
extern bool flag_start_motion;
extern bool Flag_sos;
extern bool Flag_update_cfg;
extern uint8_t ACK_Succeed[2];
extern bool Flag_Unpair_Task;
extern bool Flag_mainthread_run;
extern bool Flag_button_cycle_start;
extern TaskHandle_t check_motion_handle;
extern bool Flag_Fota;
extern bool Flag_button_do_nothing;
extern int Ext1_Wakeup_Pin;
extern bool flag_end_motion;
extern void RTC_IRAM_ATTR wake_stub(void);
extern RTC_DATA_ATTR uint64_t t_stop;
extern bool Flag_checkmotin_end;
extern bool Flag_motion_acc_wake_check;
extern RTC_DATA_ATTR uint8_t Backup_Array_Counter;
//extern RTC_DATA_ATTR char Location_Backup_Array[BU_Arr_Max_Num][540];
extern RTC_DATA_ATTR uint8_t Retry_count;
extern RTC_DATA_ATTR uint64_t t_send_backup;
extern RTC_DATA_ATTR uint64_t t_send_voltage;
extern bool Flag_FullBattery;
extern CFG VTAG_Configure;
extern bool wifi_motion_detect;
extern const char *TAG;
extern bool Flag_sleep_dtr;
esp_err_t i2c_master_init(void)
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = I2C_MASTER_SDA_IO,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_io_num = I2C_MASTER_SCL_IO,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 400,
			// .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};
	esp_err_t err = i2c_param_config(i2c_master_port, &conf);
	if (err != ESP_OK) {
		return err;
	}
	return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static esp_err_t i2c_WriteByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t Data)
{
	int ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Register, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	//printf("err write: %d\n", ret);
	i2c_cmd_link_delete(cmd);
	return ret;
	vTaskDelay(1	 / portTICK_PERIOD_MS);
}
static esp_err_t i2c_ReadByte(i2c_port_t i2c_num, uint8_t slaveAdd, uint8_t Register, uint8_t* Data)
{
	int ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, Register, ACK_CHECK_EN);
	//i2c_master_stop(cmd);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slaveAdd << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, Data, NACK_VAL);
	i2c_master_stop(cmd);

	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	//printf("err read: %d\n", ret);
	//printf("read: %02x\n", *Data);
	i2c_cmd_link_delete(cmd);
	return ret;
	vTaskDelay(1 / portTICK_PERIOD_MS);
}

void clear_interrupt_source(void)
{
	uint8_t read = 0;
#ifdef MC3416
	for (int i = 0; i < 2; i++)
	{
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_STAT_2, &read);
		read &= 0x40;
		i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_STAT_2, read);
	}
#endif
#ifdef MC3413
	for (int i = 0; i < 2; i++)
	{
		i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, &read);
		//printf("ISR: %02x\r\n", read);
		//vTaskDelay(1 / portTICK_RATE_MS);
	}
#endif
}
void acc_power_down(void)
{
	uint8_t read;
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x90);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGW(TAG_SENSOR, "Sensor power down\r\n");
}
void acc_power_up(void)
{
	uint8_t read;
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x91);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGW(TAG_SENSOR, "Sensor power up\r\n");
}
static void clear_ISR_task(void *arg)
{
	uint32_t io_num;
	for(;;)
	{
		if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
		{
			count_ISR_I++;
			clear_interrupt_source();
			ESP_LOGW(TAG_SENSOR, "Interupt: %d \r\n", count_ISR_I);
			vTaskDelay(1/portTICK_RATE_MS);
		}
	}
}

static void IRAM_ATTR gpio_sensor_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void check_motion(void* arg)
{
#define ACC_OFF_TIME            60   //sec
#define ACC_WAIT_TIME           60   //sec
	uint32_t time_tik = xTaskGetTickCount();
	uint32_t sec_count = 0;
	bool first_time_dectect = true;
	bool first_time_off = true;
	bool first_time_wait = true;
	bool start_wait = false;
	uint32_t current_dectect = 0;
	uint32_t current_off = 0;
	uint32_t current_wait = 0;
	bool end = true;
	while(1)
	{
		if(VTAG_Configure.MA == 0)
		{
			if(xTaskGetTickCount() - time_tik > 1000/portTICK_PERIOD_MS)
			{
				Flag_checkmotin_end = false;
				sec_count++;
				time_tik = xTaskGetTickCount();

				if(first_time_dectect == true)
				{
					first_time_dectect = false;
					current_dectect = sec_count;
				}
				if(sec_count - current_dectect > 3)
				{
					Flag_checkmotin_end = true;
					//					acc_capture = (uint64_t)round(rtc_time_get());
					//					ESP_LOGW(TAG_SENSOR, "Motion detected\r\n");
					//					Flag_motion_detected = true;
					if(((count_ISR_I > 40 || count_ISR_P > 15 || ((count_ISR_I > 15 || count_ISR_P > 10) && Flag_motion_detected == true))))
					{
						if(Flag_motion_acc_wake_check == false) Flag_motion_acc_wake_check = true;
						start_wait = false;
						first_time_wait = true;
						end = false;
						count_ISR_I = count_ISR_P = 0;
						first_time_dectect = true;
						if(Flag_motion_detected == false && Flag_send_DASP == false)
						{
							if(VTAG_Configure.WM == 0 ||  (VTAG_Configure.WM == 1 && wifi_motion_detect == true))// loi sau khi gui dasp thi nhan dc dast nen gui lai ban tin
							{
								if(VTAG_Configure._SS == 1)
								{
									Flag_send_DAST = true;
								}
								flag_start_motion = true;
								Flag_motion_detected = true;
								flag_end_motion = false;
								if(VTAG_Configure.WM == 0)
								{
									LED_StartMove();
								}
							}
						}
						//Flag_motion_detected = true; // Flag_motion_detected in main.c
						//acc_capture = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get())/1000000);
						acc_capture = (uint64_t)round(rtc_time_get());
						ESP_LOGW(TAG_SENSOR, "Motion detected\r\n");

					}
					else
					{
						clear_interrupt_source();
						clear_interrupt_source();
						count_ISR_I = count_ISR_P = 0;
						first_time_dectect = true;
						while( Flag_button_cycle_start == true);
						vTaskDelay(15 / portTICK_RATE_MS);
						// Addded: Ext1_Wakeup_Pin != CHARGE condition
						while(Flag_button_cycle_start == true);
						if((Ext1_Wakeup_Pin != CHARGE && Flag_Fota == false && Flag_mainthread_run == false && Flag_Unpair_Task == false && Flag_update_cfg == false && Flag_motion_detected == false && Flag_send_DASP == false && esp_sleep_get_wakeup_cause()!= ESP_SLEEP_WAKEUP_UNDEFINED && Flag_send_DAST == false && Flag_sos == false))
						{
							if(ACK_Succeed[0] == 0)
							{
								ESP_sleep(0);
							}
							else if(ACK_Succeed[0] == 3 && ACK_Succeed[1] == 1)
							{
								ESP_sleep(0);
							}
						}

					}
				}
			}
		}
		vTaskDelay(1);
	}
}
double IIR_SOS(double input, uint16_t j, double (*A)[3], double (*B)[3], double (*ftr_Internal)[3])
{
	double result = 0;
	ftr_Internal[j][0] = input - A[j][1] * ftr_Internal[j][1] - A[j][2] * ftr_Internal[j][2];
	result = B[j][0] * ftr_Internal[j][0] + B[j][1] * ftr_Internal[j][1] + B[j][2] * ftr_Internal[j][2];
	ftr_Internal[j][2] = ftr_Internal[j][1];
	ftr_Internal[j][1] = ftr_Internal[j][0];
	return result;
}
///////////////////////////////////////////////////////////////////////////
double IIR_CASCADE(double input, uint16_t k, double (*A)[3], double (*B)[3], double (*ftr_Internal)[3] )
{
	double result = 0;
	double tmp_result = 0;
	uint16_t i;
	tmp_result = input;
	//((A + 0) + 0) = 100;
	for(i = 0; i <k; i++)
	{
		tmp_result = IIR_SOS(tmp_result , i, A, B, ftr_Internal);
	}
	result = tmp_result;
	return result;
}
double A[3][3] = {{0.107390144502567, 0.107390144502567, 0}, {0.0132936811200147, 0.0265873622400294, 0.0132936811200147}, {0.011970951224841, 0.023941902449682, 0.011970951224841}};
double B[3][3] =  {{1, -0.785219710994866, 0}, {1, -1.81024598156098, 0.863420706041043}, {1, -1.63012533207258, 0.678009136971943}};
double acc_z_buf[100][3];
double acc_y_buf[100][3];
double acc_x_buf[100][3];
uint16_t acc_T_counter = 0;
void read_3axis(void * arg)
{
#define THRES 50
#define T	  30
	signed short x = 0, y = 0, z = 0;
	int delta_x = 0, delta_y = 0, delta_z = 0;
	signed short read_f = 0;
	while(1)
	{
		if(VTAG_Configure.MA == 0)
		{
			uint8_t read = 0;
			signed short read_f = 0;
			i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XOUT_EX_H, &read);
			read_f = read;
			i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XOUT_EX_L, &read);
			read_f = read_f << 8 | read;
			delta_x = read_f - x;
			x = read_f;
			if(delta_x > THRES)
			{
				count_ISR_P++;
				ESP_LOGW(TAG_SENSOR, "Polling: %d \r\n", count_ISR_P);
			}

			i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YOUT_EX_H, &read);
			read_f = read;
			i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YOUT_EX_L, &read);
			read_f = read_f << 8 | read;
			delta_y = read_f - y;
			y = read_f;
			if(delta_y > THRES)
			{
				count_ISR_P++;
				ESP_LOGW(TAG_SENSOR, "Polling: %d \r\n", count_ISR_P);
			}

			i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZOUT_EX_H, &read);
			read_f = read;
			i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZOUT_EX_L, &read);
			read_f = read_f << 8 | read;
			delta_z = read_f - z;
			z = read_f;
			if(delta_z > THRES)
			{
				count_ISR_P++;
				ESP_LOGW(TAG_SENSOR,	"Polling: %d \r\n", count_ISR_P);
			}
		}
		vTaskDelay(T / portTICK_RATE_MS);
	}
}
void gpio_init(void)
{
	print_mux = xSemaphoreCreateMutex();
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	io_conf.pin_bit_mask =  ((1ULL<<2));
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	printf("MA: %d", VTAG_Configure.MA);
	if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED )
	{

		xTaskCreate(check_motion, "check_motion", 1024*8, NULL, 5, &check_motion_handle);
	}

	xTaskCreate(read_3axis, "read_3axis", 1024*4, NULL, 5, NULL);
	xTaskCreate(clear_ISR_task, "clear_ISR_task", 1024*2, NULL, 5, NULL);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(2, gpio_sensor_isr_handler, (void*) 2);
	clear_interrupt_source();
	//	}
	if(VTAG_Configure.MA != 0)
	{
		acc_power_down();
	}
}
void acc_config(uint8_t gain, uint16_t thres)
{
	uint8_t read = 0;

#ifdef MC3416
	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x90); // 0x00
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGI(TAG_SENSOR,"MODE: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_CTRL, 0x1f); // 0x1f
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, INTR_CTRL, &read);
	ESP_LOGI(TAG_SENSOR,"INTR_CTRL: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MOTION_CTRL, 0x3f); //0x3f: all, 0x04: ANY only
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MOTION_CTRL, &read);
	ESP_LOGI(TAG_SENSOR, "MOTION_CTRL: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, RANGE, 0x39); //0x09
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, RANGE, &read);
	ESP_LOGI(TAG_SENSOR,"RANGE: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, 0x00); // 0x09
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SR, &read);
	ESP_LOGI(TAG_SENSOR,"SR: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XGAIN, gain);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, XGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"XGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YGAIN, gain);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, YGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"YGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZGAIN, gain);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, ZGAIN, &read);
	ESP_LOGI(TAG_SENSOR,"ZGAIN: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_LSB, thres & 0x00ff);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_MSB, thres >> 8);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_DB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, TF_DB, &read);
	ESP_LOGI(TAG_SENSOR,"TF_DB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_LSB, thres & 0x00ff);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_MSB, (thres & 0x7f00) >> 8);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_DB, 0x00);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, AM_DB, &read);
	ESP_LOGI(TAG_SENSOR,"AM_DB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_LSB, thres & 0x00ff);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_LSB, &read);
	ESP_LOGI(TAG_SENSOR,"SHK_THRESH_LSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_MSB, thres >> 8);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, SHK_THRESH_MSB, &read);
	ESP_LOGI(TAG_SENSOR,"SHK_THRESH_MSB: %02x", read);

	i2c_WriteByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, 0x91);
	i2c_ReadByte(I2C_MASTER_NUM, MC3413_I2C_ADDR, MODE, &read);
	ESP_LOGI(TAG_SENSOR,"MODE: %02x", read);
	clear_interrupt_source();
#endif
}
