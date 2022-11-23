/* FreeRTOS Real Time main Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

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
#include "freertos/FreeRTOSConfig.h"
#include "esp_task_wdt.h"
#include "soc/rtc_wdt.h"
#include "soc/sens_reg.h"
#include "soc/syscon_reg.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "esp_int_wdt.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_spiffs.h"
#include "../MyLib/AT_Function.h"
#include "../MyLib/Common.h"
#include "../MyLib/GPS.h"
#include "../MyLib/Sim7070G_General_Control.h"
#include "../MyLib/MQTT.h"
#include "../MyLib/Sim7070G_Battery.h"
#include "../MyLib/ESP32_GPIO.h"
#include "../MyLib/FOTA.h"
#include "../MyLib/sensor.h"
#include "../Mylib/led_indicator.h"
#include "../Mylib/SPIFFS_user.h"
#include "../Mylib/string_user.h"
#define RTC_SLOW_MEM ((uint32_t*) 0x50000000)       /*!< RTC slow memory, 8k size */

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

const char *TAG = "VTAG_ESP32";

#define	ENABLE_SLEEP_AT_STARTUP 0
#define BU_Arr_Max_Num 		 	10
#define SS_THR 65
#define VOFF_SET	20
#define THRESHOLD   3970
#define LOG 0
char MQTT_SubMessage[200] = "";
char SMS_MessageReceive[400] = "";
//char MAC_Serial_curent[250] = {0};
char MAC_Serial_Cur[250] = {0};
char MAC_Serial_Pre[250] = {0};
char MAC_Weak_Serial_Pre[250] = {0};
RTC_DATA_ATTR char VTAG_Vesion[10]		    =		"S3.4.3";
RTC_DATA_ATTR char VTAG_Version_next[10] 	=		{0};
RTC_DATA_ATTR char DeviceID_TW_Str[50]= "";
RTC_DATA_ATTR char Location_Backup_Array[BU_Arr_Max_Num][540];
RTC_DATA_ATTR uint8_t Backup_Array_Counter = 0;
RTC_DATA_ATTR bool Flag_Sim7070G_Started = false;
RTC_DATA_ATTR bool Flag_check_run = false;
RTC_DATA_ATTR CFG VTAG_Configure = {.Period = 5, .Mode = 1, .Network = 2, .CC = 1, ._SS = 1, .WM = 0, ._lc = 1, .MA = 0, .BT = 0, .FLR = 0, .F2G = 0};
RTC_DATA_ATTR uint8_t Retry_count = 0;
RTC_DATA_ATTR bool Flag_motion_detected = false;
RTC_DATA_ATTR bool Flag_period_wake = true;
RTC_DATA_ATTR bool Flag_acc_wake = false;
RTC_DATA_ATTR bool Flag_calib = false;
RTC_DATA_ATTR uint64_t Gettimeofday_capture = 0;
RTC_DATA_ATTR char Device_PairStatus[5] = {0};
RTC_DATA_ATTR bool Flag_asyn_ts = false;
//RTC_DATA_ATTR uint64_t TimeOfDay_capture = 0;
RTC_DATA_ATTR bool Flag_reboot_7070 = false;
uint8_t ACK_Succeed[2] = {0,0};
RTC_DATA_ATTR float Calib_factor = 1.05;
RTC_DATA_ATTR uint64_t t_stop_calib = 0;
RTC_DATA_ATTR uint64_t t_slept_calib = 0;
RTC_DATA_ATTR uint8_t retry = 0; //retry each time backup same data
RTC_DATA_ATTR uint16_t Fifo_bat_arr[10] = {0};
RTC_DATA_ATTR uint8_t Fifo_bat_index = 0;
RTC_DATA_ATTR uint8_t Fail_network_counter = 0;
RTC_DATA_ATTR uint8_t Reboot_reason = 0;
RTC_DATA_ATTR BU_reason Backup_reason = 0;
//-------------------------------// RTC_DATA_ATTR data for calib and send DBL mess
RTC_DATA_ATTR uint8_t Bat_raw_pre = 100;
//RTC_DATA_ATTR uint8_t Bat_raw_pre_charging = 0;
RTC_DATA_ATTR bool DBL_15 = false;
RTC_DATA_ATTR bool DBL_10 = false;
RTC_DATA_ATTR bool DBL_5 = false;
RTC_DATA_ATTR bool Flag_charged = false;
RTC_DATA_ATTR float Calib_adc_factor = 1;
RTC_DATA_ATTR float Calib_Pin_factor = 1;
RTC_DATA_ATTR uint8_t Flag_count_nb_backup = 0;
RTC_DATA_ATTR bool Flag_use_2g = false;
RTC_DATA_ATTR bool Flag_config_sms = false; // trong che do only GPS, neu do dc GPS thì flag = true
//RTC_DATA_ATTR bool Flag_simcom_cpos = false; // trong che do only GPS, neu do dc GPS thì flag = true
bool Flag_config_sendLocation = false;
CFG VTAG_Configure_temp;

bool Flag_send_DAST = false;
bool Flag_send_DASP = false;

RTC_DATA_ATTR uint64_t t_actived = 0;
RTC_DATA_ATTR uint64_t t_stop = 0;
RTC_DATA_ATTR uint64_t t_total_passed_vol = 0;
RTC_DATA_ATTR uint64_t t_total_passed_BU = 0;
//RTC_DATA_ATTR bool Flag_send_vol_percent_incharge = false;
RTC_DATA_ATTR PIN_activity Check_vol_percent = NO_AC;
RTC_DATA_ATTR bool Flag_mess_sended_wd = false;
RTC_DATA_ATTR bool Flag_wifi_loop = true;
bool BU_checked = false;
bool vol_checked = false;
uint64_t acc_counter = 0;
RTC_DATA_ATTR uint64_t acc_capture = 0;
uint64_t time_slept = 0;
uint64_t t_acc = 0;
bool flag_end_motion = false;
bool flag_start_motion = false;
Network_Signal VTAG_NetworkSignal;
RTC_DATA_ATTR Device_Param VTAG_DeviceParameter;
RTC_DATA_ATTR bool Select_NB = false;
Device_Flag VTAG_Flag;
VTAG_MessageType VTAG_MessType_G;
RTC_DATA_ATTR uint32_t AP_count_pre = 0;
RTC_DATA_ATTR uint32_t test_bk = 0;
RTC_DATA_ATTR time_t timestampSMS = 0;
RTC_DATA_ATTR int count_loop = 0;
//--------------------------------------------------------------------------------------------------------------// Define for common operating flags
bool Flag_ScanNetwok = false;
bool Flag_Cycle_Completed = false;
bool Flag_ActiveNetwork = false;
bool Flag_DeactiveNetwork = false;
bool Flag_Network_Active = false;
bool Flag_Network_Check_OK = false;
bool Flag_Wait_Exit = false;
bool Flag_GPS_Started = false;
bool Flag_GPS_Stopped = false;
bool Flag_Timer_GPS_Run = false;
bool Flag_Device_Ready = false;
bool Flag_WifiScan_Request = false;
//bool Flag_WifiScan_End = false;
bool Flag_WifiCell_OK = false;
bool Flag_Restart7070G_OK = false;
bool Flag_CFUN_0_OK = false;
bool Flag_CFUN_1_OK = false;
bool Flag_Control_7070G_GPIO_OK = false;
bool Flag_SelectNetwork_OK = false;
bool Flag_NeedToProcess_GPS = false;
bool Flag_MQTT_Stop_OK = false;
bool Flag_Reboot7070_OK = false;
bool Flag_NeedToReboot7070 = false;
// Need adding to reset parameters funtion
bool Flag_MQTT_Connected = false;
bool Flag_MQTT_Sub_OK = false;
bool Flag_MQTT_Publish_OK = false;
bool Flag_MQTT_SubMessage_Processing = false;
bool Flag_LowBattery = false;
bool Flag_wifi_init = false;
bool Flag_Done_Get_Accurency = false;
bool Flag_sos = false;
//bool Flag_wifi_scan = false;
bool Flag_update_cfg = false;
bool Flag_checkmotin_end = false;
bool Flag_Unpair_Task = false;
bool Flag_Pair_Task = false;
bool Flag_DBL_Task = false;
bool Flag_button_do_nothing = false;
bool Flag_mainthread_run = false;
bool Flag_button_cycle_start = false;
bool Flag_backup_data = false;
bool Flag_shutdown_dev = false;
bool Flag_sending_backup = false;
bool Flag_Unpair_led = false;
bool Flag_led_ble = false;
bool Flag_Fota_led = false;
bool Flag_Fota = false;
bool Flag_Fota_success = false;
bool Flag_Fota_fail = false;
bool Flag_button_do_nothing_led = false;
bool Flag_button_long_press_led = false;
bool Flag_wakeup_led = false;
bool Flag_FullBattery = false;
bool Flag_ChangeMode_led = false;
bool Flag_ChangeMode = false;
bool Flag_StopMotion_led = false;
bool Flag_test_unpair = false;
bool Flag_wifi_got_led = false;
bool Flag_wifi_scanning = false;
bool Flag_unpair_DAST = false;
bool Flag_charge_in_wake = false;
bool Flag_need_check_accuracy = false;
bool Flag_motion_acc_wake_check = false;
bool Flag_new_firmware_led = false;
bool Flag_location_send_1 = false;
bool Flag_location_send_2 = false;
bool Flag_skip_rst_acc_ISR = false;
bool wifi_motion_detect = true;
bool Flag_sms_receive = false; // true neu nhu nhan ban tin sms hoac chan RI
bool Flag_config = false; // true neu dang trong qua trinh gui ban tin
bool Flag_wifi_detected = false;
//bool Flag_bleRstWdt = false; //reset wdt sau moi 30s
RTC_DATA_ATTR bool Flag_test_connectSimcom = false;
//--------------------------------------------------------------------------------------------------------------//
//uint8_t VTAG_TrackingMode = 0;
//uint32_t VTAG_TrackingPeriod = 0;
//uint8_t VTAG_TrackingZone = 0;
//uint8_t VTAG_TrackingNetwork = 0;
//uint8_t VTAG_TrackingAccuracy = 0;
//char VTAG_TrackingType;
//--------------------------------------------------------------------------------------------------------------//
uint64_t bitmap_wakeup = 0;
uint16_t Reboot7070_Delay_Counter = 0;
uint8_t  ProgramRun_Cause = 0;
int16_t  CSQ_value = 0;
uint8_t Network_Type = GSM;
uint8_t ATC_Sent_TimeOut;
uint8_t stepConn = 0, stepDisconn = 0, stepSendData = 0, stepNetworkMode = 0;
//total AP scan
uint16_t ap_count = 0;
uint32_t count_list = 0;
uint8_t charging_time = 0;
uint16_t RTOS_TICK_PERIOD_MS = portTICK_PERIOD_MS;
uint32_t VTAG_Tracking_Period = 120; // Second
uint8_t button_led_indcator = 0;
uint8_t wifi_detect_int =0;
int count_down_minutes = 20;
int num_sms = 0;
int FlagSendLR = 0;
int FlagFota2G = 0;
wifi_detect_reason wd_reason = 0;
char IMSI[20] = {0};
char config_buff[100];
char Device_shutdown_ts[15] = {0};
//--------------------------------------------------------------------------------------------------------------// Define for loop reason
loopmain_reason_t loopmain_reason = NO_LOOPMAIN;
loopbutton_reason_t loopbutton_reason = NO_LOOPBUTTON;
void saveLoopReason();
char loopString[50] = {0};
//--------------------------------------------------------------------------------------------------------------// Define for tasks
#define MAIN_TASK_PRIO     			1
#define UART_RX_TASK_PRIO      		2
#define CHECKTIMEOUT_TASK_PRIO     	3

TaskHandle_t test_main_task_handle;
TaskHandle_t main_task_handle;
TaskHandle_t uart_rx_task_handle;
TaskHandle_t check_timeout_task_handle;
TaskHandle_t gps_scan_task_handle;
TaskHandle_t gps_processing_task_handle;
TaskHandle_t mqtt_submessage_processing_handle;
TaskHandle_t Fota2G_processing_handle;
TaskHandle_t check_motion_handle;
TaskHandle_t led_indicator_handle;

static SemaphoreHandle_t sync_main_task;
static SemaphoreHandle_t sync_uart_rx_task;
static SemaphoreHandle_t sync_check_timeout_task;
static SemaphoreHandle_t sync_gps_scan_task;
static SemaphoreHandle_t sync_gps_processing_task;
static SemaphoreHandle_t sync_mqtt_submessage_processing_task;
static SemaphoreHandle_t sync_charging_task;
SemaphoreHandle_t xMutex_LED;
//--------------------------------------------------------------------------------------------------------------// Define for OTA
#define CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL "https://192.168.2.106:8070/hello-world.bin"
#define CONFIG_EXAMPLE_OTA_RECV_TIMEOUT 5000
#define CONFIG_EXAMPLE_CONNECT_WIFI 1
#define CONFIG_EXAMPLE_WIFI_SSID "myssid"
#define CONFIG_EXAMPLE_WIFI_PASSWORD "mypassword"

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256
//--------------------------------------------------------------------------------------------------------------// Define for FAT flash
#define	DEVICEID_FLASH_STORE	0
// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
// Mount path for the partition
//char *base_path = "/spiflash";
char *base_path = "/spiffs";

void MountingFATFlash(void);
bool WritingFATFlash(char* FilePath, char* FileName, char* FileContent);
bool ReadingFATFlash(char* FilePath, char* FileName, char* buffer);
//--------------------------------------------------------------------------------------------------------------// Define for button
#define Button_LongPressed		100 // Equivalent to 3s
#define	Button_ShortPressed		40  // Equivalent to 400 ms
#define	Button_MinPressed		4  // Equivalent to 40 ms

uint8_t Button_Long_Pressed_counter = 0;
uint8_t Button_Pressed_Counter = 0;
uint16_t Button_Pressed_Duration = 0;
uint16_t Button_EndCheck_Counter = 0;
bool Flag_Button_Duration_Start = false;
bool Flag_Button_EndCheck_Start = false;
bool Flag_button_EndCheck = false;
uint16_t Charge_Delay_Counter = 0;
bool Flag_Charge_Delay_Start = false;
//--------------------------------------------------------------------------------------------------------------// Define for watch dog timer
#define TWDT_TIMEOUT_S         50
#define TASK_RESET_PERIOD_S     2

#define CHECK_ERROR_CODE(returned, expected) ({                        \
		if(returned != expected){                                  \
			printf("TWDT ERROR\n");                                \
			abort();                                               \
		}                                                          \
})
//extern void esp_task_wdt_isr_user_handler(void);
//--------------------------------------------------------------------------------------------------------------// Define for ADC
#if CONFIG_IDF_TARGET_ESP32
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
#define Bat_Channel    				  ADC2_CHANNEL_0
uint8_t output_data=0;
int     read_raw;
esp_err_t r;
gpio_num_t adc_gpio_num;
float batt_vol_prev;

void RTC_IRAM_ATTR adc1_get_raw_ram(adc1_channel_t channel);
void RTC_IRAM_ATTR adc2_get_raw_ram(adc2_channel_t channel);
//--------------------------------------------------------------------------------------------------------------// Define for charging
typedef enum charge_state_t {UNDEFINED, DISRUPTED, IN_CHARGING, FULL_BATTERY, STOP_CHARGING};
enum charge_state_t state;
#define NO_OF_SAMPLES 64
//--------------------------------------------------------------------------------------------------------------// Define for SMS
#define DirectPassSMS   0
#define BufferedSMS     1

typedef struct{
	char dataRec[128];
	char dataTrans[128];
} SMSdataType_t;

//bool Flag_SMS_Start = true;
//static uint8_t SMS_Send_Step = 0;
//static SMSdataType_t  SMSdataType;
//uint8_t SMS_Receive_Method = DirectPassSMS;
//void Set7070ToSleepMode(void);
//void SMS_Init(void);
//void SMS_Start(void);
//void SMS_Data_Callback(void *ResponseBuffer);
//void SMS_Process_Configure(uint8_t method);
//void SMS_Read_Process(void *ResponseBuffer);
//static void SMS_Operation_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void receive_sms();
//--------------------------------------------------------------------------------------------------------------// Define for deep sleep mode
#define CONFIG_EXAMPLE_EXT1_WAKEUP 1
//static RTC_DATA_ATTR struct timeval sleep_enter_time;
int sleep_time_ms = 0;
int using_device_time_s = 0;
int wakeup_time_sec = 10;
//--------------------------------------------------------------------------------------------------------------// Define for timer
#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE    (APB_CLK_FREQ / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(APB_CLK_FREQ / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (0.25)   /*!< test interval for timer 0 */
int timer_group = TIMER_GROUP_0;

int timer_idx = TIMER_0;
int TrackingPeriod_timer_idx = TIMER_1;
int cnt = 0;
int TrackingRuntime = 0;

static void tg0_timer0_init();
void IRAM_ATTR timer_group0_isr(void *para);
static void TrackingPeriod_Timer_Init();
//--------------------------------------------------------------------------------------------------------------// Define for wifi scan
#define DEFAULT_SCAN_LIST_SIZE 10
char Wifi_Buffer[500];
//--------------------------------------------------------------------------------------------------------------// Define for GPS
#define GPS_Indirect				0
#define GPS_NMEA					1
#define GPS_Select					GPS_NMEA

//char *GNSS_Data_Str;
char GNSS_Data_Str[2048];
gps_t hgps;
uint16_t GPS_Scan_Number = 45;
uint8_t GPS_Fix_Status = 0;
uint16_t GPS_Scan_Counter = 0;
uint16_t GPS_Scan_Period = 2;// 2 second

float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute, sec;
void GPS_Operation_Thread(void);
void GPS_Read(void);
static void StartGPS_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void StopGPS_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void ScanGPS_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void GPS_Data_NMEA_Callback(void *ResponseBuffer);
void GPS_Process_Callback(char *ATC_Str_Buffer);
void GPS_Position_Time_Str_Convert(char *str, uint8_t bat_lev, float accuracy, float lat, float lon, float alt, uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec);
uint8_t GPS_Decode(char *buffer, float *lat, float *lon, float *speed_kph, float *heading, float *altitude,
		uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec);
//--------------------------------------------------------------------------------------------------------------// Define for ESP32 GPIO

//#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;
//--------------------------------------------------------------------------------------------------------------// Function
long convert_date_to_epoch(int day_t,int month_t,int year_t ,int hour_t,int minute_t,int second_t);
time_t string_to_seconds(const char *timestamp_str);
void GetDeviceTimestamp(void);
//int filter_comma(char *respond_data, int begin, int end, char *output);
void MQTT_Location_Payload_Convert(void);
void BackUp_UnsentMessage(VTAG_MessageType Mess_Type);
//--------------------------------------------------------------------------------------------------------------// Wifi cell
void Bat_Wifi_Cell_Convert(char* str, char* wifi, uint8_t level, int MCC,int16_t RSRP, int16_t RSRQ, int RSSI,int cellID,int LAC,int MNC);
void CPSI_Decode(char* str, int* MCC, int* MNC, int* LAC, int* cell_ID, int* RSSI, int16_t* RSRSP, int16_t* RSRQ);
//void Bat_Wifi_Cell_Convert(char* str, char* wifi, uint8_t level, int MCC,int RSSI,int cellID,int LAC,int MNC);
char *str_str (char *s, char *s1, char *s2);
//--------------------------------------------------------------------------------------------------------------// Json
static void wifi_scan(void);
static void wifi_init(void);
static int json_add_num(cJSON *parent, const char *str, const double item);
static int json_add_str(cJSON *parent, const char *str, const char *item);
static int json_add_ap(cJSON *aps, const char * bssid, int rssi);
static int json_add_obj(cJSON *parent, const char *str, cJSON *item);
void JSON_Analyze(char* my_json_string, CFG* config);
void JSON_Analyze_init_config(char* my_json_string, CFG* config);
void JSON_Analyze_backup(char* my_json_string, CFG* config);
//--------------------------------------------------------------------------------------------------------------// Funtion for controlling 7070G
void PWRKEY_7070G_GPIO_Init();
void CheckSleepWakeUpCause(void);
void Restart7070G_Hard(void);
void Restart7070G_Soft(void);
static void GPIO_7070G_Control_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void Restart7070G_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
//--------------------------------------------------------------------------------------------------------------// Reset all parameters
void ResetAllParameters(void);
//--------------------------------------------------------------------------------------------------------------// Network
char CPSI_Str[128];
static void ScanNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void ActiveNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void CheckNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void DeactiveNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void SetCFUN_0_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void SetCFUN_1_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
static void SelectNB_GSM_Network_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer);
void SelectNB_GSM_Network(uint8_t network);
void MQTT_SendMessage_Thread(VTAG_MessageType Mess_Type);
//--------------------------------------------------------------------------------------------------------------// Wait processing
void WaitandExitLoop(bool *Flag);
//--------------------------------------------------------------------------------------------------------------// ESP32 function
float read_battery(void);
void Bat_ADC_Init(void);
void ESP32_Clock_Config(int Freq_Max, int Freq_Min, bool LighSleep_Select);
void SetESP32Sleep(void);
void ESP32_GPIO_Input_Init(void);
void ESP32_GPIO_Output_Init(void);
void ESP32_GPIO_Output_SetDefault(void);
void ESP32_GPIO_Output_Config(void);
//--------------------------------------------------------------------------------------------------------------// Pre Operation configure
void PreOperationInit(void);
void Create_Tasks(void);
void Delete_Tasks(void);
void Check_accuracy(void);
void Check_battery(void);
//--------------------------------------------------------------------------------------------------------------// ULP
static void init_ulp_program(void);
//--------------------------------------------------------------------------------------------------------------// Wake stub
void RTC_IRAM_ATTR wake_stub(void);
RTC_DATA_ATTR int RTC_ADC;
RTC_DATA_ATTR int wake_count;
int Ext1_Wakeup_Pin = 0;
//-------------------------------------------------LBS----------------------------------------------------------/
char CENG_buf[300];
char cell_buffer[600];
bool Flag_LBS_need = false;
int cell_number_scanned(char *source)
{
	char cell_num[3];
	filter_comma(source, 2, 3, cell_num, ',');
	return atoi(cell_num);
}
void conver_cells_json(char *source, char *json_cells)
{
	memset(json_cells, 0, strlen(json_cells));
	int32_t RF_ch, rxl, bsic, cellID, mcc, mnc, lac;
	int start = 0, end = 0;
	char cell[200];
	json_cells[0] = '[';
	for(int i = 0; i < strlen(source); i++)
	{
		if (source[i] == '"' && source[i-1] == ',')
		{
			start = i + 1;
		}
		if(source[i] == '"' && source[i+1] == '\r' && source[i+2] == '\n')
		{
			end = i;
			strncpy(cell, source + start, end - start);
			sscanf(cell, "%d,%d,%d,%x,%d,%d,%x", &RF_ch, &rxl, &bsic, &cellID, &mcc, &mnc, &lac);
			rxl = -111+rxl;
			sprintf(json_cells+strlen(json_cells), "{\"C\":%d,\"ID\":%d,\"L\":%d,\"N\":%d,\"S\":%d},", mcc, cellID, lac, mnc, rxl);
		}
	}
	json_cells[strlen(json_cells)-1] = ']';
}
void CENG_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)// NÃ¡ÂºÂ¿u Ã„â€˜ÃƒÂ£ scan Ã„â€˜Ã†Â°Ã¡Â»Â£c network
	{
		//ESP_LOGW(TAG, "CENG OK\r\n");
		Flag_Wait_Exit = true;
		strcpy(CENG_buf, ResponseBuffer);
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_Wait_Exit = true;
		//ESP_LOGW(TAG, "LBS timeout\r\n");
	}
	else if(event == EVEN_ERROR)
	{
		Flag_Wait_Exit = true;
		//ESP_LOGW(TAG, "LBS error\r\n");
	}
}
void Cell_get_infor()
{
	int retry = 0;
	IN:
	ATC_SendATCommandOptimize("AT+CENG=1,1\r\n", "AT+CENG=1,1", 1000, 2, ATResponse_Callback);

	CENG:
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CENG?\r\n", "+CENG: 1,1", 1000, 2, CENG_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		SoftReboot7070G();
		goto IN;
	}
	else
	{
		if(cell_number_scanned(CENG_buf) < 5 && retry++ < 10)
		{
			printf("cell_scanned: %d   retry: %d\r\n", cell_number_scanned(CENG_buf), retry);
			vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
			goto CENG;
		}
		if(cell_number_scanned(CENG_buf))
		{
			conver_cells_json(CENG_buf, cell_buffer);
			printf("cell_buffer: %s\r\n", cell_buffer);
		}
		else
		{
			printf("--------------\r\n");
			Flag_LBS_need = false;
		}
		return;
	}
}
void LBS_backup_thread()
{
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 0, SetCFUN_1_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);

	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		Hard_reset7070G();
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 0, SetCFUN_1_Callback);
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			Flag_LBS_need = false;
		}
	}
	//ESP_LOGW(TAG, "Scan GSM network\r\n");
	Flag_Wait_Exit = false;
	SelectNB_GSM_Network(GSM);
	WaitandExitLoop(&Flag_Wait_Exit);
	Cell_get_infor(CENG_buf, cell_buffer);
}
//--------------------------------------------------------------------------------------------------------------// Check accuracy
void Check_accuracy()
{
	Flag_need_check_accuracy = false;
	// AP_count < 0 or get accurancy > 60 switch to gps scan
	//	VTAG_Configure.Accuracy = 70;
	if(VTAG_Configure.Accuracy > 60)
	{
		GPS_Scan_Number = 45;
		GPS_Operation_Thread();
		if(GPS_Fix_Status != 1)
		{
			if(VTAG_Configure._lc == 1)
			{
				Flag_LBS_need = true;
				VTAG_MessType_G = LOCATION;
				MQTT_SendMessage_Thread(LOCATION);
				ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
			}
		}
		else
		{
			VTAG_MessType_G = LOCATION;
			MQTT_SendMessage_Thread(LOCATION);
			ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;

			ESP_sleep(1);
		}
	}
}
//--------------------------------------------------------------------------------------------------------------// test AT command
void CheckAtCommand(void)
{
	//ESP_LOGW(TAG, "Power on Sim7070G\r\n");
	gpio_set_level(UART_SW, 0);
	gpio_set_level(DTR_Sim7070_3V3, 0);
	//			TurnOn7070G_DTR();
	CHECK_AT:
	// Check AT response
	//ESP_LOGE(TAG, "Start program\r\n");
	ATC_SendATCommandOptimize("AT\r\n", "OK", 1000, 1, ATResponse_Callback);
	if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
	{
		Flag_Cycle_Completed = true;
		TurnOn7070G();
		vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
		goto CHECK_AT;
	}
	// Get battery percent
	//Check_battery();
	ATC_SendATCommandOptimize("AT+CSCLK=0\r\n", "OK", 1000, 2, ATResponse_Callback);
	if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
	{
		Flag_Cycle_Completed = true;
		TurnOn7070G();
		vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
		goto CHECK_AT;
	}
}
//--------------------------------------------------------------------------------------------------------------// Check battery
void Check_battery(void)
{
#define LEVEL 	0
#define INDEX_BAT_ARR 10
	// Láº¥y máº«u 5 láº§n AT+CBC
	uint16_t Bat_temp_arr[5] = {0};
	uint8_t  Bat_arr_index = 0;
	uint16_t Bat_sum = 0;
	BAT_READ:
	if(Flag_FullBattery == true || gpio_get_level(CHARGE) == 0 || Flag_charged == true)
	{
		if(Flag_FullBattery == true)
		{
			Bat_raw_pre = 100;
		}
		Fifo_bat_index = 0;
		memset(Fifo_bat_arr, 0, sizeof(Fifo_bat_arr));
	}
	while(Bat_arr_index < 5)
	{
		Bat_GetPercent();
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
			{
				Bat_arr_index = 0;
				memset(Bat_temp_arr, 0, sizeof(Bat_temp_arr));
				Hard_reset7070G();
				goto BAT_READ;
			}
			else
			{
				return;
			}
		}
		Bat_temp_arr[Bat_arr_index++] = VTAG_DeviceParameter.Bat_Voltage;
		vTaskDelay(50 / RTOS_TICK_PERIOD_MS);
	}
	//Láº¥y giÃ¡ trá»‹ max cá»§a 5 láº§n Ä‘á»�c
	VTAG_DeviceParameter.Bat_Voltage = Bat_temp_arr[0];
	for(int i = 1; i < 5; i++)
	{
		if(Bat_temp_arr[i] > VTAG_DeviceParameter.Bat_Voltage)
		{
			VTAG_DeviceParameter.Bat_Voltage = Bat_temp_arr[i];
		}
	}
	//Náº¿u buffer fifo trÃ n thÃ¬ bá»� pháº§n tá»­ cÅ© nháº¥t
	if(Fifo_bat_index >= INDEX_BAT_ARR)
	{

		Fifo_bat_index = Fifo_bat_index - 1;
		for(int i = 0; i < Fifo_bat_index; i++)
		{
			Fifo_bat_arr[i] = Fifo_bat_arr[i+1];
		}
	}
	//Ghi giÃ¡ trá»‹ má»›i vÃ o máº£ng
	Fifo_bat_arr[Fifo_bat_index] = VTAG_DeviceParameter.Bat_Voltage;
	Fifo_bat_index++;
	for(int i = 0; i < Fifo_bat_index; i++)
	{
		//ESP_LOGI(TAG,"Fifo_bat_arr[%d]: %d\r\n", i, Fifo_bat_arr[i]);
		Bat_sum = Fifo_bat_arr[i] + Bat_sum;
	}
	//ESP_LOGI(TAG,"Bat_sum: %d\r\n", Bat_sum);
	VTAG_DeviceParameter.Bat_Voltage = ceil((float)Bat_sum / Fifo_bat_index);
	Bat_Process_();
	//ESP_LOGI(TAG,"VTAG_DeviceParameter.Bat_Level: %d\r\n",  VTAG_DeviceParameter.Bat_Level);
}
//--------------------------------------------------------------------------------------------------------------//
void RTC_IRAM_ATTR adc1_get_raw_ram(adc1_channel_t channel)
{
	SENS.sar_read_ctrl.sar1_dig_force = 0; // switch SARADC into RTC channel
	SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU; // adc_power_on
	RTCIO.hall_sens.xpd_hall = false; //disable other peripherals
	SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD; // channel is set in the convert function

	// disable FSM, it's only used by the LNA.
	SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
	SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
	SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
	SENS.sar_meas_wait1.sar_amp_wait1 = 1;
	SENS.sar_meas_wait1.sar_amp_wait2 = 1;
	SENS.sar_meas_wait2.sar_amp_wait3 = 1;

	//set controller
	SENS.sar_read_ctrl.sar1_dig_force = false;      //RTC controller controls the ADC, not digital controller
	SENS.sar_meas_start1.meas1_start_force = true;  //RTC controller controls the ADC,not ulp coprocessor
	SENS.sar_meas_start1.sar1_en_pad_force = true;  //RTC controller controls the data port, not ulp coprocessor
	SENS.sar_touch_ctrl1.xpd_hall_force = true;     // RTC controller controls the hall sensor power,not ulp coprocessor
	SENS.sar_touch_ctrl1.hall_phase_force = true;   // RTC controller controls the hall sensor phase,not ulp coprocessor

	//start conversion
	SENS.sar_meas_start1.sar1_en_pad = (1 << channel); //only one channel is selected.
	while (SENS.sar_slave_addr1.meas_status != 0);
	SENS.sar_meas_start1.meas1_start_sar = 0;
	SENS.sar_meas_start1.meas1_start_sar = 1;
	while (SENS.sar_meas_start1.meas1_done_sar == 0);
	RTC_ADC = SENS.sar_meas_start1.meas1_data_sar; // set adc value!

	SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PD; // adc power off
}
void RTC_IRAM_ATTR adc2_get_raw_ram(adc2_channel_t channel)
{
	SENS.sar_read_ctrl2.sar2_dig_force = 0; // switch SARADC into RTC channel
	SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU; // adc_power_on
	RTCIO.hall_sens.xpd_hall = false; //disable other peripherals
	SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD; // channel is set in the convert function

	// disable FSM, it's only used by the LNA.
	SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
	SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
	SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
	SENS.sar_meas_wait1.sar_amp_wait1 = 1;
	SENS.sar_meas_wait1.sar_amp_wait2 = 1;
	SENS.sar_meas_wait2.sar_amp_wait3 = 1;

	//set controller
	SENS.sar_read_ctrl2.sar2_dig_force = false;      //RTC controller controls the ADC, not digital controller
	SENS.sar_meas_start2.meas2_start_force = true;  //RTC controller controls the ADC,not ulp coprocessor
	SENS.sar_meas_start2.sar2_en_pad_force = true;  //RTC controller controls the data port, not ulp coprocessor
	//SENS.sar_touch_ctrl1.xpd_hall_force = true;     // RTC controller controls the hall sensor power,not ulp coprocessor
	//SENS.sar_touch_ctrl1.hall_phase_force = true;   // RTC controller controls the hall sensor phase,not ulp coprocessor

	//start conversion
	SENS.sar_meas_start2.sar2_en_pad = (1 << channel); //only one channel is selected.
	while (SENS.sar_slave_addr1.meas_status != 0);
	SENS.sar_meas_start2.meas2_start_sar = 0;
	SENS.sar_meas_start2.meas2_start_sar = 1;
	while (SENS.sar_meas_start2.meas2_done_sar == 0);
	RTC_ADC = SENS.sar_meas_start2.meas2_data_sar; // set adc value!

	SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PD; // adc power off
}
void RTC_IRAM_ATTR wake_stub(void)
{
	REG_WRITE(GPIO_ENABLE_REG, BIT5);
	REG_WRITE(GPIO_ENABLE_REG, BIT15);

	if(((REG_READ(GPIO_IN_REG) >> (CHARGE - 32)) & 0x1) == 0)
	{
		REG_WRITE(GPIO_OUT_W1TS_REG, BIT5);
		wake_count++;
	}
	else
	{
		REG_WRITE(GPIO_OUT_W1TS_REG, BIT15);
	}
	adc2_get_raw_ram(0);
	if(wake_count >= 1000) wake_count = 0;

}
static void init_ulp_program(void)
{
	esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
			(ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
	ESP_ERROR_CHECK(err);


	Bat_ADC_Init();

	REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE);
	REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE);
	REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);
	REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_PWDET_FORCE);
	REG_SET_BIT(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_MUX);


	/* Init GPIO15 (RTC_GPIO 13, green) which will be driven by ULP */
	rtc_gpio_init(15);
	rtc_gpio_set_direction(15, RTC_GPIO_MODE_OUTPUT_ONLY);
	rtc_gpio_set_level(15,0);

	/* Init GPIO35 (RTC_GPIO 5, green) which will be driven by ULP */
	rtc_gpio_init(CHARGE);
	rtc_gpio_set_direction(CHARGE, RTC_GPIO_MODE_INPUT_ONLY);

	esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}
//------------------------------------------------------------------------------------------------------------------------//
void charge_state_setup()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.pin_bit_mask = (1ULL << CHARGE);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);
	//change gpio intrrupt type for one pin
	gpio_set_intr_type(CHARGE, GPIO_INTR_ANYEDGE);
}

float read_battery()
{
	int raw = 0;
	uint32_t adc_reading = 0;
	Bat_ADC_Init();
	for(int i = 0; i < NO_OF_SAMPLES; i++)
	{
		adc2_get_raw((adc2_channel_t)Bat_Channel, width, &raw);
		adc_reading += raw;
	}
	adc_reading /= NO_OF_SAMPLES;
	float mv_voltage = ((2.085924*3300.f*adc_reading)/4096.0f);
	return mv_voltage * Calib_adc_factor;
}

static void charging_task(void *arg)
{
	bool FLag_wifi_scanning_status = false;
	while(1)
	{
		if(Flag_wifi_scanning == true)
		{
			FLag_wifi_scanning_status = true;
		}
		if(state == IN_CHARGING && Flag_wifi_scanning == false)
		{
			Flag_charged = true;
			if(FLag_wifi_scanning_status == true)
			{
				Flag_wifi_scanning = false;
				FLag_wifi_scanning_status = false;
				vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
			}
			// Need to check whether wifi process is available
			batt_vol_prev = read_battery();
			//ESP_LOGW(TAG,"VBAT prev =  %.1f mV\r\n",batt_vol_prev);
			charging_time++;
			//ESP_LOGE(TAG,"charging_time: %d\r\n", charging_time);
		}
		vTaskDelay(5000/RTOS_TICK_PERIOD_MS);
	}
}
//--------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------//
void main_task(void *arg);
void uart_rx_task(void *arg);
void check_timeout_task(void *arg);
void gps_scan_task(void *arg);
void gps_processing_task(void *arg);
void mqtt_submessage_processing_task(void *arg);
void led_indicator(void* arg);
void Test_task(void* arg);
void fota_ack(void* arg);
void test_main_task(void *arg);
void Fota2G_processing_task(void *arg);
//--------------------------------------------------------------------------------------------------------------//
void Create_Tasks(void)
{
	xTaskCreatePinnedToCore(test_main_task, "test_main_task", 4096*2, NULL, MAIN_TASK_PRIO, &test_main_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(check_timeout_task, "check_timeout", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &check_timeout_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(gps_scan_task, "gps scan", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &gps_scan_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(gps_processing_task, "gps processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &gps_processing_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(mqtt_submessage_processing_task, "mqtt submessage processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &mqtt_submessage_processing_handle, tskNO_AFFINITY);
}

void Delete_Tasks(void)
{
	acc_power_down();
	vTaskDelete(main_task_handle);
	vTaskDelete(check_timeout_task_handle);
	vTaskDelete(gps_scan_task_handle);
	vTaskDelete(gps_processing_task_handle);
	vTaskDelete(mqtt_submessage_processing_handle);
}

void PreOperationInit(void)
{
	//	Flag_Wait_Exit = false;
	//	ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 1000, 2, ATResponse_Callback);
	//	WaitandExitLoop(&Flag_Wait_Exit);

	//	vTaskDelay(200/RTOS_TICK_PERIOD_MS);

	ATC_SendATCommandOptimize("AT+CIMI\r\n", "OK", 1000, 2, ATResponse_Callback);

	// Set clock
	ATC_SendATCommandOptimize("AT+CSCLK=0\r\n", "OK", 3000, 3, ATResponse_Callback);

	//
	ATC_SendATCommandOptimize("AT+CMNB=3\r\n", "OK", 1000, 3, ATResponse_Callback);

	//Fix band for nb-iot
	ATC_SendATCommandOptimize("AT+CBANDCFG=\"NB-IOT\",3\r\n", "OK", 1000, 3, ATResponse_Callback);
	//Select optimizeation for band scan

	ATC_SendATCommandOptimize("AT+CNBS=2\r\n", "OK", 1000, 3, ATResponse_Callback);
	//
	ATC_SendATCommandOptimize("AT+CBAND=\"ALL_MODE\"\r\n", "OK", 1000, 2, ATResponse_Callback);

	//	Flag_Wait_Exit = false;
	//	ATC_SendATCommand("AT+CFUN=0\r\n", "OK", 1000, 2, ATResponse_Callback);
	//	WaitandExitLoop(&Flag_Wait_Exit);
}
void IncreaseMessBackup(void)
{
	if(Backup_Array_Counter >= BU_Arr_Max_Num)
	{
		Backup_Array_Counter = BU_Arr_Max_Num - 1;
		for(int i = 0; i < BU_Arr_Max_Num - 1; i++)
		{
			strcpy(Location_Backup_Array[i], Location_Backup_Array[i+1]);
		}
	}
	strcpy(Location_Backup_Array[Backup_Array_Counter], Mqtt_TX_Str);
	Backup_Array_Counter++;
}
void BackUp_UnsentMessage(VTAG_MessageType Mess_Type)
{
	if(Mess_Type == FOTA_START || Mess_Type == FOTA_FAIL||Mess_Type == FOTA_SUCCESS)
	{
		return;
	}
	Fail_network_counter++;
	Flag_count_nb_backup++;
	Flag_reboot_7070 = false;
	if(Fail_network_counter > 6)
	{
		Fail_network_counter = 0;
		Flag_reboot_7070 = true;
	}
	if(Flag_count_nb_backup > 1)
	{
		Flag_count_nb_backup = 0;
		Flag_use_2g = true;
	}
	//ESP_LOGW(TAG, "Backup data before going to sleep\r\n");
	if(Flag_Fota == true) return;
	if(Flag_backup_data == true && Flag_sending_backup == false)
	{
		Flag_backup_data = false;
		IncreaseMessBackup();
	}
	else
	{
		// Convert MQTT location payload
		if(Mess_Type == LOCATION)
		{
			//ESP_LOGI(TAG, "Back up location");
			MQTT_Location_Payload_Convert();
		}
		else if(Mess_Type == GPS_FAIL)
		{
			//			//ESP_LOGI(TAG, "Back up DPOF");
			GetDeviceTimestamp();
			if(Network_Type == GSM)
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DPOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			else
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DPOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
		}
		else if(Mess_Type == STARTUP)
		{
			//ESP_LOGI(TAG, "Back up DON");
			GetDeviceTimestamp();
			if(Network_Type == GSM)
			{
				MQTT_DonDevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DON", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA,loopString);
			}
			else
			{
				MQTT_DonDevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DON", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA, loopString);
			}
		}
		else if(Mess_Type == LOWBATTERY)
		{
			//			//ESP_LOGI(TAG, "Back up DBL");
			GetDeviceTimestamp();
			if(Network_Type == GSM)
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DBL", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			else
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DBL", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
		}
		else if(Mess_Type == FULLBATTERY)
		{
			//			//ESP_LOGI(TAG, "Back up DBF");
			GetDeviceTimestamp();
			if(Network_Type == GSM)
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DBF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, 0, VTAG_Vesion, 100, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			else
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DBF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, 0, VTAG_Vesion,100, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
		}
		else if(Mess_Type == DCF_ACK && Flag_config_sms)
		{
			//			//ESP_LOGI(TAG, "Back up DCF when receive sms");
			GetDeviceTimestamp();
			Flag_config_sms = false;
			if(Network_Type == GSM)
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DCF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			else
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DCF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
		}
		if(strlen(Mqtt_TX_Str) && Mess_Type != SEND_BACKUP)
		{
			IncreaseMessBackup();
		}
		//Check acc time out
		if(Flag_motion_detected == true && Flag_send_DAST == false)
		{
			acc_counter = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - acc_capture, esp_clk_slowclk_cal_get())/1000000);
			//ESP_LOGI(TAG, "acc_counter: %d s",(int)acc_counter);
			if(acc_counter > 180)
			{
				acc_counter = 0;
				Flag_motion_detected = false;
				flag_end_motion = true;
				Flag_send_DASP = true;
				Flag_StopMotion_led = true;
				if(VTAG_Configure._SS == 0){
					Flag_send_DASP = false;
				}
			}
		}

		BACK_UP:
		if(Flag_DBL_Task == true)
		{
			if(VTAG_DeviceParameter.Bat_Level <= 15)
			{
				GetDeviceTimestamp();
				Flag_DBL_Task = false;
				GetDeviceTimestamp();
				//				//ESP_LOGW(TAG,"Send DBL message\r\n");
				if(Network_Type == GSM)
				{
					MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DBL", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Device_shutdown_ts, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
				}
				else
				{
					MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DBL", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Device_shutdown_ts, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
				}
				IncreaseMessBackup();
			}
		}

		if(Flag_FullBattery == true)
		{
			GetDeviceTimestamp();
			Flag_FullBattery = false;
			GetDeviceTimestamp();
			Bat_raw_pre = 100;
			Check_battery();
			//			//ESP_LOGE(TAG, "Send DBF message\r\n");
			if(Network_Type == GSM)
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DBF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, 100, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			else
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DBF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, 100, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			IncreaseMessBackup();
		}
		if(strstr(VTAG_Configure.Type, "O"))
		{
			GetDeviceTimestamp();
			Flag_shutdown_dev = false;
			if(Network_Type == GSM)
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			else
			{
				MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
			}
			IncreaseMessBackup();
		}
		//Check sos flag if true then backup
		if(Flag_sos == true)
		{
			if(Flag_wifi_init == false)
			{
				wifi_init();
			}
			wifi_scan();
			GetDeviceTimestamp();
			MQTT_Location_Payload_Convert();
			IncreaseMessBackup();
			Flag_sos = false;
		}
		//Check DASP if true then backup
		if(Flag_send_DASP == true)
		{
			if(Flag_wifi_init == false)
			{
				wifi_init();
			}
			wifi_scan();
			GetDeviceTimestamp();
			MQTT_Location_Payload_Convert();
			if(Backup_Array_Counter >= BU_Arr_Max_Num)
			{
				Backup_Array_Counter = BU_Arr_Max_Num - 1;
				for(int i = 0; i < (BU_Arr_Max_Num - 1); i++)
				{
					strcpy(Location_Backup_Array[i], Location_Backup_Array[i+1]);
				}
			}
			strcpy(Location_Backup_Array[Backup_Array_Counter], Mqtt_TX_Str);
			Backup_Array_Counter++;
			Flag_send_DASP = false;
		}
		//Check DAST if true then backup
		if(Flag_send_DAST == true)
		{
			if(Flag_wifi_init == false)
			{
				wifi_init();
			}
			wifi_scan();
			GetDeviceTimestamp();
			MQTT_Location_Payload_Convert();
			IncreaseMessBackup();
			Flag_send_DAST = false;
		}
	}
	if(Flag_Unpair_Task == true)
	{
		Flag_Unpair_Task = false;
	}
	if(Flag_location_send_2 == true)
	{
		if(ap_count >= 3 && ap_count < 5 && GPS_Fix_Status!= 1 && Flag_LBS_need == false)
		{
			AT_RX_event = EVEN_OK;
			GPS_Scan_Number = 45;
			GPS_Operation_Thread();
			if(GPS_Fix_Status != 1)
			{
				if(VTAG_Configure._lc == 1)
				{
					Flag_LBS_need = true;
					LBS_backup_thread();
					if(Flag_LBS_need == true)
					{
						IncreaseMessBackup();
					}
				}
			}
			else
			{
				Flag_LBS_need = false;
				MQTT_Location_Payload_Convert();
				IncreaseMessBackup();
			}
		}
	}
	else if(Flag_location_send_1 == true)
	{
		if (ap_count >= 3 && ap_count < 5 && GPS_Fix_Status!= 1  && Flag_LBS_need == false)
		{
			if(VTAG_Configure._lc == 1)
			{
				Flag_LBS_need = true;
				LBS_backup_thread();
				if(Flag_LBS_need == true)
				{
					MQTT_Location_Payload_Convert();
					IncreaseMessBackup();
				}
			}
		}
	}
	//ESP_LOGW(TAG, "Backup data list:\r\n");
	for(int i = 0; i < Backup_Array_Counter; i++)
	{
		if(strstr(Location_Backup_Array[i],"re"))
		{
			for(int j = 0; j < strlen(Location_Backup_Array[i]); j++)
			{
				if(Location_Backup_Array[i][j] == 'r' && Location_Backup_Array[i][j+1] == 'e' && Location_Backup_Array[i][j-1] == '"' && Location_Backup_Array[i][j+2] == '"')
				{
					Location_Backup_Array[i][j-2] = '}';
					Location_Backup_Array[i][j-1] = 0;
					break;
				}
			}
		}
		//ESP_LOGW(TAG, "Data %d: %s\r\n", i, Location_Backup_Array[i]);
	}
	ESP_sleep(1);
	//	Flag_config = true;
	if(!Flag_sms_receive)
	{
		goto BACK_UP;
		Flag_Unpair_Task = false;
		Flag_sos = false;
	}
}

//--------------------------------------------------------------------------------------------------------------// GPIO Input (Button) processing function
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
//--------------------------------------------------------------------------------------------------------------//
static void button_processing_task(void* arg)
{
	CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
	CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);
	if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 )
	{
		uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
		if (wakeup_pin_mask != 0)
		{
			int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
			if(pin == BUTTON)
			{
				uint32_t ulVar = 25UL;
				xQueueSend(gpio_evt_queue,  ( void * ) &ulVar, ( TickType_t ) 10 );
			}
			printf("Wake up from GPIO %d\n", pin);
		} else {
			printf("Wake up from GPIO\n");
		}
	}
	while(1)
	{
		// Counter for check button pressed duration
		if(Flag_Button_Duration_Start == true)
		{
			Button_Pressed_Duration++;
			//Flag_wake = false;
		}
		// Counter for charge delay
		if(Flag_Charge_Delay_Start == true)
		{
			Charge_Delay_Counter++;
		}
		// Delay for 3 second
		if(Charge_Delay_Counter >= 300 && ((charging_time > 6 && Flag_charge_in_wake == true) || Flag_charge_in_wake == false))
		{
			//ESP_LOGE(TAG,"STOP\r\n");
			Charge_Delay_Counter = 0;
			charging_time = 0;
			Flag_Charge_Delay_Start = false;
			//			vTaskDelay(3000 / RTOS_TICK_PERIOD_MS);
			float batt_vol_now = read_battery();
			//ESP_LOGW(TAG,"VBAT now =  %.1f mV\r\n",batt_vol_now);

			// If the battery voltage reach to a threshold then it is fully charged
			if(batt_vol_now >= THRESHOLD)
			{
				state = FULL_BATTERY;
				Flag_FullBattery = true;
				printf("CASE 2:BATTERY IS FULL\r\n");
			}
			// If the battery voltage not reach to a threshold then using other approach to check
			else
			{
				if(batt_vol_prev >= (batt_vol_now + VOFF_SET))
				{
					state = DISRUPTED;
					Flag_FullBattery = false;
					printf("CASE 3:DISRUPTED\r\n");
				}
				else
				{
					if(batt_vol_now > batt_vol_prev)
					{
						state = UNDEFINED;
						Flag_FullBattery = false;
						printf("CASE 4:UNDEFINED\r\n");
					}
					else
					{
						state = FULL_BATTERY;
						Flag_FullBattery = true;
						printf("CASE 2:BATTERY IS FULL\r\n");
					}
				}
			}
		}
		// Counter for determining end pressing button --> number of press
		if(Flag_Button_EndCheck_Start == true)
		{
			Button_EndCheck_Counter++;
			if(Button_EndCheck_Counter >= 100)
			{
				Button_EndCheck_Counter = 0;
				Flag_button_EndCheck = true;
				Flag_Button_EndCheck_Start = false;
				//ESP_LOGE(TAG,"Button pressed counter: %d\r\n", Button_Pressed_Counter);
				button_led_indcator = Button_Pressed_Counter;
				Flag_button_cycle_start = false;
				//printf("Flag_button_cycle_start false %d\r\n",Flag_button_cycle_start);
				switch (Button_Pressed_Counter)
				{
				case B_SHUTDOWN:
					ESP_LOGW(TAG,"Shutdown device task\r\n");
					LED_TurnOff();
					GetDeviceTimestamp();
					sprintf(Device_shutdown_ts,"%lld", VTAG_DeviceParameter.Device_Timestamp);
					writetofile(base_path, "vtag_off_ts.txt", Device_shutdown_ts);
					gpio_hold_dis(ESP32_PowerLatch);
					gpio_set_level(ESP32_PowerLatch, 0);	// For power latch
					vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
					break;
				case B_RESTART:
					//					loopbutton_reason = BT_RESTART;
					//					saveLoopReason();
					Flag_new_firmware_led = true;
					break;
				case B_UNPAIR:
					//					loopbutton_reason = BT_UNPAIR;
					//					saveLoopReason();
					if(strchr(Device_PairStatus,'P'))
					{
						ESP_LOGW(TAG,"Unpair device task\r\n");
						//								gpio_set_level(UART_SW, 1);
						Flag_Unpair_Task = true;
						Flag_Unpair_led =  true;
					}
					break;
				case B_FOTA:
					loopbutton_reason = BT_FOTA;
					saveLoopReason();
					if(Flag_wifi_scanning == false && Flag_Fota == false)
					{
						//ESP_LOGW(TAG,"FOTA device task\r\n");
						acc_power_down();
						//							i2c_driver_delete(I2C_MASTER_NUM);
						rtc_wdt_protect_off();
						rtc_wdt_disable();
						memset(&SIMCOM_ATCommand, 0, sizeof(SIMCOM_ATCommand));
						ESP32_Clock_Config(80, 80, false);
						CHECK_ERROR_CODE(esp_task_wdt_init(180, false), ESP_OK);
						//ESP_LOGW(TAG,"FOTA device task1\r\n");
						if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
						{
							vTaskDelete(check_motion_handle);
						}
						vTaskDelete(main_task_handle);
						vTaskDelete(check_timeout_task_handle);
						vTaskDelete(gps_scan_task_handle);
						vTaskDelete(gps_processing_task_handle);
						vTaskDelete(mqtt_submessage_processing_handle);
						if(Flag_wifi_init == false)
						{
							init_OTA_component();
						}
						ini_wifi();
						ESP_ERROR_CHECK(esp_wifi_start());
						Flag_Unpair_led = false;
						Flag_Fota_led = true;
						Flag_Fota = true;
					}
					break;
				case B_UNPAIR_TEST:
					if(strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0)
					{
						Flag_test_unpair = true;
						memset(&SIMCOM_ATCommand, 0, sizeof(SIMCOM_ATCommand));
						Delete_Tasks();
						Create_Tasks();
					}
					break;
				case 0:
					//					loopbutton_reason = BT_NOTHING;
					//					saveLoopReason();
					if(Button_Long_Pressed_counter == 0)
					{
						// Addded: Ext1_Wakeup_Pin != CHARGE condition
						if(Ext1_Wakeup_Pin != CHARGE && Flag_sos == false && Flag_send_DASP == false && Flag_send_DAST == false && ACK_Succeed[WAIT_ACK] == 0 && ACK_Succeed[ACK_DONE] == 0 && Flag_mainthread_run == false)
						{
							Flag_button_do_nothing = true;
							ESP_sleep(0);
						}
					}
					break;
				case 1:
					//						gpio_set_level(UART_SW, 0);
					// Addded: Ext1_Wakeup_Pin != CHARGE condition
					if(((esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && Flag_motion_detected == false) && Flag_mainthread_run == false && Ext1_Wakeup_Pin != CHARGE) || (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && Ext1_Wakeup_Pin != ACC_INT && Flag_motion_detected == true && Flag_sos == false))
					{
						//						loopbutton_reason = BT_ONE;
						//						saveLoopReason();
						Flag_button_do_nothing = true;
						//ESP_LOGW(TAG,"Press button: 1\r\n");
						ESP_sleep(0);
					}
					break;
				default:
					if(((esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && Flag_motion_detected == false) && Flag_mainthread_run == false && Ext1_Wakeup_Pin != CHARGE) || (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && Ext1_Wakeup_Pin != ACC_INT && Flag_motion_detected == true && Flag_sos == false))
					{
						//						loopbutton_reason = BT_DEFAULT;
						//						saveLoopReason();
						Flag_button_do_nothing = true;
//						ESP_LOGW(TAG,"Press button: default\r\n");
						ESP_sleep(0);
					}
					break;
				}
				Button_Pressed_Counter = 0;
			}
		}
		CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
		vTaskDelay(10 / RTOS_TICK_PERIOD_MS);
	}
}
//--------------------------------------------------------------------------------------------------------------//
static void gpio_task_example(void* arg)
{
	CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
	bool Flag_wake = false;
	if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 )
	{
		uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
		if (wakeup_pin_mask != 0)
		{
			int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
			if(pin == BUTTON)
			{
				Flag_wake = true;
			}
		}
	}
	uint32_t io_num;
	while(1)
	{
		if(xQueueReceive(gpio_evt_queue, &io_num, 1000/RTOS_TICK_PERIOD_MS))
		{
			// Charge GPIO interrupt
			if(io_num == CHARGE)
			{
				// Flash LED for charging indication
				if(gpio_get_level(CHARGE) == 0)
				{
					Flag_charged = true;
					Flag_charge_in_wake = true;
					charging_time = 0;
					//ESP_LOGE(TAG,"charging_time: %d\r\n", charging_time);
					Flag_Charge_Delay_Start = false;
					Charge_Delay_Counter = 0;
					state = IN_CHARGING;
					Flag_FullBattery = false;
					//ESP_LOGE(TAG,"Charging\r\n");
					// Init for ULP
					init_ulp_program();
					/* Start the ULP program */
					esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
					ESP_ERROR_CHECK(err);
				}
				// Full battery or charge disrupt
				else
				{
					if(state == IN_CHARGING)
					{
						// Re-init GPIO for LED indicator after disrupting charger
						ESP32_GPIO_Output_Config();
						state = STOP_CHARGING;
						Flag_Charge_Delay_Start = true;
						//ESP_LOGE(TAG,"START\r\n");
						//						vTaskDelay(3000 / RTOS_TICK_PERIOD_MS);
						//						float batt_vol_now = read_battery();
						//						//ESP_LOGW(TAG,"VBAT now =  %.1f mV\r\n",batt_vol_now);
						//
						//						// If the battery voltage reach to a threshold then it is fully charged
						//						if(batt_vol_now >= 4150)
						//						{
						//							state = FULL_BATTERY;
						//							Flag_FullBattery = true;
						//							printf("CASE 2:BATTERY IS FULL\r\n");
						//						}
						//						// If the battery voltage not reach to a threshold then using other approach to check
						//						else
						//						{
						//							if(batt_vol_prev >= (batt_vol_now + 30))
						//							{
						//								state = DISRUPTED;
						//								Flag_FullBattery = false;
						//								printf("CASE 3:DISRUPTED\r\n");
						//							}
						//							else
						//							{
						//								if(batt_vol_now > batt_vol_prev)
						//								{
						//									state = UNDEFINED;
						//									Flag_FullBattery = false;
						//									printf("CASE 4:UNDEFINED\r\n");
						//								}
						//								else
						//								{
						//									state = FULL_BATTERY;
						//									Flag_FullBattery = true;
						//									printf("CASE 2:BATTERY IS FULL\r\n");
						//								}
						//							}
						//						}
					}
				}
			}
			if(io_num == BUTTON)// GPIO 25
			{
				vTaskDelay(10 / RTOS_TICK_PERIOD_MS);
				Flag_button_cycle_start = true;
				//        		printf("Flag_button_cycle_start true %d\r\n",Flag_button_cycle_start);
				Button_EndCheck_Counter = 0;
				uint8_t io_state = gpio_get_level(io_num);
				if(io_state == 1) // if the interrupt is on falling edge
				{
					if(gpio_get_level(CHARGE))
					{
						gpio_set_level(LED_2, 1);
					}
					else if(gpio_get_level(CHARGE) == 0)
					{
						gpio_set_level(LED_1, 1);
					}
					//vTaskDelay(50 / RTOS_TICK_PERIOD_MS);
					Flag_Button_Duration_Start = true;
				}
				else if(io_state == 0)// if the interrupt is on rising edge
				{
					if(gpio_get_level(CHARGE))
					{
						gpio_set_level(LED_2, 0);
					}
					else
					{
						gpio_set_level(LED_1, 0);
					}
					Flag_Button_Duration_Start = false;
					if(Flag_wake == true)
					{
						Flag_wake = false;
						if(Button_Pressed_Duration <= Button_ShortPressed)
						{
							Button_Pressed_Counter++;
							ESP_LOGE(TAG,"Short press 1\r\n");
							ESP_LOGW(TAG,"Button pressed duration: %d ms\r\n", Button_Pressed_Duration);
						}
					}
					if(Button_Pressed_Duration <= Button_ShortPressed &&  Button_Pressed_Duration >= Button_MinPressed)
					{
						Button_Pressed_Counter++;
						ESP_LOGE(TAG,"Short press 2\r\n");
						ESP_LOGW(TAG,"Button pressed duration: %d ms\r\n", Button_Pressed_Duration);
					}
					else if(Button_Pressed_Duration >= Button_LongPressed)
					{
						ESP_LOGE(TAG,"Long press: SOS\r\n");
						Button_Long_Pressed_counter = true;
						Flag_button_long_press_led = true;
//						ESP_LOGW(TAG,"Button pressed duration: %d ms\r\n", Button_Pressed_Duration);
						Flag_sos = true;
						//						loopbutton_reason = BT_SOS;
						//						saveLoopReason();
						//						ACK_Succeed[0] = 3;
						//						if(Flag_wifi_scan == false && Flag_Fota == false)
						//						{
						//							gpio_set_level(UART_SW, 0);
						//							ResetAllParameters();
						//							if(main_task_handle != NULL)
						//							{
						//								vTaskDelete(main_task_handle);
						//							}
						//							xTaskCreatePinnedToCore(main_task, "main", 4096*2, NULL, MAIN_TASK_PRIO, &main_task_handle, tskNO_AFFINITY);
						//						}
					}
					Flag_Button_EndCheck_Start = true;
					Button_Pressed_Duration = 0;
				}
			}
		}
		CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
		vTaskDelay(10 / RTOS_TICK_PERIOD_MS);
	}
}
//--------------------------------------------------------------------------------------------------------------//
void ESP32_GPIO_Output_Config(void)
{
	gpio_config_t io_conf;

	io_conf.pin_bit_mask = ESP32_GPIO_OUTPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&io_conf);
}
void ESP32_GPIO_Output_SetDefault(void)
{
	gpio_set_level(PowerLatch, 1);
	gpio_set_level(PowerKey, 0);
	gpio_set_level(VCC_7070_EN, 1);
	gpio_set_level(VCC_GPS_EN, 0);
	gpio_set_level(UART_SW, 0);
	gpio_set_level(DTR_Sim7070_3V3, 1);

	//gpio_set_level(LED_2, 1);

	vTaskDelay(100 / RTOS_TICK_PERIOD_MS);

	gpio_set_level(LED_2, 0);
	gpio_set_level(LED_1, 0);

	gpio_hold_en(PowerLatch);
	//	gpio_hold_en(DTR_Sim7070_3V3);
	gpio_hold_en(VCC_7070_EN);
	gpio_deep_sleep_hold_en();

	xTaskCreatePinnedToCore(led_indicator, "led_indicator", 4096, NULL, CHECKTIMEOUT_TASK_PRIO, &led_indicator_handle, tskNO_AFFINITY);
}
void ESP32_GPIO_Output_Init(void)
{
	ESP32_GPIO_Output_Config();
	ESP32_GPIO_Output_SetDefault();
}
void ESP32_GPIO_Input_Init(void)
{
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.pin_bit_mask = ESP32_GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	xTaskCreatePinnedToCore(gpio_task_example, "gpio_task_example", 4096, NULL, 6, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(button_processing_task, "button_processing_task", 4096, NULL, 6, NULL, tskNO_AFFINITY);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(BUTTON, gpio_isr_handler, (void*) BUTTON);
	gpio_isr_handler_add(CHARGE, gpio_isr_handler, (void*) CHARGE);
}
//--------------------------------------------------------------------------------------------------------------// ADC function
void Bat_ADC_Init(void)
{
	r = adc2_pad_get_io_num(Bat_Channel, &adc_gpio_num );
	adc2_config_channel_atten(Bat_Channel, ADC_ATTEN_DB_11 );
	adc2_get_raw(Bat_Channel, width, &read_raw);
}
//--------------------------------------------------------------------------------------------------------------// Configure clock function
void ESP32_Clock_Config(int Freq_Max, int Freq_Min, bool LighSleep_Select)
{
	esp_pm_config_esp32_t pm;
	pm.max_freq_mhz = Freq_Max;
	pm.min_freq_mhz = Freq_Min;
	pm.light_sleep_enable = LighSleep_Select;

	ESP_ERROR_CHECK(esp_pm_configure(&pm));

	while (esp_clk_cpu_freq() != Freq_Max * 1000000)
	{
		//ESP_LOGW(TAG, "CPU clock: %d\r\n", esp_clk_cpu_freq()/1000000);
		vTaskDelay(100 / RTOS_TICK_PERIOD_MS);
	}
	//ESP_LOGW(TAG, "CPU clock after reconfiguring: %d\r\n", esp_clk_cpu_freq()/1000000);
}
//--------------------------------------------------------------------------------------------------------------// SMS functions
//void Set7070ToSleepMode(void)
//{
//	//	// Check AT response
//	Flag_Wait_Exit = false;
//	ATC_SendATCommand("AT\r\n", "OK", 1000, 10, ATResponse_Callback);
//	WaitandExitLoop(&Flag_Wait_Exit);
//
//	// Code for sleep Sim7070G
//	Flag_Wait_Exit = false;
//
//	ATC_SendATCommand("AT+CSCLK=1\r\n", "OK", 3000, 3, ATResponse_Callback);
//	WaitandExitLoop(&Flag_Wait_Exit);
//	gpio_set_level(DTR_Sim7070_3V3, 1);
//	//ESP_LOGE(TAG, "Set 7070G to sleep mode\r\n");
//	gpio_set_level(18, 1);
//
//	//	Flag_Cycle_Completed = false;
//	//	//ESP_LOGE(TAG, "Enter to deep sleep mode\r\n");
//	//	esp_deep_sleep_start();
//}
//--------------------------------------------------------------------------------------------------------------//
//static void SMS_Operation_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
//{
//	AT_RX_event = event;
//	Flag_Wait_Exit = true;
//	if(event == EVEN_OK)
//	{
//		switch (SMS_Send_Step)
//		{
//		case 0:
//			//ESP_LOGW(TAG, "SMS ready\r\n");
//			SMS_Process_Configure(DirectPassSMS);
//			break;
//		case 1:
//			if(SMS_Receive_Method == BufferedSMS)
//			{
//				//ESP_LOGW(TAG,"Set SMS to buffer OK\r\n");
//			}else if(SMS_Receive_Method == DirectPassSMS)
//			{
//				//ESP_LOGW(TAG,"Set SMS direct pass OK\r\n");
//			}
//			break;
//		case 2:
//			SMS_Send_Step = 2;
//			break;
//		default:
//			break;
//		}
//		SMS_Send_Step++;
//	}
//	else if(event == EVEN_TIMEOUT)
//	{
//		//ESP_LOGW(TAG,"SMS operation time out\r\n");
//	}
//	else if(event == EVEN_ERROR)
//	{
//		//ESP_LOGW(TAG,"SMS operation error\r\n");
//	}
//}
////--------------------------------------------------------------------------------------------------------------//
//void SMS_Process_Configure(uint8_t method)
//{
//	if(method == BufferedSMS)
//	{
//		SMS_Receive_Method = BufferedSMS;
//		ATC_SendATCommand("AT+CNMI=1,1,0,2,0\r\n", "OK", 3000, 3, SMS_Operation_Callback);
//	} else if(method == DirectPassSMS)
//	{
//		SMS_Receive_Method = DirectPassSMS;
//		ATC_SendATCommand("AT+CNMI=2,2,0,2,0\r\n", "OK", 3000, 3, SMS_Operation_Callback);
//	}
//}
////--------------------------------------------------------------------------------------------------------------//
//void SMS_Start(void)
//{
//	ATC_SendATCommand("AT+CMGF=1\r\n", "OK", 3000, 3, SMS_Operation_Callback);
//}
////--------------------------------------------------------------------------------------------------------------//
//void SMS_Read_Process(void *ResponseBuffer)
//{
//	char SMS_Buffer[200];
//	strlcpy(SMS_Buffer, ResponseBuffer, 200);
//	char *tok = strtok(SMS_Buffer, "\r\n");
//	tok = strtok(NULL, "\r\n");
//
//	char utf16[5];
//	char data_out[100];
//	int text_count=0;
//	int i = 0;
//	for (i = 0; i < strlen(tok) - 3;i++)
//	{
//		utf16[0] = tok[i+1];
//		utf16[1] = tok[i+1];
//		utf16[2] = tok[i+2];
//		utf16[3] = tok[i+3];
//		utf16[4] = 0;
//
//		data_out[text_count] = (int)strtol(utf16,0,16);
//		text_count ++;
//		i = i+3;
//	}
//	data_out[text_count] = 0;
//	if(strlen(data_out) > 0)
//	{
//		//ESP_LOGW(TAG,"SMS received: %s, SMS length: %d\r\n", data_out, strlen(data_out));
//		if(strstr(data_out, "D") && strlen(data_out) == 11)
//		{
//			//ESP_LOGW(TAG,"On Demand Mode\r\n");
//		}
//	}
//}
//--------------------------------------------------------------------------------------------------------------//
//int filter_comma(char *respond_data, int begin, int end, char *output)
//{
//	memset(output, 0, strlen(output));
//	int count_filter = 0, lim = 0, start = 0, finish = 0,i;
//	for (i = 0; i < strlen(respond_data); i++)
//	{
//		if ( respond_data[i] == ',')
//		{
//			count_filter ++;
//			if (count_filter == begin)			start = i+1;
//			if (count_filter == end)			finish = i;
//		}
//
//	}
//	lim = finish - start;
//	for (i = 0; i < lim; i++){
//		output[i] = respond_data[start];
//		start ++;
//	}
//	output[i] = 0;
//	return 0;
//}
//--------------------------------------------------------------------------------------------------------------//MCC, RSSI, cellID, LAC, MNC,
void CPSI_Decode(char* str, int* MCC, int* MNC, int* LAC, int* cell_ID, int* RSSI, int16_t* RSRSP, int16_t* RSRQ)
{
	*MCC = *MNC = *LAC = *cell_ID = *RSSI = *RSRSP = *RSRQ = 0;
	if(strstr(str,"GSM"))
	{
		////ESP_LOGW(TAG,"GSM CPSI decode\r\n");
		char temp_buf[50]={0};
		char _LAC[10];
		char RX_buf[10];
		int i=0, head =0, tail = 0, k=0, index=0;
		for(i=0; i < strlen(str); i++)
		{
			if(str[i] == ',') ++k;
			if(k == 1) head = i;
			if(k == 6) tail = i;//4
		}
		for(i=head+2; i <tail+1; i++)
		{
			if(str[i] == '-') str[i] = ',';
			temp_buf[index++] = str[i];
		}
		sscanf(temp_buf, "%d,%d,%[^,],%d,%[^,],,%d", MCC, MNC, _LAC, cell_ID, RX_buf, RSSI);
		*RSSI = -1 * (*RSSI);

		filter_comma(str,4,5,_LAC, ',');
		*LAC = (int)strtol(_LAC, NULL, 0);
		Network_Type = GSM;
		////ESP_LOGW(TAG,"End GSM CPSI decode\r\n");
	}
	if(strstr(str,"LTE"))
	{
		////ESP_LOGW(TAG,"LTE CPSI decode\r\n");
		char temp_buf[50]={0};
		char _LAC[10];
		char RSRP_Buf[10];
		char RSRQ_Buf[10];
		int i = 0, head1=0, tail1=0, head2=0, tail2=0, k=0, index = 0;
		for(i=0; i<strlen(str); i++)
		{
			if(str[i] == ',') ++k;
			if(k == 1) head1 = i;
			if(k == 4) tail1 = i;
			if(k == 10) head2 = i;
			if(k == 11) tail2 = i;
		}
		for(i=head1+2; i<tail1+1; i++)
		{
			if(str[i] == '-') str[i] = ',';
			temp_buf[index++] = str[i];
		}
		for(i=head2+1; i<tail2+1; i++)
		{
			temp_buf[index++] = str[i];
		}
		sscanf(temp_buf, "%d,%d,%[^,],%d,%d", MCC, MNC, _LAC, cell_ID, RSSI);
		*LAC = (int)strtol(_LAC, NULL, 0);

		filter_comma(str, 11, 12, RSRQ_Buf, ',');
		*RSRQ = atoi(RSRQ_Buf);

		filter_comma(str, 12, 13, RSRP_Buf, ',');
		*RSRSP = atoi(RSRP_Buf);

		Network_Type = NB_IoT;
		////ESP_LOGW(TAG,"End LTE CPSI decode\r\n");
	}
}
//--------------------------------------------------------------------------------------------------------------//
void MQTT_Location_Payload_Convert(void)
{
	if(GPS_Fix_Status != 1 || Flag_sos == true || Flag_send_DASP == true || Flag_send_DAST == true)
	{
		Flag_WifiCell_OK = false;
		Bat_Wifi_Cell_Convert(Mqtt_TX_Str, Wifi_Buffer, VTAG_DeviceParameter.Bat_Level, MCC,VTAG_NetworkSignal.RSRP, VTAG_NetworkSignal.RSRQ, RSSI, cell_ID, LAC, MNC);
		if(Flag_sos == 1)
		{
			str_str(Mqtt_TX_Str,"DLBS", "DSOS");
			str_str(Mqtt_TX_Str,"DWFC", "DSOS");
			Flag_sos = false;
		}
		else if(Flag_send_DASP == 1)
		{
			str_str(Mqtt_TX_Str,"DLBS", "DASP");
			str_str(Mqtt_TX_Str,"DWFC", "DASP");
			Flag_send_DASP = false;
		}
		else if(Flag_send_DAST == 1)
		{
			if(VTAG_Configure._SS == 1)
			{
				str_str(Mqtt_TX_Str,"DLBS", "DAST");
				str_str(Mqtt_TX_Str,"DWFC", "DAST");
			}
			Flag_send_DAST = false;
		}
	} else if(GPS_Fix_Status == 1)
	{
		Flag_WifiCell_OK = false;
		if(GPS_Select == GPS_NMEA)
		{
			if(GPS_Fix_Status == 1)
			{
				GPS_Position_Time_Str_Convert(Mqtt_TX_Str, VTAG_DeviceParameter.Bat_Level, 5.0, hgps.latitude, hgps.longitude, hgps.altitude, 2000 + hgps.year, hgps.month, hgps.date, hgps.hours, hgps.minutes, hgps.seconds);
				//ESP_LOGI(TAG, "%s\r\n", Mqtt_TX_Str);
			}
		}
		else if(GPS_Select == GPS_Indirect)
		{
			if(GPS_Fix_Status == 1)
			{
				GPS_Position_Time_Str_Convert(Mqtt_TX_Str, VTAG_DeviceParameter.Bat_Level, 10.0, latitude, longitude, altitude, year, month, day, hour, minute, sec);
				//ESP_LOGI(TAG, "%s\r\n", Mqtt_TX_Str);
				//ESP_LOGI(TAG, "GPS Time to First Fix: %d s\r\n", GPS_Scan_Counter);
			}
		}
		Flag_WifiCell_OK = true;
	}
}

char *str_str (char *s, char *s1, char *s2)
{
	int len = strlen (s);
	int len1 = strlen (s1);
	int len2 = strlen (s2);
	int i = 0, j, luu;
	if (len1 != 0)
	{
		while (i < len)
		{
			if (s[i] == s1[0])
			{
				j = 0;
				luu = i;
				while (s1[++j] == s[++luu] && j < len1);
				if (j == len1)
				{
					memmove (&s[i + len2], &s[i + len1], len - i - len1 + 1);
					memcpy (&s[i], s2, strlen(s2));
					len = len - len1 + len2;
					i += len2;
				}
				else i++;
			} else i++;
		}
	}
	return s;
}
//--------------------------------------------------------------------------------------------------------------//
void MQTT_SendMessage_Thread(VTAG_MessageType Mess_Type)
{
	stepConn = stepDisconn = stepSendData = 0;
	int catch_cell = 0;
	Flag_config = true;
	//	Flag_Wait_Exit = false;
	//	ATC_SendATCommand("AT+CFUN=0\r\n", "OK", 10000, 1, ATResponse_Callback);
	//	WaitandExitLoop(&Flag_Wait_Exit);
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 1, SetCFUN_1_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);

	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		SoftReboot7070G();
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 0, SetCFUN_1_Callback);
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			Backup_reason = CFUN;
			BackUp_UnsentMessage(Mess_Type);
			if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
			{
				vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
				return;
			}
		}
	}
	//Náº¿u cháº¿ Ä‘á»™ chá»�n máº¡ng lÃ  NB thÃ¬ dÃ² NB, náº¿u k cÃ³ NB thÃ¬ chuyá»ƒn GSM
	if(VTAG_Configure.Network == 3 && Flag_LBS_need == false && VTAG_Configure.MA == 0 && Flag_sos == false && Flag_Fota == false)
	{
		//ESP_LOGW(TAG, "Scan NB IoT network %d\r\n", FlagFota2G);
		Flag_Wait_Exit = false;
		SelectNB_GSM_Network(NB_IoT);
		WaitandExitLoop(&Flag_Wait_Exit);

		// Scan network
		Flag_Wait_Exit = false;
		if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
		{
			ATC_SendATCommand("AT+CPSI?\r\n", "NB-IOT,Online,452", 1000, 70, ScanNetwork_Callback); // 70
		}
		else
		{
			ATC_SendATCommand("AT+CPSI?\r\n", "NB-IOT,Online,452", 1000, 45, ScanNetwork_Callback); // 45
		}
		WaitandExitLoop(&Flag_Wait_Exit);

		//Náº¿u khÃ´ng dÃ² Ä‘c Nb thÃ¬ chuyá»ƒn dÃ² GSM
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			// Select GSM network
			if((Flag_use_2g == true && VTAG_Configure.Mode == 2) || VTAG_Configure.Mode == 1)
			{
				Flag_use_2g = false;
				Flag_count_nb_backup = 0;
				//ESP_LOGW(TAG, "Scan GSM network\r\n");
				Flag_Wait_Exit = false;
				SelectNB_GSM_Network(GSM);
				WaitandExitLoop(&Flag_Wait_Exit);
				Reboot7070G();
				Flag_Wait_Exit = false;
				ATC_SendATCommand("AT+CPSI?\r\n", "GSM,Online,452", 1000, 20, ScanNetwork_Callback);
				WaitandExitLoop(&Flag_Wait_Exit);

				if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
				{
					Backup_reason = SCAN_NET;
					BackUp_UnsentMessage(Mess_Type);
					if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
					{
						//						vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
						return;
					}
				}
			}
			else
			{
				Backup_reason = SCAN_NET;
				BackUp_UnsentMessage(Mess_Type);
				if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
				{
					//					vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
					return;
				}
			}
		}
		//Náº¿u dÃ² Ä‘Æ°á»£c NB thÃ¬ dÃ² cÃ¡c thÃ´ng sá»‘ cell, náº¿u RSRQ tháº¥p thÃ¬ chuyá»ƒn qua GSM
		else
		{
			CPSI_Decode(CPSI_Str, &MCC, &MNC, &LAC, &cell_ID, &RSSI, &VTAG_NetworkSignal.RSRP, &VTAG_NetworkSignal.RSRQ);
			//DÃ² cÃ¡c thÃ´ng sá»‘ cell NB náº¿u chÆ°a dÃ² Ä‘c (tá»‘i Ä‘a 20 láº§n)
			catch_cell = 0;
			while((MCC == 0 || MNC == 0 || LAC == 0 || cell_ID == 0 || RSSI == 0) && catch_cell < 10)
			{
				catch_cell++;
				Flag_Wait_Exit = false;
				ATC_SendATCommand("AT+CPSI?\r\n", "Online,452", 1000, 5, ScanNetwork_Callback);
				WaitandExitLoop(&Flag_Wait_Exit);
				CPSI_Decode(CPSI_Str, &MCC, &MNC, &LAC, &cell_ID, &RSSI, &VTAG_NetworkSignal.RSRP, &VTAG_NetworkSignal.RSRQ);
				vTaskDelay(1000/RTOS_TICK_PERIOD_MS);
			}
			if(VTAG_NetworkSignal.RSRP < -120 || VTAG_NetworkSignal.RSRP == 0)
			{
				//XÃ³a cÃ¡c thÃ´ng sá»‘ cell NB Ä‘á»ƒ dÃ¹ng cÃ¡c thÃ´ng sá»‘ cell GSM
				MCC = MNC = LAC = cell_ID = RSSI = 0;
				// Select GSM network
				if((Flag_use_2g == true && (VTAG_Configure.Mode == 2)) || (VTAG_Configure.Mode == 1))
				{
					Flag_count_nb_backup = 0;
					Flag_use_2g = false;
					//ESP_LOGW(TAG, "Scan GSM network\r\n");
					Flag_Wait_Exit = false;
					SelectNB_GSM_Network(GSM);
					WaitandExitLoop(&Flag_Wait_Exit);
					Reboot7070G();
					Flag_Wait_Exit = false;
					ATC_SendATCommand("AT+CPSI?\r\n", "GSM,Online,452", 1000, 15, ScanNetwork_Callback);
					WaitandExitLoop(&Flag_Wait_Exit);

					if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
					{
						Backup_reason = SCAN_NET;
						BackUp_UnsentMessage(Mess_Type);
						if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
						{
							//							vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
							return;
						}
					}
				}
				else
				{
					Backup_reason = SCAN_NET;
					BackUp_UnsentMessage(Mess_Type);
					if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
					{
						//						vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
						return;
					}
				}
			}
		}
	}
	//Náº¿u cháº¿ Ä‘á»™ máº¡ng lÃ  GSM thÃ¬ dÃ² GSM
	else if(VTAG_Configure.Network == 2 || Flag_LBS_need == true || Flag_sos == true || VTAG_Configure.MA != 0 || Flag_Fota == true)
	{
		test_bk++;
		//ESP_LOGW(TAG, "Scan GSM network\r\n");
		Flag_Wait_Exit = false;
		SelectNB_GSM_Network(GSM);
		WaitandExitLoop(&Flag_Wait_Exit);
		if(Flag_LBS_need == true)
		{
			Cell_get_infor(CENG_buf, cell_buffer);
			if(Flag_LBS_need == false && ap_count >= 3)
			{
				return;
			}
		}
		// Scan network
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CPSI?\r\n", "GSM,Online,452", 1000, 15, ScanNetwork_Callback);
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			Backup_reason = SCAN_NET;
			BackUp_UnsentMessage(Mess_Type);
			if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
			{
				//				vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
				return;
			}
		}
	}
	else if(VTAG_Configure.Network == 1 && Flag_LBS_need == false)
	{
		//ESP_LOGW(TAG, "Scan NB IoT network\r\n");
		Flag_Wait_Exit = false;
		SelectNB_GSM_Network(NB_IoT);
		WaitandExitLoop(&Flag_Wait_Exit);

		// Scan network
		Flag_Wait_Exit = false;
		if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
		{
			ATC_SendATCommand("AT+CPSI?\r\n", "NB-IOT,Online,452", 1000, 70, ScanNetwork_Callback); // 70
		}
		else
		{
			ATC_SendATCommand("AT+CPSI?\r\n", "NB-IOT,Online,452", 1000, 45, ScanNetwork_Callback); // 45
		}
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			Backup_reason = SCAN_NET;
			BackUp_UnsentMessage(Mess_Type);
			if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
			{
				return;
			}
		}
	}
	CPSI_Decode(CPSI_Str, &MCC, &MNC, &LAC, &cell_ID, &RSSI, &VTAG_NetworkSignal.RSRP, &VTAG_NetworkSignal.RSRQ);
	// DÃ² cÃ¡c thÃ´ng sá»‘ cá»§a cel (tá»‘i Ä‘a 20 láº§n)
	catch_cell = 0;
	while((MCC == 0 || MNC == 0 || LAC == 0 || cell_ID == 0 || RSSI == 0) && catch_cell < 10)
	{
		catch_cell++;
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CPSI?\r\n", "Online,452", 1000, 5, ScanNetwork_Callback);
		WaitandExitLoop(&Flag_Wait_Exit);
		CPSI_Decode(CPSI_Str, &MCC, &MNC, &LAC, &cell_ID, &RSSI, &VTAG_NetworkSignal.RSRP, &VTAG_NetworkSignal.RSRQ);
		vTaskDelay(1000/RTOS_TICK_PERIOD_MS);
	}

	if(Network_Type == NB_IoT) sprintf(Network_Type_Str, "nb");
	else if(Network_Type == GSM) sprintf(Network_Type_Str, "2g");
	CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
	//Chá»� Ä‘Äƒng kÃ½ Ä‘Æ°á»£c vÃ o máº¡ng
	ATC_SendATCommandOptimize("AT+CGREG?\r\n", "0,1", 1000, 15, ATResponse_Callback);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		Backup_reason = REG_NET;
		BackUp_UnsentMessage(Mess_Type);
		if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
		{
			return;
		}
	}

	Flag_Wait_Exit = false;
	if(AT_RX_event == EVEN_OK && Flag_ScanNetwok == true)
	{
		Flag_ScanNetwok = false;
		ATC_SendATCommand("AT+CNACT?\r\n", "OK", 3000, 3, CheckNetwork_Callback);
		vTaskDelay(100 / RTOS_TICK_PERIOD_MS);
		WaitandExitLoop(&Flag_Wait_Exit);
	}
	vTaskDelay(1000 / RTOS_TICK_PERIOD_MS);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		Backup_reason = CHECK_ACT_NET;
		BackUp_UnsentMessage(Mess_Type);
		if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
		{
			//			vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
			return;
		}
	}
	// Active network
	Flag_MQTT_Connected = false;
	if(AT_RX_event == EVEN_OK && Flag_Network_Active == false)
	{
		//		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CNACT=0,1\r\n", "0,ACTIVE", 15000, 1	, ActiveNetwork_Callback); // 5000
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			Backup_reason = ACT_NET;
			BackUp_UnsentMessage(Mess_Type);
			if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
			{
				ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 1000, 1, NULL);
				return;
			}
		}
	}
	else if(AT_RX_event == EVEN_OK && Flag_Network_Active == true)
	{
		MQTT_Connect_Callback(EVEN_OK, "");
	}
	WaitandExitLoop(&Flag_MQTT_Connected);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 1000, 0, NULL);
		Backup_reason = MQTT_CON;
		BackUp_UnsentMessage(Mess_Type);
		if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
		{
			//			vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
			return;
		}
	}

	// Subcribe to MQTT topic
	//Flag_MQTT_Sub_OK = false;
	memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
	sprintf(MQTT_ID_Topic, "messages/%s/control", DeviceID_TW_Str);
	vTaskDelay(1500/RTOS_TICK_PERIOD_MS);
	MQTT_SubTopic(MQTT_ID_Topic, 0);
	WaitandExitLoop(&Flag_MQTT_Sub_OK);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 1000, 0, NULL);
		Backup_reason = MQTT_SUB;
		BackUp_UnsentMessage(Mess_Type);
		if(Flag_sms_receive == true || Mess_Type == FOTA_START|| Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
		{
			return;
		}
		//TurnOff7070G();
	}
	// Gá»­i báº£n tin backup (Backup_Array_Counter lÃ  sá»‘ báº£n tin backup)
	if(Backup_Array_Counter > 0)
	{
		Flag_sending_backup = true;
		while(Backup_Array_Counter)
		{
			//ThÃªm trÆ°á»�ng retry má»—i báº£n tin backup Ä‘á»ƒ trÃ¡nh trÆ°á»�ng há»£p gá»­i trÃ¹ng báº£n tin thÃ¬ sáº½ k SUB Ä‘Æ°á»£c, gÃ¢y loop
			char retry_buf[20] = {0};
			sprintf(retry_buf, ",\"re\":%d}", retry++);
			sprintf(Location_Backup_Array[0] + strlen(Location_Backup_Array[0]) - 1, "%s", retry_buf);
			// Publish message to MQTT topic
			Flag_MQTT_Publish_OK = false;

			if(strstr(Location_Backup_Array[0], "DOF"))
			{
				ACK_Succeed[WAIT_ACK] = MESS_DOF;
				ACK_Succeed[ACK_DONE] = 0;
			}

			memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
			if(strstr(Location_Backup_Array[0], "DPOS") || strstr(Location_Backup_Array[0], "GET") || strstr(Location_Backup_Array[0], "DPOF"))
			{
				sprintf(MQTT_ID_Topic, "messages/%s/gps", DeviceID_TW_Str);
				MQTT_PubDataToTopic(MQTT_ID_Topic, Location_Backup_Array[0], strlen(Location_Backup_Array[0]), 0, 0);
			}
			else if(strstr(Location_Backup_Array[0], "DWFC") || strstr(Location_Backup_Array[0], "DLBS") || strstr(Location_Backup_Array[0], "DSOS") || strstr(Location_Backup_Array[0], "DASP") || strstr(Location_Backup_Array[0], "DAST") )
			{
				sprintf(MQTT_ID_Topic, "messages/%s/wificell", DeviceID_TW_Str);
				MQTT_PubDataToTopic(MQTT_ID_Topic, Location_Backup_Array[0], strlen(Location_Backup_Array[0]), 0, 0);
			}
			else if(strstr(Location_Backup_Array[0], "DAD") || strstr(Location_Backup_Array[0], "DON") || strstr(Location_Backup_Array[0], "DOF") || strstr(Location_Backup_Array[0], "DCF") || strstr(Location_Backup_Array[0], "DUP") || \
					strstr(Location_Backup_Array[0], "DBL") || strstr(Location_Backup_Array[0], "DBF")|| strstr(Location_Backup_Array[0], "DBO")|| strstr(Location_Backup_Array[0], "DBOF")|| strstr(Location_Backup_Array[0], "DOS")|| \
					strstr(Location_Backup_Array[0], "DOSS") || strstr(Location_Backup_Array[0], "DOFA"))
			{
				sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
				MQTT_PubDataToTopic(MQTT_ID_Topic, Location_Backup_Array[0], strlen(Location_Backup_Array[0]), 0, 0);
			}
			WaitandExitLoop(&Flag_MQTT_Publish_OK);

			// Delay for receive MQTT sub message
			MQTT_SubReceive_Wait(MQTT_SUB_TIMEOUT);// Wait for maximum of 5s
			Flag_Done_Get_Accurency = false;
			char *Sub_Str = strstr(MQTT_SubMessage, "{");
			JSON_Analyze_backup(Sub_Str, &VTAG_Configure);
			if(Flag_asyn_ts == false)
			{
				Flag_asyn_ts = true;
				struct timeval time_now;
				gettimeofday(&time_now, 0);
				long t_on = atol(VTAG_Configure.Server_Timestamp) - time_now.tv_sec;
				for(int i = 0; i < Backup_Array_Counter; i++)
				{
					String_process_backup_message(Location_Backup_Array[i], t_on);
					//ESP_LOGI(TAG, "%s\r\n", Location_Backup_Array[i]);
				}
				long Server_Timestamp_l = atol(VTAG_Configure.Server_Timestamp);
				struct timeval epoch = {Server_Timestamp_l, 0};
				struct timezone utc = {0,0};
				settimeofday(&epoch, &utc);
				GetDeviceTimestamp();
			}
			//Náº¿u lÃ  báº£n tin DOF thÃ¬ táº¯t nguá»“n
			if(ACK_Succeed[WAIT_ACK] == MESS_DOF && ACK_Succeed[ACK_DONE] == 1)
			{
				//ESP_LOGI(TAG, "Shut down V-TAG\r\n");
				LED_TurnOff();
				gpio_hold_dis(ESP32_PowerLatch);
				gpio_set_level(PowerLatch, 0);
				vTaskDelay(10000/RTOS_TICK_PERIOD_MS);
			}
			Backup_Array_Counter--;
			for(int i = 0; i < Backup_Array_Counter; i++)
			{
				strcpy(Location_Backup_Array[i], Location_Backup_Array[i+1]);
			}
			retry = 0;
		}
		Retry_count = 0;
		Flag_sending_backup = false;
		if(Mess_Type == SEND_BACKUP)
		{
			ESP_sleep(1);
			if(Flag_sos == true || Flag_send_DAST == true)
			{
				Mess_Type = LOCATION;
				return;
			}
			else if(Flag_Unpair_Task == true)
			{
				Mess_Type = UNPAIR_GET;
				return;
			}
			else if(Flag_sms_receive == true)
			{
				//				vTaskDelay(2000/RTOS_TICK_PERIOD_MS);
				return;
			}
		}
	}
	Backup_Array_Counter = 0;
	bool Flag_UnpairGET_IsSent = false;

	CONVERTPAYLOAD:
	// Convert MQTT payload
	if(Mess_Type == FOTA_SUCCESS)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send FOTA_SUCCESS message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DOSS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOSS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == FOTA_START)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send FOTA START message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_FOTA_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, 0, VTAG_Vesion, Network_Type_Str, VTAG_Configure.Network);
		}
		else
		{
			MQTT_DevConf_FOTA_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, 0, VTAG_Vesion, Network_Type_Str, VTAG_Configure.Network);
		}
	}
	else if(Mess_Type == FOTA_FAIL)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send FOTA_FAIL message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DOFA", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOFA", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == LOCATION)
	{
		//ESP_LOGE(TAG, "Send LOCATION message\r\n");
		MQTT_Location_Payload_Convert();
	}
	else if(Mess_Type == STARTUP)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send DON message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DonDevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DON", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Device_shutdown_ts, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA, loopString);
		}
		else
		{
			MQTT_DonDevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DON", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode,  VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Device_shutdown_ts, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA, loopString);
		}
	}
	else if(Mess_Type == LOWBATTERY)
	{
		GetDeviceTimestamp();
		//ESP_LOGW(TAG,"Send DBL message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DBL", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DBL", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == FULLBATTERY)
	{
		GetDeviceTimestamp();
		Bat_raw_pre = 100;
		Check_battery();
		//ESP_LOGE(TAG, "Send DBF message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DBF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode,  VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, 100, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DBF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode,  VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, 100, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	// Newly added
	else if(Mess_Type == PAIR)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send DAD message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DAD", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DAD", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == UNPAIR)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send DUP message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DUP", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DUP", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == UNPAIR_GET)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send GET message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "GET", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "GET", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == GPS_FAIL)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send DPOF message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DPOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DPOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == DCF_ACK)
	{
		GetDeviceTimestamp();
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DCF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DCF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	// Newly added
	else if(Mess_Type == OFF_ACK)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send DOF message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOF", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	// Publish message to MQTT topic
	Flag_MQTT_Publish_OK = false;
	memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
	if(strstr(Mqtt_TX_Str, "DPOS") || strstr(Mqtt_TX_Str, "GET") ||  strstr(Mqtt_TX_Str, "DPOF"))
	{
		ACK_Succeed[0] = 3;
		sprintf(MQTT_ID_Topic, "messages/%s/gps", DeviceID_TW_Str);
		MQTT_PubDataToTopic(MQTT_ID_Topic, Mqtt_TX_Str, strlen(Mqtt_TX_Str), 0, 0);
		ACK_Succeed[1] = 1;
	}
	else if(strstr(Mqtt_TX_Str, "DWFC") ||  strstr(Mqtt_TX_Str, "DLBS") || strstr(Mqtt_TX_Str, "DSOS") || strstr(Mqtt_TX_Str, "DASP") || strstr(Mqtt_TX_Str, "DAST"))
	{
		ACK_Succeed[0] = 3;
		sprintf(MQTT_ID_Topic, "messages/%s/wificell", DeviceID_TW_Str);
		MQTT_PubDataToTopic(MQTT_ID_Topic, Mqtt_TX_Str, strlen(Mqtt_TX_Str), 0, 0);
		ACK_Succeed[1] = 1;
	}
	else if(strstr(Mqtt_TX_Str, "DAD") || strstr(Mqtt_TX_Str, "DON") || strstr(Mqtt_TX_Str, "DOF") || strstr(Mqtt_TX_Str, "DCF") || strstr(Mqtt_TX_Str, "DUP") || strstr(Mqtt_TX_Str, "DBL") || strstr(Mqtt_TX_Str, "DBF") \
			|| strstr(Mqtt_TX_Str, "DOFA") || strstr(Mqtt_TX_Str, "DOSS") || strstr(Mqtt_TX_Str, "DOS") || strstr(Mqtt_TX_Str, "DBO")|| strstr(Mqtt_TX_Str, "DBOF"))
	{
		sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
		MQTT_PubDataToTopic(MQTT_ID_Topic, Mqtt_TX_Str, strlen(Mqtt_TX_Str), 0, 0);
	}
	WaitandExitLoop(&Flag_MQTT_Publish_OK);

	// Delay for receive MQTT sub message
	MQTT_SubReceive_Wait(MQTT_SUB_TIMEOUT);// Wait for maximum of 5s

	Flag_Done_Get_Accurency = false;
	char *Sub_Str = strstr(MQTT_SubMessage, "{");
	JSON_Analyze(Sub_Str, &VTAG_Configure_temp);
	FlagFota2G = VTAG_Configure_temp.F2G;
	Flag_asyn_ts = true;
	//Calib factor if time slept longer than ...
	if(Flag_calib == true && t_slept_calib > 1*3600)
	{
		Flag_calib = false;
		//ESP_LOGI(TAG, "VTAG_Configure_temp.Server_Timestamp: %lld", atoll(VTAG_Configure_temp.Server_Timestamp));
		//ESP_LOGI(TAG, "Gettimeofday_capture: %lld", Gettimeofday_capture);
		//ESP_LOGI(TAG, "time_slept + TrackingRuntime: %lld", (t_slept_calib + TrackingRuntime));

		Calib_factor =(float) (atoll(VTAG_Configure_temp.Server_Timestamp) - Gettimeofday_capture) / (t_slept_calib + TrackingRuntime);
		//write calib factor to flash
		if(Calib_factor > 1.07)
		{
			Calib_factor = 1.05;
		}
		char calib_str[50] = "";
		sprintf(calib_str, "%f", Calib_factor);
		writetofile(base_path, "calib.txt", calib_str);
		//ESP_LOGI(TAG, "Calib_factor: %f \r\n", Calib_factor);
	}
	//Ä�á»•i cáº¥u hÃ¬nh thiáº¿t bá»‹ náº¿u cÃ³ sá»± thay Ä‘á»•i  Period, CC, Network vÃ  gá»­i DCF
	if((VTAG_Configure_temp.Period != VTAG_Configure.Period || VTAG_Configure_temp.CC != VTAG_Configure.CC || \
			VTAG_Configure_temp.Network != VTAG_Configure.Network || VTAG_Configure_temp.Mode != VTAG_Configure.Mode ||\
			VTAG_Configure_temp._SS != VTAG_Configure._SS || VTAG_Configure_temp.WM != VTAG_Configure.WM || VTAG_Configure_temp._lc != VTAG_Configure._lc || VTAG_Configure_temp.MA != VTAG_Configure.MA) \
			&& (strstr(VTAG_Configure_temp.Type, "O") == NULL)&& \
			VTAG_Configure_temp.Period != 0 && VTAG_Configure_temp.CC != 0)
	{
#if LOG
		//ESP_LOGI(TAG, "UPDATE CONFIG");
		if(VTAG_Configure_temp.Period != VTAG_Configure.Period)
		{
			//ESP_LOGI(TAG, "Period");
		}
		if(VTAG_Configure_temp.CC != VTAG_Configure.CC)
		{
			//ESP_LOGI(TAG, "UPDATE cc");
		}
		if(VTAG_Configure_temp.Network != VTAG_Configure.Network)
		{
			//ESP_LOGI(TAG, "UPDATE Network");
		}
		if(VTAG_Configure_temp._SS != VTAG_Configure._SS)
		{
			//ESP_LOGI(TAG, "UPDATE _SS");
		}
		if(VTAG_Configure_temp.WM != VTAG_Configure.WM)
		{
			//ESP_LOGI(TAG, "UPDATE WM");
		}
		if(VTAG_Configure_temp._lc != VTAG_Configure._lc)
		{
			//ESP_LOGI(TAG, "UPDATE _lc");
		}
		if(VTAG_Configure_temp.MA != VTAG_Configure.MA)
		{
			//ESP_LOGI(TAG, "UPDATE MA");
		}
#endif
		Flag_Unpair_led = false;
		Flag_ChangeMode= true;
		JSON_Analyze(Sub_Str, &VTAG_Configure);
		VTAG_Configure.F2G = 0;
		char str[150];
		sprintf(str, "{\"M\":%d,\"P\":%d,\"Type\":\"%s\",\"CC\":%d,\"N\":%d,\"a\":%d,\"T\":\"%s\",\"_ss\":%d,\"WM\":%d,\"_lc\":%d, \"MA\":%d}",\
				VTAG_Configure.Mode,VTAG_Configure.Period,VTAG_Configure.Type,VTAG_Configure.CC,VTAG_Configure.Network,\
				VTAG_Configure.Accuracy,VTAG_Configure.Server_Timestamp,VTAG_Configure._SS, VTAG_Configure.WM, \
				VTAG_Configure._lc, VTAG_Configure.MA);
		writetofile(base_path, "test.txt", str);
		Flag_ChangeMode_led = true;
		if(Flag_config_sms == true)
		{
			Flag_config_sms = false;
			Flag_config_sendLocation = true;
		}
		Mess_Type = DCF_ACK;
		goto CONVERTPAYLOAD;
	}
	JSON_Analyze(Sub_Str, &VTAG_Configure);
	timestampSMS = VTAG_DeviceParameter.Device_Timestamp;
	printf("%ld", timestampSMS);
	Flag_Done_Get_Accurency = true;

	// Save configure to flash Pair/Unpair status
	if(strstr(Device_PairStatus, VTAG_Configure.Type) == NULL) // if there is a change in pair/unpair status
	{
		if(strstr(VTAG_Configure.Type, "P") || strstr(VTAG_Configure.Type, "U") || strstr(VTAG_Configure.Type, "O") || strstr(VTAG_Configure.Type, "C"))
		{
			if(ACK_Succeed[WAIT_ACK] ==  MESS_UNPAIR && ACK_Succeed[ACK_DONE] == 1) // if DUP is succeed
			{
				ACK_Succeed[WAIT_ACK] = ACK_Succeed[ACK_DONE] = 0;
				//ESP_LOGE(TAG, "Write configure to flash\r\n");
				writetofile(base_path, "vtag_pr.txt", "U");
				strcpy(Device_PairStatus, "U");
				char str[100];
				sprintf(str, "{\"M\":%d,\"P\":%d,\"Type\":\"%s\",\"CC\":%d,\"N\":%d,\"a\":%d,\"T\":\"%s\",\"_ss\":%d,\"WM\":%d,\"_lc\":%d,\"MA\":%d}", 1, 5,VTAG_Configure.Type, 1, VTAG_Configure.Network ,VTAG_Configure.Accuracy, "0", 1, 0, 1, 1);
				writetofile(base_path, "test.txt", str);
				// Shutdown device
				//ESP_LOGI(TAG, "Shut down V-TAG\r\n");
				xSemaphoreTake(xMutex_LED, portMAX_DELAY);
				//ESP_LOGE(TAG, "Lock xMutex_LED_3 \r\n");
				gpio_set_level(LED_1, 1);
				vTaskDelay(3000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				gpio_hold_dis(PowerLatch);
				gpio_set_level(PowerLatch, 0);
				vTaskDelay(10000 / RTOS_TICK_PERIOD_MS);
				//ESP_LOGE(TAG, "Unlock_xMutex_LED_3 \r\n");
				xSemaphoreGive(xMutex_LED);
			}
			else if(ACK_Succeed[WAIT_ACK] ==  MESS_PAIR && ACK_Succeed[ACK_DONE] == 1)
			{
				//ESP_LOGE(TAG, "Write configure to flash\r\n");
				writetofile(base_path, "vtag_pr.txt", "P");
				strcpy(Device_PairStatus, "P");
				ACK_Succeed[WAIT_ACK] = ACK_Succeed[ACK_DONE] = 0;
				xSemaphoreTake(xMutex_LED, portMAX_DELAY);
				gpio_set_level(LED_2, 1);
				vTaskDelay(3000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				xSemaphoreGive(xMutex_LED);
			}
			else if(ACK_Succeed[WAIT_ACK] ==  MESS_NONE && ACK_Succeed[ACK_DONE] == 0)
			{
				if((strstr(Device_PairStatus,"U") || strlen(Device_PairStatus) == 0) && (strstr(VTAG_Configure.Type, "P") || strstr(VTAG_Configure.Type, "O") || strstr(VTAG_Configure.Type, "C")))
				{
					Flag_Pair_Task = true;
				}
			}
		}
	}
	if(Flag_Fota == false && FlagFota2G == 1)
	{
		//ESP_LOGI(TAG, "Start Fota by 2G");
		acc_power_down();
		Flag_Fota = true;
		ATC_SendATCommandOptimize(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 10000, 1, ATResponse_Callback);
		xTaskCreatePinnedToCore(Fota2G_processing_task, "uart", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &Fota2G_processing_handle, tskNO_AFFINITY);
		if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED && check_motion_handle != NULL)
		{
			vTaskDelete(check_motion_handle);
		}
		vTaskDelete(gps_scan_task_handle);
		vTaskDelete(gps_processing_task_handle);
		vTaskDelete(NULL);
	}
	if(Mess_Type == FOTA_START || Mess_Type == FOTA_FAIL ||  Mess_Type == FOTA_SUCCESS)
	{
		ATC_SendATCommandOptimize(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 10000, 1, ATResponse_Callback);
		return;
	}
	if(VTAG_Configure.MA != 0)
	{
		//ESP_LOGI(TAG, "turn off detect sensor after DCF");
		Flag_motion_detected = false;
		acc_power_down();
		Flag_send_DAST = false;
	}
	//check acc_counter if timeout then send set DASP immediately in order to reduce time scan cellular
	if(Flag_motion_detected == true && Flag_send_DAST == false)
	{
		//ESP_LOGI(TAG, "what the fuck what's that man");
		acc_counter = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - acc_capture, esp_clk_slowclk_cal_get())/1000000);
//		ESP_LOGI(TAG, "acc_counter: %d s",(int)acc_counter);
		if(acc_counter > 180 || Flag_send_DASP == true)
		{
			acc_counter = 0;
			Flag_motion_detected = false;
			flag_end_motion = true;
			Flag_send_DASP = true;
			Flag_StopMotion_led = true;
			if(VTAG_Configure._SS == 0){
				Flag_send_DASP = false;
			}
		}
	}
	// Processing unpair task, including two steps: unpair get and unpair ack
	if(Flag_Unpair_Task == true)
	{
		if(strstr(VTAG_Configure.Type, "U"))// Received unpair msg
		{
			Mess_Type = UNPAIR;
			Flag_Unpair_Task = false;
			ACK_Succeed[WAIT_ACK] = MESS_UNPAIR;
			ACK_Succeed[ACK_DONE] = 0;
			goto CONVERTPAYLOAD;
		}
		else if(Flag_UnpairGET_IsSent == false)
		{
			Mess_Type = UNPAIR_GET;
			Flag_Unpair_Task = false;
			Flag_UnpairGET_IsSent = true;
			goto CONVERTPAYLOAD;
		}
	}

	if(strstr(VTAG_Configure.Type, "U"))
	{
		if(Flag_unpair_DAST == false)
		{
			Flag_unpair_DAST = true;
			Mess_Type = UNPAIR;
			ACK_Succeed[WAIT_ACK] = MESS_UNPAIR;
			ACK_Succeed[ACK_DONE] = 0;
			goto CONVERTPAYLOAD;
		}
		else
		{
			//ESP_LOGI(TAG, "Shut down V-TAG\r\n");
			gpio_hold_dis(PowerLatch);
			gpio_set_level(PowerLatch, 0);
			vTaskDelay(10000 / RTOS_TICK_PERIOD_MS);
		}
	}

	// Processing pair task
	if(Flag_Pair_Task == true)
	{
		if((strstr(Device_PairStatus,"U") || strlen(Device_PairStatus) == 0) && (strstr(VTAG_Configure.Type, "P") || strstr(VTAG_Configure.Type, "O") || strstr(VTAG_Configure.Type, "C")))// Received pair msg
		{
			Mess_Type = PAIR;
			ACK_Succeed[WAIT_ACK] = MESS_PAIR;
			ACK_Succeed[ACK_DONE] = 0;
			Flag_Pair_Task = false;
			goto CONVERTPAYLOAD;
		}
	}

	//ACK turn off device
	if(ACK_Succeed[WAIT_ACK] == MESS_DOF && ACK_Succeed[ACK_DONE] == 1)
	{
		//ESP_LOGI(TAG, "Shut down V-TAG\r\n");
		LED_TurnOff();
		gpio_hold_dis(ESP32_PowerLatch);
		gpio_set_level(PowerLatch, 0);
		vTaskDelay(10000/RTOS_TICK_PERIOD_MS);
	}
	if(strstr(VTAG_Configure.Type, "O"))
	{
		ACK_Succeed[WAIT_ACK] = MESS_DOF;
		ACK_Succeed[ACK_DONE] = 0;
		Mess_Type = OFF_ACK;
		Flag_shutdown_dev = false;
		goto CONVERTPAYLOAD;
	}
	// DBL task
	if(Flag_DBL_Task == true)
	{
		if(VTAG_DeviceParameter.Bat_Level <= 15)
		{
			Mess_Type = LOWBATTERY;
			Flag_DBL_Task = false;
			goto CONVERTPAYLOAD;
		}
	}
	// DBF task
	if(Flag_FullBattery == true)
	{
		Mess_Type = FULLBATTERY;
		Flag_FullBattery = false;
		goto CONVERTPAYLOAD;
	}
	//SOS
	if(Flag_sos == true)
	{
		//ESP_LOGI(TAG, "Send SOS after send an another message");
		//		Flag_wifi_scan = true;
		if(Flag_wifi_init == false)
		{
			wifi_init();
		}
		wifi_scan();
		GetDeviceTimestamp();
		Flag_config_sms = false;
		Flag_config_sendLocation = false;
		Mess_Type = LOCATION;
		//Flag_sos = false;
		goto CONVERTPAYLOAD;
	}
	//DA
	if(Flag_send_DASP == true)
	{
		//ESP_LOGI(TAG, "Send DASP after send an another message");
		Flag_motion_detected = false;
		//		Flag_wifi_scan = true;
		if(Flag_wifi_init == false)
		{
			wifi_init();
		}
		wifi_scan();
		GetDeviceTimestamp();
		Flag_config_sms = false;
		Flag_config_sendLocation = false;
		Mess_Type = LOCATION;
		//Flag_sos = false;
		goto CONVERTPAYLOAD;
	}
	if(Flag_send_DAST == true)
	{
		//ESP_LOGI(TAG, "Send DAST after send an another message");
		//		Flag_wifi_scan = true;
		if(Flag_wifi_init == false)
		{
			wifi_init();
		}
		wifi_scan();
		GetDeviceTimestamp();
		Flag_config_sms = false;
		Flag_config_sendLocation = false;
		Mess_Type = LOCATION;
		//Flag_sos = false;
		goto CONVERTPAYLOAD;
	}
	if(ProgramRun_Cause == ESP_SLEEP_WAKEUP_UNDEFINED)
	{
		//ESP_LOGI(TAG, "Send location after DON");
		GetDeviceTimestamp();
		Mess_Type = LOCATION;
		Flag_config_sms = false;
		Flag_config_sendLocation = false;
		ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
		goto CONVERTPAYLOAD;
	}
	if(Flag_config_sms == true || Flag_config_sendLocation == true)
	{
		//ESP_LOGI(TAG, "sms conffig");
		//		Flag_wifi_scan = true;
		if(Flag_wifi_init == false)
		{
			wifi_init();
		}
		wifi_scan();
		GetDeviceTimestamp();
		Mess_Type = LOCATION;
		Flag_config_sms = false;
		Flag_config_sendLocation = false;
		goto CONVERTPAYLOAD;
	}
	Retry_count = 0;
	// Disconnect MQTT
	Flag_MQTT_Stop_OK = false;
	ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 10000, 3, MQTT_Disconnected_Callback);
	WaitandExitLoop(&Flag_MQTT_Stop_OK);
}
//--------------------------------------------------------------------------------------------------------------//
void MQTT_SendMessage_Thread_Fota(VTAG_MessageType Mess_Type)
{
	stepConn = stepDisconn = stepSendData = 0;
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 0, SetCFUN_1_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);

	//ESP_LOGW(TAG, "Scan GSM network\r\n");
	Flag_Wait_Exit = false;
	SelectNB_GSM_Network(GSM);
	WaitandExitLoop(&Flag_Wait_Exit);
	// Scan network
	Flag_Wait_Exit = false;
	ATC_SendATCommand("AT+CPSI?\r\n", "GSM,Online,452", 1000, 20, ScanNetwork_Callback);
	WaitandExitLoop(&Flag_Wait_Exit);

	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		return;
	}

	CPSI_Decode(CPSI_Str, &MCC, &MNC, &LAC, &cell_ID, &RSSI, &VTAG_NetworkSignal.RSRP, &VTAG_NetworkSignal.RSRQ);
	ATC_SendATCommandOptimize("AT+CGREG?\r\n", "0,1", 1000, 30, ATResponse_Callback);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		return;
	}

	if(Network_Type == GSM) sprintf(Network_Type_Str, "2g");
	// Check status of network
	Flag_Wait_Exit = false;
	if(AT_RX_event == EVEN_OK && Flag_ScanNetwok == true)
	{
		Flag_Wait_Exit = false;
		Flag_ScanNetwok = false;
		ATC_SendATCommand("AT+CNACT?\r\n", "OK", 3000, 3, CheckNetwork_Callback);
		vTaskDelay(100 / RTOS_TICK_PERIOD_MS);
		WaitandExitLoop(&Flag_Wait_Exit);
	}
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		return;
	}
	// Active network
	Flag_MQTT_Connected = false;
	if(AT_RX_event == EVEN_OK && Flag_Network_Active == false)
	{
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CNACT=0,1\r\n", "0,ACTIVE", 15000, 1, ActiveNetwork_Callback); // 5000
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			return;
		}
	}
	else if(AT_RX_event == EVEN_OK && Flag_Network_Active == true)
	{
		MQTT_Connect_Callback(EVEN_OK, "");
	}
	WaitandExitLoop(&Flag_MQTT_Connected);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		return;
	}
	// Subcribe to MQTT topic
	//Flag_MQTT_Sub_OK = false;
	memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
	sprintf(MQTT_ID_Topic, "messages/%s/control", DeviceID_TW_Str);

	Flag_MQTT_Sub_OK = false;
	MQTT_SubTopic(MQTT_ID_Topic, 0);
	WaitandExitLoop(&Flag_MQTT_Sub_OK);
	if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
	{
		return;
	}

	// Convert MQTT payload

	if(Mess_Type == FOTA_SUCCESS)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send FOTA_SUCCESS message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DOSS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOSS", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}
	else if(Mess_Type == FOTA_FAIL)
	{
		GetDeviceTimestamp();
		//ESP_LOGE(TAG, "Send FOTA_FAIL message\r\n");
		if(Network_Type == GSM)
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, RSSI, VTAG_Configure.CC, "DOFA", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
		else
		{
			MQTT_DevConf_Payload_Convert(Mqtt_TX_Str, VTAG_NetworkSignal.RSRP, VTAG_Configure.CC, "DOFA", VTAG_NetworkSignal.RSRQ, VTAG_Configure.Period, VTAG_Configure.Mode, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, VTAG_DeviceParameter.Bat_Level, Network_Type_Str, VTAG_Configure.Network, VTAG_Configure._SS, VTAG_Configure.WM, VTAG_Configure._lc, VTAG_Configure.MA);
		}
	}

	// Publish message to MQTT topic
	memset(MQTT_ID_Topic, 0, sizeof(MQTT_ID_Topic));
	if(strstr(Mqtt_TX_Str, "DOFA") || strstr(Mqtt_TX_Str, "DOSS"))
	{
		sprintf(MQTT_ID_Topic, "messages/%s/devconf", DeviceID_TW_Str);
		Flag_MQTT_Publish_OK = false;
		MQTT_PubDataToTopic(MQTT_ID_Topic, Mqtt_TX_Str, strlen(Mqtt_TX_Str), 0, 0);
		WaitandExitLoop(&Flag_MQTT_Publish_OK);
		if(AT_RX_event == EVEN_ERROR || AT_RX_event == EVEN_TIMEOUT)
		{
			return;
		}
	}
	// Delay for receive MQTT sub message
	ACK_Succeed[WAIT_ACK] = MESS_DOFA;
	ACK_Succeed[ACK_DONE] = 0;
	MQTT_SubReceive_Wait(MQTT_SUB_TIMEOUT);// Wait for maximum of 5s
	char *Sub_Str = strstr(MQTT_SubMessage, "{");
	JSON_Analyze(Sub_Str, &VTAG_Configure);
	if(ACK_Succeed[WAIT_ACK] == MESS_DOFA && ACK_Succeed[ACK_DONE] == 1)
	{
		//		esp_restart();
		forcedReset();
	}
	else
	{
		return;
	}

	// Disconnect MQTT
	Flag_MQTT_Stop_OK = false;
	ATC_SendATCommand(AT_MQTT_List[MQTT_STATE_DISC].content, "OK", 10000, 3, MQTT_Disconnected_Callback);
	WaitandExitLoop(&Flag_MQTT_Stop_OK);
}
//--------------------------------------------------------------------------------------------------------------//
void Bat_Wifi_Cell_Convert(char* str, char* wifi, uint8_t level, int MCC,int16_t RSRP, int16_t RSRQ, int RSSI,int cellID,int LAC,int MNC)
{
	if(Flag_LBS_need)
	{
		if(Network_Type == GSM)
		{
			sprintf(str,"{\"ss\":%d,\"Type\":\"DLBS\",\"_Type\":\"LBS\",\"B\":%d,\"r\":%d,\"C\":%s,\"T\":%lld,\"V\":\"%s\",\"RR\":%d%d,", RSSI, level, RSRQ, cell_buffer, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, Reboot_reason, Backup_reason);
			printf("cell_buffer: %s\r\n", str);
		}
		else
		{
			sprintf(str,"{\"ss\":%d,\"Type\":\"DLBS\",\"_Type\":\"LBS\",\"B\":%d,\"r\":%d,\"C\":%s,\"T\":%lld,\"V\":\"%s\",\"RR\":%d%d,", RSRP, level, RSRQ, cell_buffer, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, Reboot_reason, Backup_reason);
		}
		if (Network_Type == GSM)
		{
			sprintf(str+strlen(str),"%s","\"W\":[],\"Cn\":\"2g\"}");
		}
		else if(Network_Type == NB_IoT)
		{
			sprintf(str+strlen(str),"%s","\"W\":[],\"Cn\":\"nb\"}");
		}
	}
	else
	{
		if(Network_Type == GSM)
		{
			sprintf(str,"{\"ss\":%d,\"Type\":\"DWFC\",\"_Type\":\"WIFI\",\"B\":%d,\"r\":%d,\"C\":[{\"C\":%d,\"S\":%d,\"ID\":%d,\"L\":%d,\"N\":%d}],\"T\":%lld,\"V\":\"%s\",\"RR\":%d%d,", RSSI, level, RSRQ, MCC, RSSI, cellID, LAC, MNC, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, Reboot_reason, Backup_reason);
		}
		else
		{
			sprintf(str,"{\"ss\":%d,\"Type\":\"DWFC\",\"_Type\":\"WIFI\",\"B\":%d,\"r\":%d,\"C\":[{\"C\":%d,\"S\":%d,\"ID\":%d,\"L\":%d,\"N\":%d}],\"T\":%lld,\"V\":\"%s\",\"RR\":%d%d,", RSRP, level, RSRQ, MCC, RSSI, cellID, LAC, MNC, VTAG_DeviceParameter.Device_Timestamp, VTAG_Vesion, Reboot_reason, Backup_reason);
		}
		if(strlen(wifi) > 0)
		{
			sprintf(str+strlen(str),"%s",wifi);
		}
		if (Network_Type == GSM)
		{
			if(strlen(wifi) > 0)
			{
				sprintf(str+strlen(str),"%s",",\"Cn\":\"2g\"}");
			}
			else
			{
				sprintf(str+strlen(str),"%s","\"W\":[],\"Cn\":\"2g\"}");
			}
		}
		else if(Network_Type == NB_IoT)
		{
			if(strlen(wifi) > 0)
			{
				sprintf(str+strlen(str),"%s",",\"Cn\":\"nb\"}");
			}
			else
			{
				sprintf(str+strlen(str),"%s","\"W\":[],\"Cn\":\"nb\"}");
			}
		}
	}
	//ESP_LOGW(TAG, "MQTT string: %s\r\n", str);
	Flag_WifiCell_OK = true;
}
//------------------------------------------------------------------------------------------------------------------------//
long convert_date_to_epoch(int day_t,int month_t,int year_t ,int hour_t,int minute_t,int second_t)
{
	struct tm time;
	time.tm_year 	= year_t - 1900;  			// Year - 1900
	time.tm_mon 	= month_t - 1;           	// Month, where 0 = jan
	time.tm_mday 	= day_t;          			 // Day of the month
	time.tm_hour 	= hour_t;
	time.tm_min 	= minute_t;
	time.tm_sec 	= second_t;
	time.tm_isdst 	= -1;        			// Is DST on? 1 = yes, 0 = no, -1 = unknown

	long epoch = (long) mktime(&time);
	//ESP_LOGW(TAG,"epoch_convert = %ld\r\n", epoch);
	return epoch;
}
//------------------------------------------------------------------------------------------------------------------------//
time_t string_to_seconds(const char *timestamp_str)
{
	struct tm tm;
	time_t seconds;
	int r;

	if (timestamp_str == NULL) {
		printf("null argument\n");
		return (time_t)-1;
	}
	r = sscanf(timestamp_str, "%d/%d/%d,%d:%d:%d+28", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
	if (r != 6) {
		printf("expected %d numbers scanned in %s\n", r, timestamp_str);
		return (time_t)-1;
	}

	tm.tm_year += 100 ;
	tm.tm_mon -= 1;
	tm.tm_hour-=7;
	tm.tm_isdst = 0;
	seconds = mktime(&tm);
	if (seconds == (time_t)-1) {
		printf("reading time from %s failed\n", timestamp_str);
	}

	return seconds;
}
//------------------------------------------------------------------------------------------------------------------------//
void GetDeviceTimestamp(void)
{
	struct timeval time_now;
	gettimeofday(&time_now, 0);
	VTAG_DeviceParameter.Device_Timestamp = time_now.tv_sec;
	//ESP_LOGW(TAG,"Device timestamp: %lld\r\n", VTAG_DeviceParameter.Device_Timestamp);
	//	timestampSMS = VTAG_DeviceParameter.Device_Timestamp;
	//	printf("%ld", timestampSMS);
}
//------------------------------------------------------------------------------------------------------------------------// FAT flash function
void MountingFATFlash(void)
{
	//ESP_LOGI(TAG, "Mounting FAT file system");
	const esp_vfs_fat_mount_config_t mount_config =
	{
			.max_files = 6,
			.format_if_mount_failed = true,
			.allocation_unit_size = CONFIG_WL_SECTOR_SIZE
	};
	esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
	if (err != ESP_OK)
	{
		//ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
		return;
	}
}
bool ReadingFATFlash(char* FilePath, char* FileName, char* buffer)
{
	// Open file for reading
	//ESP_LOGI(TAG, "Reading file");
	char file[64];
	sprintf(file, "%s/%s", (char*)FilePath, (char*)FileName);
	FILE *f = fopen(file, "rb");
	//FILE *f = fopen("/spiflash/hello.txt", "r+");
	if (f == NULL) {
		//ESP_LOGE(TAG, "Failed to open file for reading");
		return 0;
	}
	char line[1024];
	fgets(line, sizeof(line), f);
	fclose(f);
	// strip newline
	char *pos = strchr(line, '\n');
	if (pos) {
		*pos = '\0';
	}
	strcpy(buffer, line);
	//ESP_LOGI(TAG, "Read from file: '%s'", buffer);
	return 1;
}
bool WritingFATFlash(char* FilePath, char* FileName, char* FileContent)
{
	//ESP_LOGI(TAG, "Opening file");
	char file[64];
	sprintf(file, "%s/%s", FilePath, FileName);
	FILE *f = fopen(file, "wb");
	//FILE *f = fopen("/spiflash/hello.txt", "wb");
	if (f == NULL) {
		//ESP_LOGE(TAG, "Failed to open file for writing");
		return 0;
	}
	fprintf(f, "%s\n", FileContent);
	fclose(f);
	//ESP_LOGI(TAG, "File written");
	return 1;
}
bool AppendingFATFlash(char* FilePath, char* FileName, char* FileContent)
{
	//ESP_LOGI(TAG, "Opening file");
	char file[64];
	sprintf(file, "%s/%s", FilePath, FileName);
	FILE *f = fopen(file, "a");
	//FILE *f = fopen("/spiflash/hello.txt", "wb");
	if (f == NULL)
	{
		//ESP_LOGE(TAG, "Failed to open file for appending");
		return 0;
	}
	fprintf(f, "%s", FileContent);
	fclose(f);
	//ESP_LOGI(TAG, "File appended");
	return 1;
}
//------------------------------------------------------------------------------------------------------------------------//
void ResetAllParameters(void)
{
	GPS_Fix_Status = 0;
	GPS_Scan_Counter = 0;
	Backup_reason = NORMAL;
	//Define for Flag
	Flag_ScanNetwok = false;
	Flag_Cycle_Completed = true;
	Flag_ActiveNetwork = false;
	Flag_DeactiveNetwork = false;
	Flag_Network_Active = false;
	Flag_Network_Check_OK = false;
	Flag_Wait_Exit = false;
	Flag_GPS_Started = false;
	Flag_GPS_Stopped = false;
	Flag_Timer_GPS_Run = false;
	Flag_Device_Ready = false;
	Flag_WifiScan_Request = false;
	Flag_WifiCell_OK = false;
	Flag_Restart7070G_OK = false;
	Flag_CFUN_0_OK = false;
	Flag_CFUN_1_OK = false;
	Flag_Control_7070G_GPIO_OK = false;
	Flag_SelectNetwork_OK = false;
	Flag_NeedToProcess_GPS = false;
	Flag_MQTT_Stop_OK = false;

	//	VTAG_Configure.Period = 2;
	//	VTAG_Configure.Mode =  1;
	//	VTAG_Configure.Network = 3;
	//	VTAG_Configure.CC = 2;
}

static void Restart7070G_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	if(event == EVEN_OK)
	{
		//ESP_LOGW(TAG, "Restart 7070G OK\r\n");
		Flag_Restart7070G_OK = true;
		Flag_Cycle_Completed = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		//ESP_LOGE(TAG, "Restart 7070G timeout\r\n");
		//Restart7070G_Hard();
		Flag_Cycle_Completed = true;
	}
	else if(event == EVEN_ERROR)
	{
		Restart7070G_Soft();
	}
}
void Restart7070G_Soft(void)
{
	ATC_SendATCommand("AT+CFUN=1,1\r\n", "OK", 3000, 1, Restart7070G_Callback);
}
void Restart7070G_Hard(void)
{
	gpio_set_level(Sim7070G_PWRKEY, 1);
	vTaskDelay(pdMS_TO_TICKS(10000));
	gpio_set_level(Sim7070G_PWRKEY, 0);
}
void GPIO_7070G_Control()
{
	ATC_SendATCommand("AT+SGPIO=0,2,1,1\r\n", "OK", 3000, 3, GPIO_7070G_Control_Callback);
}
static void GPIO_7070G_Control_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	if(event == EVEN_OK)
	{
		//ESP_LOGW(TAG, "Control GPIO OK\r\n");
		Flag_Control_7070G_GPIO_OK = true;
		Flag_Wait_Exit = true;
		//ATC_SendATCommand("AT\r\n", "OK", 3000, 10, ATResponse_Callback);
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_Control_7070G_GPIO_OK = false;
		Flag_Wait_Exit = true;
		//ESP_LOGW(TAG, "Control GPIO timeout\r\n");;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_Control_7070G_GPIO_OK = false;
		Flag_Wait_Exit = true;
		//ESP_LOGW(TAG, "Control GPIO error\r\n");
	}
}
void PWRKEY_7070G_GPIO_Init()
{
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
}
void CheckSleepWakeUpCause(void)
{
	ProgramRun_Cause = esp_sleep_get_wakeup_cause();

	switch (ProgramRun_Cause)
	{
#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
	case ESP_SLEEP_WAKEUP_EXT0:
	{
		Flag_sms_receive = true;
		//ESP_LOGI(TAG,"have a sms in simcom");
		break;
	}
	case ESP_SLEEP_WAKEUP_EXT1:
	{
		uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
		if (wakeup_pin_mask != 0)
		{
			Ext1_Wakeup_Pin = __builtin_ffsll(wakeup_pin_mask) - 1;
			// Check charging state
			if(Ext1_Wakeup_Pin == CHARGE)
			{
				float batt_vol_prev = (float) (Calib_adc_factor*RTC_ADC*2.085924*3300/4095);
				printf("STUB VBAT =  %.1f mV\r\n", batt_vol_prev);
				uint32_t io_state = gpio_get_level(Ext1_Wakeup_Pin);
				// Charging
				if(io_state == 0)
				{
					Flag_charged = true;
					state = IN_CHARGING;
					Flag_FullBattery = false;
					printf("CASE 1:CHARGING\r\n");
				}
				// Full battery or charge disrupt
				else if(io_state == 1)
				{
					ESP32_GPIO_Output_Config();
					ulp_charging = 0;
					ulp_sample_counter = 0;
					vTaskDelay(3000 / RTOS_TICK_PERIOD_MS);
					float batt_vol_now = read_battery();
					printf("VBAT now =  %.1f mV\r\n",batt_vol_now);
					// If the battery voltage reach to a threshold then it is fully charged
					if(batt_vol_now >= THRESHOLD)
					{
						state = FULL_BATTERY;
						Flag_FullBattery = true;
						printf("CASE 2:BATTERY IS FULL\r\n");
					}
					// If the battery voltage not reach to a threshold then using other approach to check
					else
					{
						if(batt_vol_prev >= (batt_vol_now + VOFF_SET))
						{
							state = DISRUPTED;
							Flag_FullBattery = false;
							printf("CASE 3:DISRUPTED\r\n");
						}
						else
						{
							if(batt_vol_now > batt_vol_prev)
							{
								state = UNDEFINED;
								Flag_FullBattery = false;
								printf("CASE 4:UNDEFINED\r\n");
							}
							else
							{
								state = FULL_BATTERY;
								Flag_FullBattery = true;
								printf("CASE 2:BATTERY IS FULL\r\n");
							}
						}
					}
				}

			}
			//ESP_LOGE(TAG,"Wake up from GPIO %d\n", Ext1_Wakeup_Pin);
		}
		else
		{
			//ESP_LOGE(TAG,"Wake up from GPIO abc\n");
		}
		break;
	}
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
	case ESP_SLEEP_WAKEUP_GPIO: {
		uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
		if (wakeup_pin_mask != 0) {
			int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
			//ESP_LOGE(TAG,"Wake up from GPIO %d\n", pin);
		} else {
			//ESP_LOGE(TAG,"Wake up from GPIO\n");
		}
		break;
	}
#endif //SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
	case ESP_SLEEP_WAKEUP_TIMER:
	{
		//ESP_LOGE(TAG,"Wake up from timer. Time using deice: %ds\n", using_device_time_s);
		break;
	}
#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
	case ESP_SLEEP_WAKEUP_TOUCHPAD: {
		//ESP_LOGE(TAG,"Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
		break;
	}
#endif // CONFIG_EXAMPLE_TOUCH_WAKEUP
#ifdef CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
	case ESP_SLEEP_WAKEUP_ULP: {
		//ESP_LOGI(TAG,"Wake up from ULP\n");
		int16_t diff_high = (int16_t) ulp_data_read(3);
		int16_t diff_low = (int16_t) ulp_data_read(4);
		if (diff_high < 0) {
			//ESP_LOGE(TAG,"High temperature alarm was triggered\n");
		} else if (diff_low < 0) {
			//ESP_LOGE(TAG,"Low temperature alarm was triggered\n");
		} else {
			assert(false && "temperature has stayed within limits, but got ULP wakeup\n");
		}
		break;
	}
#endif // CONFIG_IDF_TARGET_ESP32
#endif // CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
	case ESP_SLEEP_WAKEUP_UNDEFINED:
	{
		//ESP_LOGE(TAG,"Not a deep sleep reset\n");

		char line_l[100] = {0};
		long dvts = 0;
		// Read device ID from flash
		memset(line_l, 0, strlen(line_l));
		readfromfile(base_path, "vtag_id.txt", line_l);
		strcpy(DeviceID_TW_Str, line_l);
		//ESP_LOGI(TAG, "Device ID: '%s'", DeviceID_TW_Str);

		// Read device pair status from flash
		memset(line_l, 0, strlen(line_l));
		readfromfile(base_path, "vtag_pr.txt", line_l);
		strcpy(Device_PairStatus, line_l);
		//ESP_LOGI(TAG, "Device pair status: '%s'", Device_PairStatus);

		//Kiá»ƒm tra thiáº¿t bá»‹ táº¯t má»�m hay cá»©ng
		memset(line_l, 0, strlen(line_l));
		readfromfile(base_path, "vtag_off_ts.txt", line_l);
		strcpy(Device_shutdown_ts, line_l);
		dvts =  atol(Device_shutdown_ts);
		//ESP_LOGI(TAG, "Device show down time: '%ld'", dvts);
		sprintf(Device_shutdown_ts, "%ld", dvts);
		writetofile(base_path, "vtag_off_ts.txt", "0");
		// Read calib pin factor from flash
		memset(line_l, 0, strlen(line_l));
		readfromfile(base_path, "vtag_pin.txt", line_l);
		if(strlen(line_l))
		{
			Calib_Pin_factor = atof(line_l);
		}
		//ESP_LOGI(TAG, "Calib Pin Factor: '%f'", Calib_Pin_factor);
		break;
	}
	default:
		break;
	}
}
void SetESP32Sleep(void)
{
	esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
}
//------------------------------------------------------------------------------------------------------------------------// Json
//TÃ¡ch chuá»—i Json Ä‘á»ƒ láº¥y timestamp tá»« server khi cÃ³ backup
void JSON_Analyze_backup(char* my_json_string, CFG* config)
{
	cJSON *root = cJSON_Parse(my_json_string);
	cJSON *current_element = NULL;
	cJSON_ArrayForEach(current_element, root)
	{
		if (current_element->string)
		{
			const char* string = current_element->string;
			if(strcmp(string, "T") == 0)
			{
				strncpy(config->Server_Timestamp, current_element->valuestring, 15);
				//ESP_LOGW(TAG,"Server timestamp: %s\r\n", config->Server_Timestamp);
				//				long Server_Timestamp_l = atol(config->Server_Timestamp);
				//				////ESP_LOGW(TAG,"Server timestamp: %ld\r\n", Server_Timestamp_l);
				//				struct timeval epoch = {Server_Timestamp_l, 0};
				//				struct timezone utc = {0,0};
				//				settimeofday(&epoch, &utc);
			}
		}
	}
	cJSON_Delete(root);
}
void JSON_Analyze_init_config(char* my_json_string, CFG* config)
{
	cJSON *root = cJSON_Parse(my_json_string);
	cJSON *current_element = NULL;
	cJSON_ArrayForEach(current_element, root)
	{
		if (current_element->string)
		{
			const char* string = current_element->string;
			if(strcmp(string, "Type") == 0)
			{
				strncpy(config->Type, current_element->valuestring, 5);
				//ESP_LOGW(TAG,"Type: %s\r\n", config->Type);
			}

			if(strcmp(string, "M") == 0)
			{
				if(current_element->valueint != 0)
				{
					config->Mode = current_element->valueint;
				}
				//ESP_LOGW(TAG,"Mode: %d\r\n", config->Mode);
			}
			if(strcmp(string, "P") == 0)
			{
				if(current_element->valueint != 0)
				{
					config->Period = current_element->valueint;
				}
				//ESP_LOGW(TAG,"Period: %d\r\n", config->Period);
			}
			if(strcmp(string, "CC") == 0)
			{
				if(current_element->valueint != 0)
				{
					config->CC = current_element->valueint;
				}
				//ESP_LOGW(TAG,"CC: %d\r\n", config->CC);
			}
			if(strcmp(string, "N") == 0)
			{
				if(current_element->valueint != 0)
				{
					config->Network = current_element->valueint;
				}
				//ESP_LOGW(TAG,"Network: %d\r\n", config->Network);
			}
			if(strcmp(string, "a") == 0)
			{
				config->Accuracy = current_element->valueint;
				//ESP_LOGW(TAG,"Accuracy: %d\r\n", config->Accuracy);
			}
			if(strcmp(string, "_ss") == 0)
			{
				config->_SS = current_element->valueint;
				//ESP_LOGW(TAG,"_SS: %d\r\n", config->_SS);
			}
			if(strcmp(string, "WM") == 0)
			{
				config->WM = current_element->valueint;
				//ESP_LOGW(TAG,"WM: %d\r\n", config->WM);
			}
			if(strcmp(string, "_lc") == 0)
			{
				config->_lc = current_element->valueint;
				//ESP_LOGW(TAG,"_lc: %d\r\n", config->_lc);
			}
			if(strcmp(string, "MA") == 0)
			{
				config->MA = current_element->valueint;
				//ESP_LOGW(TAG,"MA: %d\r\n", config->MA);
			}
		}
	}
	cJSON_Delete(root);
}
void JSON_Analyze(char* my_json_string, CFG* config)
{
	bool Flag_type_O = false;
	cJSON *root = cJSON_Parse(my_json_string);
	cJSON *current_element = NULL;
	cJSON_ArrayForEach(current_element, root)
	{
		if (current_element->string)
		{
			const char* string = current_element->string;
			if(strcmp(string, "Type") == 0)
			{
				strncpy(config->Type, current_element->valuestring, 5);
				//ESP_LOGW(TAG,"Type: %s\r\n", config->Type);
				if(!strcmp(config->Type,"O"))
				{
					Flag_type_O = true;
				}
			}
			if(Flag_type_O == false)
			{
				if(strcmp(string, "M") == 0)
				{
					if(current_element->valueint != 0)
					{
						config->Mode = current_element->valueint;
					}
					//ESP_LOGW(TAG,"Mode: %d\r\n", config->Mode);
				}
				if(strcmp(string, "P") == 0)
				{
					if(current_element->valueint != 0)
					{
						config->Period = current_element->valueint;
					}
					//ESP_LOGW(TAG,"Period: %d\r\n", config->Period);
				}
				if(strcmp(string, "CC") == 0)
				{
					if(current_element->valueint != 0)
					{
						config->CC = current_element->valueint;
					}
					//ESP_LOGW(TAG,"CC: %d\r\n", config->CC);
				}
				if(strcmp(string, "N") == 0)
				{
					if(current_element->valueint != 0)
					{
						config->Network = current_element->valueint;
					}
					//ESP_LOGW(TAG,"Network: %d\r\n", config->Network);
				}
				if(strcmp(string, "a") == 0)
				{
					config->Accuracy = current_element->valueint;
					//ESP_LOGW(TAG,"Accuracy: %d\r\n", config->Accuracy);
				}
				if(strcmp(string, "_ss") == 0)
				{
					config->_SS = current_element->valueint;
					//ESP_LOGW(TAG,"_SS: %d\r\n", config->_SS);
				}
				if(strcmp(string, "WM") == 0)
				{
					config->WM = current_element->valueint;
					//ESP_LOGW(TAG,"WM: %d\r\n", config->WM);
				}
				if(strcmp(string, "_lc") == 0)
				{
					config->_lc = current_element->valueint;
					//ESP_LOGW(TAG,"_lc: %d\r\n", config->_lc);
				}
				if(strcmp(string, "MA") == 0)
				{
					config->MA = current_element->valueint;
					//					//ESP_LOGW(TAG,"MA: %d\r\n", config->MA);
				}
				if(strcmp(string, "FLR") == 0)
				{
					config->FLR = current_element->valueint;
					//					//ESP_LOGW(TAG,"FlagLR: %d\r\n", config->FLR);
				}
				if(strcmp(string, "F2G") == 0)
				{
					config->F2G = current_element->valueint;
					//					//ESP_LOGW(TAG,"FlagLR: %d\r\n", config->FLR);
				}
			}
			if(strcmp(string, "T") == 0)
			{
				strncpy(config->Server_Timestamp, current_element->valuestring, 15);
				//ESP_LOGW(TAG,"Server timestamp: %s\r\n", config->Server_Timestamp);
				long Server_Timestamp_l = atol(config->Server_Timestamp);
				////ESP_LOGW(TAG,"Server timestamp: %ld\r\n", Server_Timestamp_l);

				struct timeval epoch = {Server_Timestamp_l, 0};
				struct timezone utc = {0,0};
				settimeofday(&epoch, &utc);
			}
		}
	}
	cJSON_Delete(root);
}

static int json_add_obj(cJSON *parent, const char *str, cJSON *item)
{
	cJSON_AddItemToObject(parent, str, item);

	return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item)
{
	cJSON *json_str;

	json_str = cJSON_CreateString(item);
	if (json_str == NULL)
	{
		//return -ENOMEM;
		return 0;
	}

	return json_add_obj(parent, str, json_str);
}

static int json_add_num(cJSON *parent, const char *str, const double item)
{
	cJSON *json_num;

	json_num = cJSON_CreateNumber(item);
	if (json_num == NULL)
	{
		return 0;
		//return -ENOMEM;
	}

	return json_add_obj(parent, str, json_num);
}

static int json_add_ap(cJSON *aps, const char * bssid, int rssi)
{
	int err = 0;
	cJSON *ap_obj = cJSON_CreateObject();
	err |= json_add_str(ap_obj, "M", bssid);
	err |= json_add_num(ap_obj, "S", rssi);
	cJSON_AddItemToArray(aps, ap_obj);
	return err;
}
//------------------------------------------------------------------------------------------------------------------------// Wifi scan
typedef struct {
	uint8_t ssid[33];                     /**< SSID of AP */
	uint8_t bssid[6];
	uint8_t	macserial[13];
	int  rssi;                         /**< signal strength of AP */
} wifi_ap_strength;

RTC_DATA_ATTR wifi_ap_strength wifi_ap_pre[20];
wifi_ap_strength wifi_ap_cur[20];
bool wifi_detect_motion(wifi_ap_strength WiFi_Previous[], wifi_ap_strength WiFi_Current[])
{
#define W1  1
#define W2  1
#define W3  1
#define W4 	1
#define CNT_THR	1
	Flag_wifi_detected = true;
	Flag_mess_sended_wd = false;
	bool Ro1 = false, Ro2 = false, Ro3 = false, Ro5 = false;
	float  Check_Count_1 = 0, Check_Count_2 = 0, Check_Count_3 = 0, Check_Count_5 = 0, Check_total_wifi = 0, Check_weak_wifi = 0, AP_Weak_count_pre = 0, AP_Weak_count_cur = 0;
	uint32_t AP_count_cur = 0;

	AP_count_cur = count_list;
	//ESP_LOGW("WIFI", "------------------> WiFi Previous <------------------");
	for (int i = 0; i < AP_count_pre; i++){
		if (WiFi_Previous[i].rssi == 0) continue;
		ESP_LOGI("WIFI", "%s               , %02X:%02X:%02X:%02X:%02X:%02X, %d", WiFi_Previous[i].ssid, WiFi_Previous[i].bssid[0], WiFi_Previous[i].bssid[1], WiFi_Previous[i].bssid[2],\
				WiFi_Previous[i].bssid[3], WiFi_Previous[i].bssid[4], WiFi_Previous[i].bssid[5], WiFi_Previous[i].rssi);
		sprintf(MAC_Serial_Pre + strlen(MAC_Serial_Pre), "%s", WiFi_Previous[i].macserial);
	}

	//ESP_LOGW("WIFI", "------------------> WiFi Current <------------------");
	for (int i = 0; i < AP_count_cur; i++){
		ESP_LOGI("WIFI", "%s               , %02X:%02X:%02X:%02X:%02X:%02X, %d", WiFi_Current[i].ssid, WiFi_Current[i].bssid[0], WiFi_Current[i].bssid[1], WiFi_Current[i].bssid[2],\
				WiFi_Current[i].bssid[3], WiFi_Current[i].bssid[4], WiFi_Current[i].bssid[5], WiFi_Current[i].rssi);
		sprintf(MAC_Serial_Cur + strlen(MAC_Serial_Cur), "%s", WiFi_Current[i].macserial);
	}

	//ESP_LOGW("ROUND 1", "Check new strong WiFi appear");
	//	for (int i = 0; i < AP_count_pre; i++)
	//	{
	//		sprintf(MAC_Serial_Pre + strlen(MAC_Serial_Pre), "%s", WiFi_Previous[i].macserial);
	//	}
	//ESP_LOGI("ROUND 1", "MAC_Serial_Pre: %s", MAC_Serial_Pre);
	for (int i = 0; i < AP_count_cur; i++){
		if (abs(WiFi_Current[i].rssi) < SS_THR && WiFi_Current[i].rssi != 0)
		{
			if (!strstr(MAC_Serial_Pre, (char*) WiFi_Current[i].macserial))
			{
				Check_Count_1 = (Check_Count_1 + 1) * W1;
			}
		}
	}
	//ESP_LOGE("ROUND 1", "Check_Count: %g", Check_Count_1);
	if(Check_Count_1 > 1)
	{
		wd_reason = wd_reason * 10 +STR_PRE;
		Ro1 = true;
	}

	//ESP_LOGW("ROUND 2", "Check new strong WiFi disappear");
	//	for (int i = 0; i < AP_count_cur; i++)
	//	{
	//		sprintf(MAC_Serial_Cur + strlen(MAC_Serial_Cur), "%s", WiFi_Current[i].macserial);
	//	}
	//ESP_LOGI("ROUND 2", "MAC_Serial_Cur: %s", MAC_Serial_Cur);
	for (int i = 0; i < AP_count_pre; i++)
	{
		if (abs(WiFi_Previous[i].rssi) < SS_THR && WiFi_Previous[i].rssi != 0)
		{
			if (!strstr(MAC_Serial_Cur, (char*) WiFi_Previous[i].macserial))
			{
				Check_Count_2 = (Check_Count_2 + 1) * W2;
			}
		}
	}
	//ESP_LOGE("ROUND 2", "Check_Count: %g", Check_Count_2);
	if(Check_Count_2 > 1)
	{
		wd_reason = wd_reason * 10 + STR_CUR;
		Ro2 = true;
	}

	//ESP_LOGW("ROUND 3", "Check WiFi change rssi (15dbm)");
	for (int i = 0; i < AP_count_cur; i++)
	{
		for (int j = 0; j < AP_count_pre; j++)
		{
			if (strstr((char*) WiFi_Current[i].macserial,(char*) WiFi_Previous[j].macserial) && abs(WiFi_Current[i].rssi) != 0 && abs(WiFi_Previous[j].rssi) != 0)
			{
				if (abs(WiFi_Current[i].rssi - WiFi_Previous[j].rssi) > 15)
				{
					printf("\n %s", WiFi_Current[i].macserial);
					Check_Count_3 = (Check_Count_3 + 1) * W3;
					break;
				}
			}
		}
	}
	//ESP_LOGE("ROUND 3", "Check_Count: %g", Check_Count_3);
	if (Check_Count_3/AP_count_pre > 0.3)
	{
		wd_reason = wd_reason * 10 + CHANGE_RSSI;
		Ro3 = true;
	}

	//	//ESP_LOGW("ROUND 4", "Check weak wifi disappear");
	//	for (int i = 0; i < AP_count_pre; i++)
	//	{
	//		if(abs(WiFi_Previous[i].rssi) >= SS_THR)
	//		{
	//			sprintf(MAC_Weak_Serial_Pre + strlen(MAC_Weak_Serial_Pre), "%s", WiFi_Previous[i].macserial);
	//			AP_Weak_count_pre = AP_Weak_count_pre + 1;
	//		}
	//	}
	//	for (int i = 0; i < AP_count_cur; i++)
	//	{
	//		if(abs(WiFi_Current[i].rssi) >= SS_THR)
	//		{
	//			AP_Weak_count_cur = AP_Weak_count_cur + 1;
	//
	//			if(strstr(MAC_Weak_Serial_Pre, (char*)WiFi_Current[i].macserial) != 0)
	//			{
	//				Check_weak_wifi = Check_weak_wifi + 1.0;
	//			}
	//		}
	//	}
	//	if(AP_Weak_count_pre != 0)
	//	{
	//
	//		if(Check_weak_wifi / AP_Weak_count_pre <= 0.35)
	//		{
	//			Check_Count_4 = (Check_Count_4 + 1) * W4;
	//			Ro4 = true;
	//		}
	//	}
	//	else if(AP_Weak_count_pre == 0 && AP_Weak_count_cur !=0 )
	//	{
	//		Check_Count_4 = (Check_Count_4 + 1) * W4;
	//		Ro4 = true;
	//
	//	}
	//
	//	//ESP_LOGE("ROUND 4", "Check_Count: %g", Check_Count_4);

	//ESP_LOGW("ROUND 5", "Check total wifi disappear");
	for (int i = 0; i < AP_count_cur; i++)
	{
		if(strstr(MAC_Serial_Pre, (char*) WiFi_Current[i].macserial) != 0)
		{
			Check_total_wifi = Check_total_wifi + 1.0;
		}
	}
	if(AP_count_pre != 0)
	{
		if(Check_total_wifi / AP_count_pre <= 0.3)
		{
			Check_Count_5 = (Check_Count_5 + 1);
			wd_reason = wd_reason * 10 + WIFI_DISAPPEARE;
			Ro5 = true;
		}
	}
	else if(AP_count_pre == 0 && AP_count_cur !=0 )
	{
		Check_Count_5 = (Check_Count_5 + 1);
		wd_reason = wd_reason * 10 + WIFI_DISAPPEARE;
		Ro5 = true;
	}
	//ESP_LOGE("ROUND 5", "Check_Count: %g", Check_Count_5);

	memset(wifi_ap_pre, 0, sizeof(wifi_ap_pre));
	for(int i = 0; i < AP_count_cur; i++)
	{
		wifi_ap_pre[i] = wifi_ap_cur[i];
	}
	memset(wifi_ap_cur, 0, sizeof(wifi_ap_cur));
	AP_count_pre = AP_count_cur;
	if (Ro1 || Ro2 || Ro3 || Ro5)
	{
		Flag_mess_sended_wd = true;
		return true;
	}
	else return false;
}
static void wifi_init(void)
{
	if(!strlen(Wifi_Buffer))
	{
		Flag_wifi_scanning = true;
		Charge_Delay_Counter = 0;
		Flag_wifi_init = false;
		//Flag_WifiScan_End = false;
		ESP32_Clock_Config(80, 80, false);

		esp_netif_init();
		esp_event_loop_create_default();
		//	esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
		//	assert(sta_netif);
		esp_netif_create_default_wifi_sta();
		Flag_wifi_init = true;
	}
}
static void wifi_scan(void)
{
	int n = 0;
	char MAC_Serial_cn2[150] = {0};
	char MAC_Serial_cn1[150] = {0};
	uint8_t WFT2_MacString[13];
	uint8_t WFT2_bssid[6];
	if(!strlen(Wifi_Buffer))
	{
		count_list = 0;
		if(VTAG_Configure.WM == 1 && esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED && VTAG_Configure.MA == 0 && Flag_wifi_loop)
		{
			count_loop = 2;
		}
		else
		{
			count_loop = 1;
		}
		acc_power_down();
		vTaskDelay(1000/RTOS_TICK_PERIOD_MS);
		ESP32_Clock_Config(80, 80, false);
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		esp_wifi_init(&cfg);
		//Flag_WifiScan_End = false;
		char *buffer = NULL;

		int err = 0;
		cJSON *root_obj = cJSON_CreateObject();
		cJSON *aps_obj = cJSON_CreateArray();

		if (root_obj == NULL)
		{
			//ESP_LOGE(TAG, "Create JSON object fail!\r\n");
		}
		uint16_t number = DEFAULT_SCAN_LIST_SIZE;
		wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
		//ESP_LOGE(TAG, "size of ap_info%d\r\n", sizeof(ap_info));
		//memset(ap_info, 0, sizeof(ap_info));
		esp_wifi_set_mode(WIFI_MODE_STA);

		while(n < count_loop)
		{
			esp_wifi_start();
			esp_wifi_scan_start(NULL, true);
			esp_wifi_scan_get_ap_records(&number, ap_info);
			esp_wifi_scan_get_ap_num(&ap_count);
			//ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
			for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
				//ESP_LOGI(TAG, "SSID: %s", ap_info[i].ssid);

				//ESP_LOGI(TAG, "BSSID: %02X:%02X:%02X:%02X:%02X:%02X", ap_info[i].bssid[0], ap_info[i].bssid[1], ap_info[i].bssid[2],\=
						//ap_info[i].bssid[3], ap_info[i].bssid[4], ap_info[i].bssid[5]);
				//ESP_LOGI(TAG, "RSSI: %d", ap_info[i].rssi);
				if(n == 0)
				{
					if(abs(ap_info[i].rssi) != 0)
					{
						memcpy(wifi_ap_cur[count_list].ssid, ap_info[i].ssid, sizeof(ap_info[i].ssid));
						memcpy(wifi_ap_cur[count_list].bssid, ap_info[i].bssid, sizeof(ap_info[i].bssid));
						sprintf((char*)wifi_ap_cur[count_list].macserial, "%02X%02X%02X%02X%02X%02X", wifi_ap_cur[count_list].bssid[0], wifi_ap_cur[count_list].bssid[1], wifi_ap_cur[count_list].bssid[2],\
								wifi_ap_cur[count_list].bssid[3], wifi_ap_cur[count_list].bssid[4], wifi_ap_cur[count_list].bssid[5]);
						wifi_ap_cur[count_list].rssi = ap_info[i].rssi;
						sprintf(MAC_Serial_cn1 + strlen(MAC_Serial_cn1), "%s", wifi_ap_cur[count_list].macserial);
						count_list++;
					}
				}
				else
				{
					if(abs(ap_info[i].rssi) != 0)
					{
						memcpy(WFT2_bssid, ap_info[i].bssid, sizeof(ap_info[i].bssid));
						sprintf((char*)WFT2_MacString, "%02X%02X%02X%02X%02X%02X", WFT2_bssid[0], WFT2_bssid[1], WFT2_bssid[2],\
								WFT2_bssid[3], WFT2_bssid[4], WFT2_bssid[5]);
						sprintf(MAC_Serial_cn2 + strlen(MAC_Serial_cn2), "%s", WFT2_MacString);
						if (!strstr(MAC_Serial_cn1, (char*) WFT2_MacString))
						{
							memcpy(wifi_ap_cur[count_list].ssid, ap_info[i].ssid, sizeof(ap_info[i].ssid));
							memcpy(wifi_ap_cur[count_list].bssid, ap_info[i].bssid, sizeof(ap_info[i].bssid));
							strcpy((char*)wifi_ap_cur[count_list].macserial, (char*)WFT2_MacString);
							wifi_ap_cur[count_list].rssi = ap_info[i].rssi;
							count_list++;
						}
						memset(WFT2_MacString, 0, sizeof(WFT2_MacString));
					}
				}
			}
			ESP_LOGI(TAG, "%s", MAC_Serial_cn1);
			ESP_LOGI(TAG, "%s", MAC_Serial_cn2);
			esp_wifi_scan_stop();
			esp_wifi_stop();
			n++;
			vTaskDelay(300/portTICK_PERIOD_MS);
		}
		esp_wifi_deinit();
		if(count_list > 0)
		{
			//			err |= json_add_num(root_obj, "A", ap_count);
			for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < count_list); i++)
			{
				char BSSID_Buf[20];
				sprintf(BSSID_Buf, "%x:%x:%x:%x:%x:%x", wifi_ap_cur[i].bssid[0], wifi_ap_cur[i].bssid[1], wifi_ap_cur[i].bssid[2], wifi_ap_cur[i].bssid[3], wifi_ap_cur[i].bssid[4], wifi_ap_cur[i].bssid[5]);
				err |= json_add_ap(aps_obj, BSSID_Buf, wifi_ap_cur[i].rssi);
			}
			err |= json_add_obj(root_obj, "W", aps_obj);
			if (err)
			{
				//ESP_LOGE(TAG,"There are some error when making JSON object!\r\n");
			}
			else
			{
				buffer = cJSON_PrintUnformatted(root_obj);
				cJSON_Delete(root_obj);
			}
			if (buffer == NULL)
			{
				//ESP_LOGE(TAG,"There are no data to send!\r\n");
			}
			else
			{
				//ESP_LOGE(TAG, "Print wifi buffer \r\n");
				//ESP_LOGW(TAG,"%s",buffer);
				//ESP_LOGE(TAG, "Allocate memory for wifi buffer \r\n");
				//ESP_LOGE(TAG, "Copy wifi buffer \r\n");
				for(int i = 1; i < strlen(buffer) - 1; i++)
				{
					Wifi_Buffer[i-1] = buffer[i];
				}
			}
		}
		else
		{
			memset(Wifi_Buffer, 0, sizeof(Wifi_Buffer));
		}
		ESP32_Clock_Config(20, 20, false);
		Flag_wifi_scanning = false;
		vTaskDelay(1000/RTOS_TICK_PERIOD_MS);
		if(VTAG_Configure.MA != 1)
		{
			acc_power_up();
		}
	}
}
//------------------------------------------------------------------------------------------------------------------------// Timer
//------------------------------------------------------------------------------------------------------------------------// timer group 0, ISR
void IRAM_ATTR timer_group0_isr(void *para)
{
	int timer_idx_p = (int) para;
	uint32_t intr_status = TIMERG0.int_st_timers.val;
	if((intr_status & BIT(timer_idx_p)) && timer_idx_p == TIMER_0)
	{
		TIMERG0.hw_timer[timer_idx_p].update = 1;
		TIMERG0.int_clr_timers.t0 = 1;
		TIMERG0.hw_timer[timer_idx_p].config.alarm_en = 1;
		switch (GPS_Select)
		{
		case GPS_Indirect:
		{
			Flag_Timer_GPS_Run = true;
			break;
		}
		default:
			break;
		}
	}
	else if((intr_status & BIT(timer_idx_p)) && timer_idx_p == TIMER_1)
	{
		TIMERG0.hw_timer[timer_idx_p].update = 1;
		TIMERG0.int_clr_timers.t1 = 1;
		TIMERG0.hw_timer[timer_idx_p].config.alarm_en = 1;

		TrackingRuntime++;
	}
}
//------------------------------------------------------------------------------------------------------------------------//
static void tg0_timer0_init()
{
	timer_config_t config;
	config.alarm_en = 1;
	config.auto_reload = 1;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_SEL;
	config.counter_en = TIMER_PAUSE;
	/*Configure timer*/
	timer_init(timer_group, timer_idx, &config);
	/*Stop timer counter*/
	timer_pause(timer_group, timer_idx);
	/*Load counter value */
	timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
	/*Set alarm value*/
	timer_set_alarm_value(timer_group, timer_idx, (TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
	/*Enable timer interrupt*/
	timer_enable_intr(timer_group, timer_idx);
	/*Set ISR handler*/
	timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
	/*Start timer counter*/
	timer_start(timer_group, timer_idx);
	/*Stop timer counter*/
	timer_pause(timer_group, timer_idx);
}

//------------------------------------------------------------------------------------------------------------------------//
static void TrackingPeriod_Timer_Init()
{
	timer_config_t config;
	config.alarm_en = 1;
	config.auto_reload = 1;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_SEL;
	config.counter_en = TIMER_PAUSE;
	/*Configure timer*/
	timer_init(timer_group, TrackingPeriod_timer_idx, &config);
	/*Stop timer counter*/
	timer_pause(timer_group, TrackingPeriod_timer_idx);
	/*Load counter value */
	timer_set_counter_value(timer_group, TrackingPeriod_timer_idx, 0x00000000ULL);
	/*Set alarm value*/
	timer_set_alarm_value(timer_group, TrackingPeriod_timer_idx, (TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
	/*Enable timer interrupt*/
	timer_enable_intr(timer_group, TrackingPeriod_timer_idx);
	/*Set ISR handler*/
	timer_isr_register(timer_group, TrackingPeriod_timer_idx, timer_group0_isr, (void*) TrackingPeriod_timer_idx, ESP_INTR_FLAG_IRAM, NULL);
	/*Start timer counter*/
	timer_start(timer_group, TrackingPeriod_timer_idx);
	/*Stop timer counter*/
	timer_pause(timer_group, TrackingPeriod_timer_idx);
}
//------------------------------------------------------------------------------------------------------------------------// GPS processing
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// GPS process callback
void GPS_Process_Callback(char *ATC_Str_Buffer)
{
	char Buffer[100];
	strlcpy(Buffer, ATC_Str_Buffer, 100);
	/* Init and process GPS data */
	latitude = longitude = speed_kph = heading = altitude = year = month = day = hour = minute = sec = 0;
	GPS_Fix_Status = GPS_Decode(Buffer, &latitude, &longitude, &speed_kph, &heading, &altitude, &year, &month, &day, &hour, &minute, &sec);

	if(GPS_Fix_Status == 1)
	{
		timer_pause(timer_group, timer_idx);
		ATC_SendATCommand("AT+CGNSPWR=0\r\n", "OK", 1000, 2, StopGPS_Callback);
	}
}
//------------------------------------------------------------------------------------------------------------------------// GPS string format convert
void GPS_Position_Time_Str_Convert(char *str, uint8_t bat_lev, float accuracy, float lat, float lon, float alt, uint16_t year_t, uint8_t month_t, uint8_t date_t, uint8_t hour_t, uint8_t min_t, uint8_t sec_t)
{
	int interLat, fracLat, interLon, fracLon, interAlt, fracAlt;
	interLat = (int)lat;
	fracLat = (lat - interLat)*1000000;
	interLon = (int)lon;
	fracLon = (lon - interLon)*1000000;
	interAlt = (int)alt;
	fracAlt = (lon - interAlt)*1000000;

	char Network_Type_Str[10];
	if (Network_Type == NB_IoT)
	{
		sprintf(Network_Type_Str, "nb");
	}
	else if(Network_Type == GSM)
	{
		sprintf(Network_Type_Str, "2g");
	}
	//sync time esp with gps time
	//	long TS = convert_date_to_epoch(date_t, month_t, year_t, hour_t, min_t, sec_t);
	//
	//	struct timeval epoch = {TS, 0};
	//	struct timezone utc = {0,0};
	//	settimeofday(&epoch, &utc);

	//	sprintf(str, "{\"Type\":\"DPOS\",\"V\":\"%s\",\"ss\":%d,\"r\":%d,\"B\":%d,\"Cn\":\"%s\",\"Acc\":%f,\"lat\":%d.%06d,\"lon\":%d.%06d,\"T\":%ld}", VTAG_Vesion, VTAG_NetworkSignal.RSRP, VTAG_NetworkSignal.RSRQ, bat_lev, Network_Type_Str, accuracy, interLat, fracLat, interLon, fracLon, TS);

	//Use time from esp32 instead of gps time
	if(Network_Type == GSM)
	{
		sprintf(str, "{\"Type\":\"DPOS\",\"_Type\":\"GPS\",\"V\":\"%s\",\"ss\":%d,\"r\":%d,\"B\":%d,\"Cn\":\"%s\",\"Acc\":%f,\"lat\":%d.%06d,\"lon\":%d.%06d,\"T\":%lld,\"RR\":%d%d}", VTAG_Vesion, RSSI, VTAG_NetworkSignal.RSRQ, bat_lev, Network_Type_Str, accuracy, interLat, fracLat, interLon, fracLon, VTAG_DeviceParameter.Device_Timestamp, Reboot_reason, Backup_reason);
	}
	else
	{
		sprintf(str, "{\"Type\":\"DPOS\",\"_Type\":\"GPS\",\"V\":\"%s\",\"ss\":%d,\"r\":%d,\"B\":%d,\"Cn\":\"%s\",\"Acc\":%f,\"lat\":%d.%06d,\"lon\":%d.%06d,\"T\":%lld,\"RR\":%d%d}", VTAG_Vesion, VTAG_NetworkSignal.RSRP, VTAG_NetworkSignal.RSRQ, bat_lev, Network_Type_Str, accuracy, interLat, fracLat, interLon, fracLon, VTAG_DeviceParameter.Device_Timestamp, Reboot_reason, Backup_reason);
	}
}
//------------------------------------------------------------------------------------------------------------------------// GPS decode
uint8_t GPS_Decode(char *buffer, float *lat, float *lon, float *speed_kph, float *heading, float *altitude,
		uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	char gpsbuffer[120];
	GPS_Fix_Status = 0;
	strlcpy(gpsbuffer, buffer, 120);

	// skip GPS run status
	char *tok = strtok(gpsbuffer, ",");
	if (! tok) return false;

	// skip fix status
	tok = strtok(NULL, ",");
	if (! tok)
	{
		GPS_Fix_Status = 0;
		return false;
	}
	else
	{
		GPS_Fix_Status = atoi(tok);
	}

	if(GPS_Fix_Status == 1)
	{
		// only grab date and time if needed
		if ((year != NULL) && (month != NULL) && (day != NULL) && (hour != NULL) && (min != NULL) && (sec != NULL))
		{
			char *date = strtok(NULL, ",");
			if (! date) return false;

			// Seconds
			char *ptr = date + 12;
			*sec = atof(ptr);

			// Minutes
			ptr[0] = 0;
			ptr = date + 10;
			*min = atoi(ptr);

			// Hours
			ptr[0] = 0;
			ptr = date + 8;
			*hour = atoi(ptr);

			// Day
			ptr[0] = 0;
			ptr = date + 6;
			*day = atoi(ptr);

			// Month
			ptr[0] = 0;
			ptr = date + 4;
			*month = atoi(ptr);

			// Year
			ptr[0] = 0;
			ptr = date;
			*year = atoi(ptr);
		}
		else
		{
			// skip date
			tok = strtok(NULL, ",");
			if (! tok) return false;
		}

		// grab the latitude
		char *latp = strtok(NULL, ",");
		if (! latp) return false;

		// grab longitude
		char *longp = strtok(NULL, ",");
		if (! longp) return false;

		*lat = atof(latp);
		*lon = atof(longp);

		// only grab altitude if needed
		if (altitude != NULL)
		{
			// grab altitude
			char *altp = strtok(NULL, ",");
			if (! altp) return false;

			*altitude = atof(altp);
		}
		// only grab speed if needed
		if (speed_kph != NULL) {
			// grab the speed in km/h
			char *speedp = strtok(NULL, ",");
			if (! speedp) return false;

			*speed_kph = atof(speedp);
		}
		// only grab heading if needed
		if (heading != NULL)
		{

			// grab the speed in knots
			char *coursep = strtok(NULL, ",");
			if (! coursep) return false;

			*heading = atof(coursep);
		}
	}
	return GPS_Fix_Status;
}
//------------------------------------------------------------------------------------------------------------------------// GPS
void GPS_Read(void)
{
	//Led bÃƒÂ¡o thiÃ¡ÂºÂ¿t bÃ¡Â»â€¹ Ã„â€˜ang hoÃ¡ÂºÂ¡t Ã„â€˜Ã¡Â»â„¢ng
	if(GPS_Fix_Status != 1)
	{
		ATC_SendATCommand("AT+CGNSINF\r\n","OK",1000,2,ScanGPS_Callback);
	}
	else
	{
		if(Flag_GPS_Started == true)
		{
			//ESP_LOGW(TAG, "GPS Time to First Fix: %d s\r\n", GPS_Scan_Counter);
		}
	}
}
//------------------------------------------------------------------------------------------------------------------------// Start GPS callback
static void StartGPS_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)// NÃ¡ÂºÂ¿u Ã„â€˜ÃƒÂ£ scan Ã„â€˜Ã†Â°Ã¡Â»Â£c network
	{
		Flag_GPS_Started = true;
		//ESP_LOGW(TAG, "GPS start ok\r\n");
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		//ESP_LOGW(TAG, "GPS start timeout\r\n");
		Flag_GPS_Started = false;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		//ESP_LOGW(TAG, "GPS start error\r\n");
		Flag_GPS_Started = false;
		Flag_Wait_Exit = true;
	}
}
//------------------------------------------------------------------------------------------------------------------------// Scan GPS callback
static void ScanGPS_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)//
	{
		GPS_Scan_Counter++;
		if(GPS_Scan_Counter >= GPS_Scan_Number)//50
		{
			//ESP_LOGE(TAG, "\r\nGPS Scan Counter = %d --> GPS Tracking Timeout --> Stop tracking GPS\r\n", GPS_Scan_Counter);
			GPS_Scan_Counter = 0;
			timer_pause(timer_group, timer_idx);
			ATC_SendATCommand("AT+CGNSPWR=0\r\n", "OK", 1000, 2, StopGPS_Callback);
		}
		else if(GPS_Scan_Counter < GPS_Scan_Number)
		{
			//ESP_LOGW(TAG, "GPS scan time: %d s\r\n", GPS_Scan_Counter * GPS_Scan_Period);
			if(strstr(ResponseBuffer, "+CGNSINF:"))
			{
				GPS_Process_Callback(ResponseBuffer);
			}
		}
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		//ESP_LOGW(TAG, "Timeout\r\n");
		//Flag_GPS_Started = false;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_Wait_Exit = true;
		Restart7070G_Soft();
	}
}
//------------------------------------------------------------------------------------------------------------------------// Stop GPS callback
static void StopGPS_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)//
	{
		//ESP_LOGW(TAG, "GPS stop ok\r\n");
		Flag_GPS_Stopped = true;
		Flag_Wait_Exit = true;
		Flag_GPS_Started = false;
	}
	else if(event == EVEN_TIMEOUT)
	{
		//ESP_LOGE(TAG, "Stop GSPS timeout\r\n");
		Flag_GPS_Stopped = true;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		//ESP_LOGE(TAG, "Stop GPS error\r\n");
		Flag_GPS_Stopped = true;
		Flag_Wait_Exit = true;
	}
}
//------------------------------------------------------------------------------------------------------------------------// Network
//------------------------------------------------------------------------------------------------------------------------// Check network callback
static void SelectNB_GSM_Network_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		Flag_SelectNetwork_OK = true;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_SelectNetwork_OK = false;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_SelectNetwork_OK = false;
		Flag_Wait_Exit = true;
	}
}
void SelectNB_GSM_Network(uint8_t network)
{
	if(network == NB_IoT)
	{
		ATC_SendATCommand("AT+CNMP=38\r\n", "OK", 1000, 3, SelectNB_GSM_Network_Callback);
	}
	else if (network == GSM)
	{
		ATC_SendATCommand("AT+CNMP=13\r\n", "OK", 1000, 3, SelectNB_GSM_Network_Callback);
	}
	else if (network == Both_NB_GSM)
	{
		ATC_SendATCommand("AT+CNMP=51\r\n", "OK", 1000, 3, SelectNB_GSM_Network_Callback);
	}
	else if(network == 2)
	{
		ATC_SendATCommand("AT+CNMP=2\r\n", "OK", 1000, 3, SelectNB_GSM_Network_Callback);
	}
}
static void SetCFUN_0_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)// NÃ¡ÂºÂ¿u Ã„â€˜ÃƒÂ£ scan Ã„â€˜Ã†Â°Ã¡Â»Â£c network
	{
		Flag_CFUN_0_OK = true;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_CFUN_0_OK = false;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_CFUN_0_OK = false;
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
	}

}
static void SetCFUN_1_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;

	if(event == EVEN_OK)// NÃ¡ÂºÂ¿u Ã„â€˜ÃƒÂ£ scan Ã„â€˜Ã†Â°Ã¡Â»Â£c network
	{
		Flag_CFUN_1_OK = true;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_CFUN_1_OK = false;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_CFUN_1_OK = false;
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
	}

}
static void CheckNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)// NÃ¡ÂºÂ¿u Ã„â€˜ÃƒÂ£ scan Ã„â€˜Ã†Â°Ã¡Â»Â£c network
	{
		Flag_Network_Check_OK = true;
		if(strstr(ResponseBuffer,"0,1"))// NÃ¡ÂºÂ¿u mÃ¡ÂºÂ¡ng Ã„â€˜ÃƒÂ£ active
		{
			//ESP_LOGW(TAG, "Network is activated\r\n");
			Flag_Network_Active = true;
		}
		else// NÃ¡ÂºÂ¿u mÃ¡ÂºÂ¡ng chÃ†Â°a active
		{
			//ESP_LOGW(TAG, "Network is deactivated\r\n");
			Flag_Network_Active = false;
		}
		Flag_Wait_Exit = true;

	}
	else if(event == EVEN_TIMEOUT)
	{
		//ESP_LOGW(TAG, "Timeout\r\n");
		Flag_Network_Active = false;
		Flag_Wait_Exit = true;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
	}
}
//------------------------------------------------------------------------------------------------------------------------// Active network callback
static void ActiveNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		//ESP_LOGW(TAG, "Active network OK\r\n");
		Flag_ActiveNetwork = true;
		Flag_Wait_Exit = true;

		MQTT_Connect_Callback(EVEN_OK, "");
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_Cycle_Completed = true;
		Flag_ActiveNetwork = false;
		Flag_Wait_Exit = true;
		//ESP_LOGE(TAG, "Active network error\r\n");
		// Backup location data
	}
	else if(event == EVEN_ERROR)
	{
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
		//ESP_LOGE(TAG, "Active network error\r\n");
		// Backup location data
	}
}
//------------------------------------------------------------------------------------------------------------------------// Deactive network callback
static void DeactiveNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)
	{
		//ESP_LOGW(TAG, "Deactive network OK\r\n");
		Flag_DeactiveNetwork = true;
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
		//ESP_LOGE(TAG, "Enter to deep sleep mode\r\n");
		TurnOn7070G();
		vTaskDelay(pdMS_TO_TICKS(3000));
		esp_deep_sleep_start();
	}
	else if(event == EVEN_TIMEOUT)
	{
		//ESP_LOGW(TAG, "Deactive network timeout\r\n");
		Flag_DeactiveNetwork = false;
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
	}
	else if(event == EVEN_ERROR)
	{
		Flag_Wait_Exit = true;
		Flag_Cycle_Completed = true;
	}
}
//------------------------------------------------------------------------------------------------------------------------// Scan network callback
static void ScanNetwork_Callback(SIMCOM_ResponseEvent_t event, void *ResponseBuffer)
{
	AT_RX_event = event;
	if(event == EVEN_OK)// NÃ¡ÂºÂ¿u Ã„â€˜ÃƒÂ£ scan Ã„â€˜Ã†Â°Ã¡Â»Â£c network
	{
		//ESP_LOGW(TAG, "Scan network OK\r\n");
		Flag_ScanNetwok = true;
		Flag_Wait_Exit = true;
		strcpy(CPSI_Str, ResponseBuffer);
	}
	else if(event == EVEN_TIMEOUT)
	{
		Flag_ScanNetwok = false;
		Flag_Wait_Exit = true;
		//ESP_LOGW(TAG, "Scan network timeout\r\n");
	}
	else if(event == EVEN_ERROR)
	{
		Flag_ScanNetwok = false;
		Flag_Wait_Exit = true;
		//ESP_LOGW(TAG, "Scan network error\r\n");
	}
}
//------------------------------------------------------------------------------------------------------------------------// Tasks
//void esp_task_wdt_isr_user_handler(void)
//{
//	/* restart firmware */
////	TurnOn7070G();
////	//ESP_LOGW(TAG, "WDT\r\n");
//	esp_restart();
//}
//------------------------------------------------------------------------------------------------------------------------//
bool MQTT_SubMessage_NeedToProcess = false;
bool MQTT_Message_NeedToProcess = false;
void mqtt_submessage_processing_task(void *arg)
{
	//xSemaphoreTake(sync_mqtt_submessage_processing_task, portMAX_DELAY);
	while (1)
	{
		if(MQTT_SubMessage_NeedToProcess == true)
		{
			Flag_count_nb_backup = 0;
			Flag_use_2g = false;
			Fail_network_counter = 0;
			Flag_reboot_7070 = false;
			VTAG_Configure_temp = VTAG_Configure;
			if(ACK_Succeed[WAIT_ACK] == MESS_UNPAIR || ACK_Succeed[WAIT_ACK] == MESS_PAIR || ACK_Succeed[WAIT_ACK] == 3 || ACK_Succeed[WAIT_ACK] == MESS_DOF || ACK_Succeed[WAIT_ACK] == MESS_DOFA)
			{
				ACK_Succeed[ACK_DONE] = 1;
			}

			//MQTT_SubReceive_Callback(MQTT_SubMessage);
			Flag_Done_Get_Accurency = false;
			//			Flag_MQTT_SubMessage_Processing = false;
			Flag_MQTT_SubMessage_Processing = true;
			MQTT_SubMessage_NeedToProcess = false;
			Flag_Done_Get_Accurency = true;
		}
		if(MQTT_Message_NeedToProcess == true)
		{
			VTAG_Configure_temp = VTAG_Configure;
			MQTT_Message_NeedToProcess = false;
		}
		vTaskDelay(TIMER_ATC_PERIOD / RTOS_TICK_PERIOD_MS);
	}
}
//------------------------------------------------------------------------------------------------------------------------// Check timeout task
void check_timeout_task(void *arg)
{
	//xSemaphoreTake(sync_check_timeout_task, portMAX_DELAY);
	while (1)
	{
		if(SIMCOM_ATCommand.TimeoutATC > 0 && SIMCOM_ATCommand.CurrentTimeoutATC < SIMCOM_ATCommand.TimeoutATC)
		{
			SIMCOM_ATCommand.CurrentTimeoutATC += TIMER_ATC_PERIOD;
			if(SIMCOM_ATCommand.CurrentTimeoutATC >= SIMCOM_ATCommand.TimeoutATC)
			{
				SIMCOM_ATCommand.CurrentTimeoutATC -= SIMCOM_ATCommand.TimeoutATC;
				if(SIMCOM_ATCommand.RetryCountATC > 0)
				{
					SIMCOM_ATCommand.RetryCountATC--;
					RetrySendATC();
				}
				else
				{
					if(SIMCOM_ATCommand.SendATCallBack != NULL)
					{
						SIMCOM_ATCommand.TimeoutATC = 0;
						SIMCOM_ATCommand.SendATCallBack(EVEN_TIMEOUT, "@@@");
						ATC_Sent_TimeOut = 1;
					}
				}
			}
		}
		vTaskDelay(TIMER_ATC_PERIOD / RTOS_TICK_PERIOD_MS);
	}
}
//------------------------------------------------------------------------------------------------------------------------// GPS scan task
void gps_scan_task(void *arg)
{
	//xSemaphoreTake(sync_gps_scan_task, portMAX_DELAY);
	while (1)
	{
		if(Flag_Timer_GPS_Run == true)
		{
			switch (GPS_Select)
			{
			case GPS_Indirect:
			{
				Flag_Timer_GPS_Run = false;
				GPS_Read();
				break;
			}
			case GPS_NMEA:
			{
				if((strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0) && Flag_test_unpair == true)
				{
					gpio_set_level(LED_2, 1);
				}

				cnt++;
				//ESP_LOGW(TAG, "GPS Scan Time: %d s\r\n", cnt);
				if(cnt >= (GPS_Scan_Number))
				{
					cnt = 0;
					Flag_Timer_GPS_Run = false;
					Flag_NeedToProcess_GPS = false;
					gpio_set_level(UART_SW, 0);
					ATC_SendATCommand("AT+SGNSCMD=0\r\n", "OK", 1000, 1, StopGPS_Callback);
				}
				break;
			}
			default:
				break;
			}
		}
		vTaskDelay(150 / RTOS_TICK_PERIOD_MS);
		if((strchr(Device_PairStatus,'U') || strlen(Device_PairStatus) == 0) && GPS_Fix_Status == false && Flag_test_unpair == true)
		{
			gpio_set_level(LED_2, 0);
		}
		vTaskDelay(850 / RTOS_TICK_PERIOD_MS);
	}
}

//------------------------------------------------------------------------------------------------------------------------// GPS processing task
void GPS_Operation_Thread(void)
{
	GPS_Fix_Status = 0;
	// Set funtionality
	if(AT_RX_event == EVEN_OK)
	{
		Flag_Wait_Exit = false;
		ATC_SendATCommand("AT+CFUN=0\r\n", "OK", 10000, 0, SetCFUN_0_Callback);
		WaitandExitLoop(&Flag_Wait_Exit);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			SoftReboot7070G();
			Flag_Wait_Exit = false;
			ATC_SendATCommand("AT+CFUN=0\r\n", "OK", 10000, 0, SetCFUN_0_Callback);
			WaitandExitLoop(&Flag_Wait_Exit);
			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
			{

			}
		}
	}
	else if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
	{

	}
	// Enable GPS VCC
	if(AT_RX_event == EVEN_OK)
	{
		Flag_Wait_Exit = false;
		gpio_set_level(VCC_GPS_EN, 1);
	}
	else if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
	{
		// Skip
	}

	// Check enable GPS VCC, if can not enable GPS VCC then skip GPS
	if(AT_RX_event == EVEN_OK)
	{
		// GNSS operation
		switch (GPS_Select)
		{
		case GPS_Indirect:
		{
			// Power on GPS
			if(AT_RX_event == EVEN_OK && Flag_Control_7070G_GPIO_OK == true) {Flag_Control_7070G_GPIO_OK = false; ATC_SendATCommand("AT+CGNSPWR=1\r\n", "OK", 1000, 3, StartGPS_Callback);}
			WaitandExitLoop(&Flag_GPS_Started);
			// Scan GPS
			timer_start(timer_group, timer_idx);
			break;
		}
		case GPS_NMEA:
		{
			// Configure NMEA GPS
			Flag_Wait_Exit = false;
			ATC_SendATCommand("AT+SGNSCFG=\"NMEAOUTPORT\",2,115200\r\n", "OK", 1000, 3, StartGPS_Callback);
			WaitandExitLoop(&Flag_Wait_Exit);
			// Power on GPS
			Flag_Wait_Exit = false;
			if((AT_RX_event == EVEN_OK && Flag_GPS_Started == true))
			{
				Flag_GPS_Started = false;
				ATC_SendATCommand("AT+SGNSCMD=2,1000,0,1\r\n", "OK", 1000, 3, StartGPS_Callback);
				WaitandExitLoop(&Flag_Wait_Exit); // Continue the program even in the case GPS can not be started
				// If can not turn on GNSS then turn it off and turn it on again
				if(AT_RX_event == EVEN_OK)
				{
					Flag_Timer_GPS_Run = true;
				}
				else if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
				{
					// Turn off GNSS
					Flag_Wait_Exit = false;
					ATC_SendATCommand("AT+SGNSCMD=0\r\n", "OK", 1000, 3, StartGPS_Callback);
					WaitandExitLoop(&Flag_Wait_Exit);
					// Turn on GNSS
					if(AT_RX_event == EVEN_OK)
					{
						Flag_Wait_Exit = false;
						ATC_SendATCommand("AT+SGNSCMD=2,1000,0,1\r\n", "OK", 1000, 3, StartGPS_Callback);
						WaitandExitLoop(&Flag_Wait_Exit); // Continue the program even in the case GPS can not be started

						Flag_Timer_GPS_Run = true;
					}
					else break;
				}
			}
			break;
		}
		default:
			break;
		}

		if(AT_RX_event == EVEN_OK && Flag_GPS_Started == true)
		{
			gpio_set_level(UART_SW, 1);
			vTaskDelay(50/RTOS_TICK_PERIOD_MS);
			gpio_set_level(UART_SW, 1);
			WaitandExitLoop(&Flag_GPS_Stopped);
			gpio_set_level(UART_SW, 0);
		}
		// Disable GPS VCC
		Flag_Wait_Exit = false;
		gpio_set_level(VCC_GPS_EN, 0);
	}
}
static void GPS_Data_NMEA_Callback(void *ResponseBuffer)
{
	strcpy((char*)GNSS_Data_Str, (char*)ResponseBuffer);
}
void gps_processing_task(void *arg)
{
	//xSemaphoreTake(sync_gps_processing_task, portMAX_DELAY);
	while (1)
	{
		if(Flag_NeedToProcess_GPS == true)
		{
			Flag_NeedToProcess_GPS = false;
			if(GPS_Fix_Status != 1)
			{
				gps_process(&hgps, GNSS_Data_Str, strlen((char*)GNSS_Data_Str));
				//				hgps.is_valid = 1;
				if(hgps.is_valid == 1)
				{
					//ESP_LOGW(TAG, "------------Current Pos-----------\r\n");
					//ESP_LOGW(TAG, "Valid status: %d\r\n", hgps.is_valid);
					//ESP_LOGW(TAG, "Latitude: %f degrees\r\n", hgps.latitude);
					//ESP_LOGW(TAG, "Longitude: %f degrees\r\n", hgps.longitude);
					//ESP_LOGW(TAG, "Altitude: %f meters\r\n", hgps.altitude);
					//ESP_LOGW(TAG, "------------Time Stamp -----------\r\n");
					//ESP_LOGW(TAG,"hour: %d\r\n", (uint8_t)hgps.hours + 7);
					//ESP_LOGW(TAG,"min: %d\r\n", (uint8_t)hgps.minutes);
					//ESP_LOGW(TAG,"sec: %d\r\n",(uint8_t)hgps.seconds);
					//ESP_LOGW(TAG,"date: %d\r\n", (uint8_t)hgps.date);
					//ESP_LOGW(TAG,"month: %d\r\n", (uint8_t)hgps.month);
					//ESP_LOGW(TAG,"year: %d\r\n",(uint8_t)hgps.year + 2000);

					if(hgps.latitude != 0 && hgps.longitude != 0)
					{
						GPS_Fix_Status = 1;
					}
					else
					{
						GPS_Fix_Status = 0;
					}
					//					GPS_Fix_Status = 1;
					//ESP_LOGE(TAG, "GPS Time to First Fix: %d s\r\n", cnt);
					GPS_Scan_Counter = 0;
					Flag_Timer_GPS_Run = false;
					cnt = 0;
					if(Flag_GPS_Stopped == false)
					{
						ATC_SendATCommand("AT+SGNSCMD=0\r\n", "OK", 1000, 1, StopGPS_Callback);
						gpio_set_level(UART_SW, SW_UART_AT);
					}
				}
			}
		}
		vTaskDelay(50 / RTOS_TICK_PERIOD_MS);
	}
}
//------------------------------------------------------------------------------------------------------------------------// UART RX task
void uart_rx_task(void *arg)
{
	//xSemaphoreTake(sync_uart_rx_task, portMAX_DELAY);
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	char str_num[10];
	uart_config_t uart_config = {
			.baud_rate = ECHO_UART_BAUD_RATE,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_REF_TICK,
			//.source_clk = UART_SCLK_APB,
	};
	int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
	intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

	(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
	(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

	uint8_t data[BUF_SIZE];

	while (1)
	{
		// Read data from the UART
		int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 100 / RTOS_TICK_PERIOD_MS);

		if(len > 0)
		{
			data[len] = 0;
			ESP_LOGI(TAG, "Rec: \r\n%s", data);

			// Call GPS process function
			if(Flag_GPS_Started == true)
			{
				Flag_NeedToProcess_GPS = true;
				GPS_Data_NMEA_Callback(data);
			}

			// GÃ¡Â»ï¿½i cÃƒÂ¡c hÃƒÂ m callback tÃ†Â°Ã†Â¡ng Ã¡Â»Â©ng vÃ¡Â»â€ºi kÃ¡ÂºÂ¿t quÃ¡ÂºÂ£ AT trÃ¡ÂºÂ£ vÃ¡Â»ï¿½
			if(SIMCOM_ATCommand.ExpectResponseFromATC[0] != 0 && strstr((const char*)data, SIMCOM_ATCommand.ExpectResponseFromATC))
			{
				SIMCOM_ATCommand.ExpectResponseFromATC[0] = 0;
				if(SIMCOM_ATCommand.SendATCallBack != NULL)
				{
					SIMCOM_ATCommand.TimeoutATC = 0;
					SIMCOM_ATCommand.SendATCallBack(EVEN_OK, data);
				}
			}
			// Check error
			if(strstr((const char*)data, "ERROR"))
			{
				Flag_Cycle_Completed = true;
				if(SIMCOM_ATCommand.SendATCallBack != NULL)
				{
					SIMCOM_ATCommand.SendATCallBack(EVEN_ERROR, data);
				}
			}
			// Check MQTT subcribe
			if(strstr((const char*)data, "+SMSUB:"))
			{
				//ESP_LOGE(TAG, "Sub data\r\n");
				strcpy(MQTT_SubMessage, (const char*)data);
				MQTT_SubMessage_NeedToProcess = true;
			}
			// Check sms receive
			if(strstr((const char*)data, "+CMGR:"))
			{
				//ESP_LOGE(TAG, "SMS data: %s\r\n", data);
				strcpy(SMS_MessageReceive, (const char*)data);
				Flag_sms_receive = true;
				MQTT_Message_NeedToProcess = true;
			}
			if(strstr((const char*)data, "+CMTI:"))
			{
				//ESP_LOGE(TAG, "SMS data: %s\r\n", data);
				Flag_sms_receive = true;
				Flag_config = false;
				MQTT_Message_NeedToProcess = true;
				filter_comma((const char*)data, 1, 2, str_num, ',');
				num_sms = atoi((const char *)str_num);

			}
			//Read IMSI
			if(strstr((char*)data, "AT+CIMI"))
			{
				int index = 0;
				for(int i = 0; i < strlen((char*)data); i++)
				{
					if(data[i] > 47 && data[i] < 58)
					{
						IMSI[index++] = data[i];
					}
				}
				////ESP_LOGI(TAG, "IMSI buff: %s\r\n", IMSI);
			}
			if(strstr((char*)data, "+CSQ"))
			{
				char number[3] = {0};
				for(int i = 0; i < strlen((char*)data); i++)
				{
					if(data[i] > 47 && data[i] < 58)
					{
						number[0] = data[i];
						number[1] = data[i+1];
						break;
					}
				}
				if(atoi(number) == 99)
				{
					CSQ_value = 0;
				}
				if((CSQ_value < atoi(number) || CSQ_value == 0) && atoi(number) != 99)
				{
					CSQ_value = atoi(number);
				}
				//ESP_LOGI(TAG, "CSQ: %d\r\n", CSQ_value);
			}
		}
		vTaskDelay(10 / RTOS_TICK_PERIOD_MS);
		// Write data back to the UART
	}
}
//------------------------------------------------------------------------------------------------------------------------// Led indicator
void led_indicator(void *arg)
{
	bool Flag_Pair_Led = false;
	bool Flag_PowerOn_Led = false;
	if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
	{
		Flag_PowerOn_Led = true;
	}
	while(1)
	{
		if(Flag_PowerOn_Led == true)
		{
			Flag_PowerOn_Led = false;
			LED_TurnOn();
		}

		if(Flag_Fota_led == false && Flag_Pair_Led == false)
		{
			Flag_Pair_Led = true;
			LED_Pair();
		}

		if(Flag_button_long_press_led )
		{
			Flag_button_long_press_led = false;
			LED_SendSOS();
		}
		if(Flag_ChangeMode_led == true)
		{
			LED_ChangeMode();
			Flag_ChangeMode_led = false;
		}
		if(Flag_Fota_led == true)
		{
			LED_Fota();
		}
		if(Flag_Unpair_led == true)
		{
			LED_UnpairAndCfg();
			Flag_Unpair_led = false;
		}
		if(Flag_StopMotion_led == true)
		{
			LED_StopMove();
			Flag_StopMotion_led = false;
		}
		vTaskDelay(10 / RTOS_TICK_PERIOD_MS);
	}
}
void receive_sms()
{
	char Sub_Str[500];
	char Sub_Str2[200];
	char Sub_Str3[200];
	char stringDateTime[100];
	char str_at[15];
	time_t timestampSMSFuture;
	//	loopmain_reason = SMS_RECEIVE;
	//	saveLoopReason();
	ATC_SendATCommandOptimize("AT+CMGF=1\r\n", "OK", 1000, 5, ATResponse_Callback);

	ATC_SendATCommandOptimize("AT+CSCS=\"GSM\"\r\n", "OK", 1000, 5, ATResponse_Callback);

	ATC_SendATCommandOptimize("AT+CNMI=2,1\r\n", "OK", 1000, 5, ATResponse_Callback);

	sprintf(str_at, "AT+CMGR=%d\r\n", num_sms);
	ATC_SendATCommandOptimize(str_at, "+CMGR:", 1000, 2, ATResponse_Callback);
	vTaskDelay(1000/portTICK_PERIOD_MS);
	if(AT_RX_event == EVEN_OK)
	{
		//ESP_LOGW(TAG, "sms: %s", SMS_MessageReceive);
		filter_comma_sms(SMS_MessageReceive, "+CMGR:", Sub_Str3);
		filter_comma(Sub_Str3, 5, 6, stringDateTime, '\"');
		//ESP_LOGI(TAG, "date time: %s", stringDateTime);
		timestampSMSFuture = string_to_seconds(stringDateTime);
		//ESP_LOGI(TAG, "timestamp sms: %ld", timestampSMSFuture);
		if(Flag_sms_receive == true && strstr(SMS_MessageReceive, "007B") && timestampSMSFuture > timestampSMS)
		{
			timestampSMS = timestampSMSFuture;
			filter_comma_sms(SMS_MessageReceive, "007B", Sub_Str);
			//ESP_LOGI(TAG, "sms: %s", Sub_Str);
			decodeMessage(Sub_Str, Sub_Str2);
			//ESP_LOGI(TAG, "sms: %s", Sub_Str2);
			JSON_Analyze(Sub_Str2, &VTAG_Configure_temp);
			FlagSendLR = VTAG_Configure_temp.FLR;
			FlagFota2G = VTAG_Configure_temp.F2G;
			//ESP_LOGI(TAG, "VTAG_Configure_temp.FLR: %d", VTAG_Configure_temp.FLR);
			//ESP_LOGI(TAG, "VTAG_Configure_temp.F2G: %d", VTAG_Configure_temp.F2G);
			if(FlagFota2G == 1 && Flag_Fota == false)
			{
				acc_power_down();
				Flag_Fota = true;
				xTaskCreatePinnedToCore(Fota2G_processing_task, "uart", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &Fota2G_processing_handle, tskNO_AFFINITY);
				if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED && check_motion_handle != NULL)
				{
					vTaskDelete(check_motion_handle);
				}
				vTaskDelete(gps_scan_task_handle);
				vTaskDelete(gps_processing_task_handle);
				vTaskDelete(NULL);
			}
			if(VTAG_Configure_temp.Period != VTAG_Configure.Period || VTAG_Configure_temp.CC != VTAG_Configure.CC ||\
					VTAG_Configure_temp.WM != VTAG_Configure.WM|| VTAG_Configure_temp._lc != VTAG_Configure._lc ||  VTAG_Configure_temp.MA != VTAG_Configure.MA)
			{
#if LOG
				if(VTAG_Configure_temp.Period != VTAG_Configure.Period)
				{
					//ESP_LOGI(TAG, "P");
				}
				if( VTAG_Configure_temp.CC != VTAG_Configure.CC)
				{
					//ESP_LOGI(TAG, "CC");
				}
				if(VTAG_Configure_temp.WM != VTAG_Configure.WM)
				{
					//ESP_LOGI(TAG, "WM");
				}
				if(VTAG_Configure_temp._lc != VTAG_Configure._lc)
				{
					//ESP_LOGI(TAG, "lc");
				}
				if(VTAG_Configure_temp.MA != VTAG_Configure.MA)
				{
					//ESP_LOGI(TAG, "ma");
				}
#endif
				//ESP_LOGI(TAG, "SEND CONFIG AFTER LOCATION");
				Flag_config_sms = true;
				Flag_Unpair_led = false;
				JSON_Analyze(Sub_Str2, &VTAG_Configure);
				VTAG_Configure.FLR = 0;
				VTAG_Configure.F2G = 0;
				char str[150];
				sprintf(str, "{\"M\":%d,\"P\":%d,\"Type\":\"%s\",\"CC\":%d,\"N\":%d,\"a\":%d,\"T\":\"%s\",\"_ss\":%d,\"WM\":%d,\"_lc\":%d,\"MA\":%d}",\
						VTAG_Configure.Mode,VTAG_Configure.Period,VTAG_Configure.Type,VTAG_Configure.CC,VTAG_Configure.Network,\
						VTAG_Configure.Accuracy,VTAG_Configure.Server_Timestamp,VTAG_Configure._SS, VTAG_Configure.WM, \
						VTAG_Configure._lc, VTAG_Configure.MA);
				writetofile(base_path, "test.txt", str);
				Flag_ChangeMode_led = true;
			}
		}
		else if( timestampSMSFuture <= timestampSMS || !strstr(SMS_MessageReceive, "007B"))
		{
			Flag_sms_receive = false;
			loopmain_reason = SMS_RECEIVE_PAST;
			saveLoopReason();
			ESP_sleep(1);
		}
	}
	ATC_SendATCommandOptimize("AT+CMGD=1,4\r\n", "OK", 1000, 3, ATResponse_Callback);
	memset(SMS_MessageReceive, 0, strlen(SMS_MessageReceive));
	num_sms = 0;
	Flag_sms_receive = false;
}
void saveLoopReason()
{
	char str_loop[50];
	sprintf(str_loop, "%d%d", loopmain_reason, loopbutton_reason);
	printf("Loop reason now: %s \r\n", str_loop);
	writetofile(base_path, "vtag_loop.txt", str_loop);
}
//------------------------------------------------------------------------------------------------------------------------// Main task
void main_task(void *arg)
{
	//xSemaphoreTake(sync_main_task, portMAX_DELAY);
	//Subscribe this task to TWDT, then check if it is subscribe
	esp_task_wdt_add(NULL);
	esp_task_wdt_status(NULL);
	rtc_wdt_disable();
	//ESP_LOGW(TAG, "----------------------\r\n");
	while (1)
	{
		//ESP_LOGE(TAG, "main_task\r\n");
		while(Flag_button_cycle_start == true);
		//while(Flag_button_cycle_start);
		//wake up and wait 30s to check wherether there is motion
		if(Flag_acc_wake == true && Flag_motion_detected == true)
		{
			Flag_skip_rst_acc_ISR = true;
			Flag_mainthread_run = true;
			Flag_acc_wake = false;
			while(TrackingRuntime < 30 && Flag_sos == false && Flag_Unpair_Task == false && Flag_motion_acc_wake_check == false)
			{
				vTaskDelay(500 / RTOS_TICK_PERIOD_MS);
			}
			acc_counter = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - acc_capture, esp_clk_slowclk_cal_get())/1000000);
			if(acc_counter > 180)
			{
				Flag_send_DASP = true;
			}
			if(Flag_sos == false && acc_counter < 180 && Flag_Unpair_Task == false) ESP_sleep(0);
		}
		// If not full battery
		if(Ext1_Wakeup_Pin == CHARGE)
		{
			if(Flag_FullBattery == false)
			{
				//				esp_sleep_enable_ext0_wakeup((ACC_INT), 0);
				bitmap_wakeup |= 1ULL<<ACC_INT;
				if(gpio_get_level(CHARGE) == 0)
				{
					bitmap_wakeup |= 1ULL<<CHARGE;
				}
				bitmap_wakeup |= 1ULL<<BUTTON;
				esp_sleep_enable_ext1_wakeup(bitmap_wakeup, ESP_EXT1_WAKEUP_ANY_HIGH);
				ESP_sleep(0);
				esp_set_deep_sleep_wake_stub(&wake_stub);
				esp_deep_sleep_start();
			}
		}
		// if DASP in backup array then wake and send
		if(Flag_motion_detected == false && Backup_Array_Counter > 0 &&  esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
		{
			loopmain_reason = SEND_BK;
			saveLoopReason();
			//ESP_LOGE(TAG, "Send back up array %d time \r\n", Retry_count);
			Retry_count ++;
			t_total_passed_BU = 0;
			BU_checked = true;
			Flag_mainthread_run = true;

			//Trigged pwrkey and check AT command
			CheckAtCommand();
			// Get battery percent
			Check_battery();
			//Disable the RTC WDT timer
			rtc_wdt_disable();
			//			VTAG_MessType_G = SEND_BACKUP;
			//			MQTT_SendMessage_Thread(SEND_BACKUP);
			VTAG_MessType_G = UNPAIR_GET;
			MQTT_SendMessage_Thread(UNPAIR_GET);
			ESP_sleep(1);
		}
		//Flag_config = true de chan nhay vao ham nay khi co return sms
		if(ProgramRun_Cause == ESP_SLEEP_WAKEUP_TIMER && Flag_motion_detected == false && Flag_config == true && Check_vol_percent != NO_AC)
		{
			loopmain_reason = CHECK_PIN;
			saveLoopReason();
			//ESP_LOGI(TAG, "pin or nothing");
			t_total_passed_vol = 0;
			vol_checked = true;
			Flag_mainthread_run = true;

			CheckAtCommand();
			Check_battery();
			//Disable the RTC WDT timer
			rtc_wdt_disable();
			if (Check_vol_percent == Send_BP){
				Check_vol_percent = NO_AC;
				VTAG_MessType_G = UNPAIR_GET;
				MQTT_SendMessage_Thread(UNPAIR_GET);
			}
			Check_vol_percent = NO_AC;
			ESP_sleep(1);
		}
		ESP_NOTSLEEP:
		// Addded:  Flag_FullBattery == true
		if(Flag_Cycle_Completed == true && (ProgramRun_Cause == ESP_SLEEP_WAKEUP_UNDEFINED || Flag_sms_receive || Flag_FullBattery == true|| \
				Flag_Fota == true || Flag_sos == true || Flag_motion_detected == true || acc_counter > 180 || Flag_period_wake == true || \
				Flag_Unpair_Task == true) && Flag_button_do_nothing == false)
		{
#if 0
			if(Flag_motion_detected == true)
			{
				//ESP_LOGI(TAG, "device motion detection");
			}
			if(acc_counter > 180)
			{
				//ESP_LOGI(TAG, "acc >180");
			}
			if(Flag_period_wake == true)
			{
				//ESP_LOGI(TAG, "period wake");
			}
#endif
			loopmain_reason = MAIN_ENL;
			saveLoopReason();
			Flag_mainthread_run = true;
			Flag_period_wake = false;
			Flag_Cycle_Completed = false;
			if((ProgramRun_Cause == ESP_SLEEP_WAKEUP_TIMER || ProgramRun_Cause == ESP_SLEEP_WAKEUP_EXT1) \
					&& VTAG_Configure.WM == 1 && VTAG_Configure.MA == 0 && !Flag_config_sms && !Flag_sms_receive && !Flag_FullBattery)
			{
				//4
				loopmain_reason = WIFI_DETECT_1;
				saveLoopReason();
				//ESP_LOGI(TAG, "CHECK WIFI\r\n");
				if(Flag_wifi_init == false)
				{
					wifi_init();
				}
				wifi_scan();
				if(Flag_wifi_detected == false)
				{
					wifi_motion_detect = wifi_detect_motion(wifi_ap_pre, wifi_ap_cur);
				}
				if (ap_count > 3 && wifi_motion_detect == false)
				{
					if (Flag_send_DAST == true)
					{
						Flag_send_DAST = false;
						acc_counter = 0;
						Flag_motion_detected = false;
						flag_end_motion = true;
						Flag_StopMotion_led = true;
						//						Flag_wifi_scan = true;
					}
					if(Flag_sos == false && Flag_Unpair_Task == false && Backup_Array_Counter == 0)
					{
						wifi_detect_int = 1;
						if(Flag_motion_detected == true && Flag_send_DAST == false)
						{
							loopmain_reason = WD1_DASP;
							saveLoopReason();
							GetDeviceTimestamp();
							//ESP_LOGE(TAG, "send dasp after find changing of wifi\r\n");
							acc_counter = 0;
							Flag_motion_detected = false;
							flag_end_motion = true;
							Flag_send_DASP = true;
							Flag_StopMotion_led = true;
							//							Flag_wifi_scan = true;
							CheckAtCommand();
							if(Flag_wifi_init == false)
							{
								wifi_init();
							}
							wifi_scan();
							if(VTAG_Configure._SS == 1)
							{
								VTAG_MessType_G = LOCATION;
								MQTT_SendMessage_Thread(LOCATION);
								Check_accuracy();
							} else
							{
								Flag_send_DASP = false;
							}
							ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
							ESP_sleep(1);
							goto ESP_NOTSLEEP;
						}
						if (Flag_send_DASP == true || Flag_send_DAST == true)
						{
							acc_counter = 0;
							Flag_motion_detected = false;
							flag_end_motion = true;
							Flag_send_DASP = false;
							Flag_StopMotion_led = true;
							//							Flag_wifi_scan = true;
						}
						// 6
						Flag_send_DAST = false;
						loopmain_reason = WD1_SLEEP;
						saveLoopReason();
						ATC_SendATCommandOptimize("AT\r\n", "OK", 1000, 0, ATResponse_Callback);
						if(AT_RX_event == EVEN_OK || AT_RX_event == EVEN_ERROR)
						{
							ESP_sleep(1);
						}
						else
						{
							ESP_sleep(0);
						}
						goto ESP_NOTSLEEP;
					}
				}
				else if(wifi_motion_detect == true)
				{
					LED_StartMove();
				}
				if(Flag_mess_sended_wd == false && Backup_Array_Counter > 0)
				{
					loopmain_reason = WD1_BACKUP;
					saveLoopReason();
					// trigged pwrkey and check at command
					CheckAtCommand();
					VTAG_MessType_G = UNPAIR_GET;
					MQTT_SendMessage_Thread(UNPAIR_GET);
					ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
					ESP_sleep(1);
					goto ESP_NOTSLEEP;
				}
			}
			// trigged pwrkey and check at command
			CheckAtCommand();
			if((VTAG_Configure.MA == 1 || Flag_motion_detected == false) && Flag_sms_receive == true)
			{
				receive_sms();
			}
			Flag_config = true; // reset true ke ca co convert hay k
			if(VTAG_Configure.MA != 0 && esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
			{
				//ESP_LOGI(TAG, "turn off detect sensor after message");
				Flag_motion_detected = false;
				acc_power_down();
				Flag_send_DAST = false;
			}
			// Get battery percent
			Flag_Wait_Exit = false;
			SelectNB_GSM_Network(NB_IoT);
			WaitandExitLoop(&Flag_Wait_Exit);
			Check_battery();
			//Disable the RTC WDT timer
			rtc_wdt_disable();
			if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
			{
				//ESP_LOGI(TAG,"Pre operation configure\r\n");
				PreOperationInit();
			}
			// Main thread run here
			MAIN_THREAD:
			switch (ProgramRun_Cause)
			{
			// wake by ACC
			case  ESP_SLEEP_WAKEUP_EXT0:
			{
				//ESP_LOGE(TAG, "ESP_SLEEP_WAKEUP_EXT0\r\n");
				loopmain_reason = WAKEUP_EXT0;
				saveLoopReason();
				if(Flag_Unpair_Task == true && Flag_motion_detected == false && !Flag_config_sms)
				{
					Flag_Unpair_Task = false;
					//					Flag_wifi_scan = true;
					if(Flag_wifi_init == false)
					{
						wifi_init();
					}
					wifi_scan();
					GetDeviceTimestamp();
					VTAG_MessType_G = LOCATION;
					MQTT_SendMessage_Thread(LOCATION);
					Flag_Unpair_Task = false;
					ESP_sleep(1);
					// If return from TurnOff7070G then change wake cause = TIMER and go to MAIN_THREAD to send SOS || CGF || DAST
					ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
					CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
					//					goto MAIN_THREAD;
					break;
				}
				else
				{

					ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
					goto MAIN_THREAD;
				}
				break;
			}
			// wake by button or charge
			case ESP_SLEEP_WAKEUP_EXT1: // Cause by external wakeup
			{
				//ESP_LOGE(TAG, "ESP_SLEEP_WAKEUP_EXT1\r\n");
				ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
				loopmain_reason = WAKEUP_EXT1;
				saveLoopReason();
				if(Flag_Unpair_Task == true && Flag_motion_detected == false && !Flag_config_sms)
				{
					Flag_Unpair_Task = false;
					//					Flag_wifi_scan = true;
					if(Flag_wifi_init == false)
					{
						wifi_init();
					}
					wifi_scan();
					GetDeviceTimestamp();
					VTAG_MessType_G = LOCATION;
					MQTT_SendMessage_Thread(LOCATION);
					Flag_Unpair_Task = false;
					ESP_sleep(1);
					CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
					// If return from TurnOff7070G then change wake cause = TIMER and go to MAIN_THREAD to send SOS || CGF || DAST
					ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
					//					goto MAIN_THREAD;
					break;
				}
				if(Ext1_Wakeup_Pin == CHARGE)
				{
					// Send DBF message
					loopmain_reason = PIN_FULL;
					saveLoopReason();
					if(Flag_FullBattery == true)
					{
						Flag_FullBattery = false;
						// Get battery percent
						//Bat_GetPercent();
						// MQTT operation
						VTAG_MessType_G = FULLBATTERY;
						MQTT_SendMessage_Thread(FULLBATTERY);
						// Turn off 7070G and set ESP32 to deep sleep
						Ext1_Wakeup_Pin = 0;
						ESP_sleep(1);
						break;
					}
					// if not full battery status then turn off 7070 and go to sleep
					else
					{
						ESP_sleep(1);
						break;
					}
				}
				else
				{
					// Go back to main thread
					//ESP_LOGI(TAG, "Go back to main thread");
					goto MAIN_THREAD;
				}
				break;
			}
			case ESP_SLEEP_WAKEUP_TIMER:// Run main thread after getting timestamp
			{
				//ESP_LOGE(TAG, "Run main thread\r\n");
				//				//ESP_LOGE(TAG, "NORMAL MODE\r\n");
				// GNSS operation
				// Run this operation only for GNSS period with country mode , all DAST, DASP, CFG, SOS are send via WifiCell
				if((VTAG_Configure.CC != 1|| VTAG_Configure.MA == 1 || FlagSendLR == 1) && Flag_sos == 0 && acc_counter < 180 && Flag_send_DASP == false && Flag_send_DAST == false && Flag_Unpair_Task == false && !Flag_config_sms)
				{
					Flag_location_send_1 = true;
					if(VTAG_Configure.CC == 2|| VTAG_Configure.MA == 1 || FlagSendLR == 1)
					{
						loopmain_reason = MODE_CC2;
						saveLoopReason();
						//ESP_LOGI(TAG, "mode cc = 2");
						GPS_Scan_Number = 45;
						GPS_Operation_Thread();
						CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
						GetDeviceTimestamp();
						// If can not get GPS location, execute Wifi scan
						if(GPS_Fix_Status != 1)
						{
							//GPS_Fix_Status = 0;
							//							Flag_wifi_scan = true;
							if(Flag_wifi_init == false)
							{
								wifi_init();
							}
							wifi_scan();
							if(ap_count < 3)
							{
								if(VTAG_Configure._lc == 1)
								{
									Flag_LBS_need = true;
								}
								else
								{
									VTAG_MessType_G = LOCATION;
									MQTT_SendMessage_Thread(LOCATION);
									ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
									ESP_sleep(1);
									break;
								}
							}
							else
							{
								Flag_need_check_accuracy = true;
								Flag_LBS_need = false;
							}
						}
						// MQTT operation
						VTAG_MessType_G = LOCATION;
						MQTT_SendMessage_Thread(LOCATION);
						if(ap_count >= 3 || Flag_need_check_accuracy == true)
						{
							if(VTAG_Configure.Accuracy > 60)
							{
								if(VTAG_Configure._lc == 1)
								{
									Flag_LBS_need = true;
									VTAG_MessType_G = LOCATION;
									MQTT_SendMessage_Thread(LOCATION);
									ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
									ESP_sleep(1);
									CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
								}
							}
						}
						FlagSendLR = 0;
						ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
						ESP_sleep(1);
					}
					else if(VTAG_Configure.CC == 3)
					{
						loopmain_reason = MODE_CC3;
						saveLoopReason();
						//ESP_LOGI(TAG, "mode cc = 3");
						GPS_Scan_Number = 80;
						GPS_Operation_Thread();
						CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
						GetDeviceTimestamp();
						if(GPS_Fix_Status)
						{
							VTAG_MessType_G = LOCATION;
							MQTT_SendMessage_Thread(LOCATION);
						}
						else
						{
							VTAG_MessType_G = GPS_FAIL;
							MQTT_SendMessage_Thread(GPS_FAIL);
						}
						ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
						ESP_sleep(1);
					}
					break;
				}
				//if mode city, sos, unpair/config, send DASP, DAST
				if(((VTAG_Configure.CC == 1 || Flag_sos == true || Flag_Unpair_Task == true || Flag_send_DASP == true || Flag_send_DAST == true) && acc_counter < 180 )\
						|| Flag_config_sms == true)
				{
#if LOG
					if(Flag_send_DAST)
					{
						//ESP_LOGI(TAG, "send DAST");
					}
					if(Flag_send_DASP)
					{
						//ESP_LOGW(TAG, "send DASP");
					}
					if(Flag_sos)
					{
						//ESP_LOGW(TAG, "send SOS");
					}
					if(VTAG_Configure.MA == 1)
					{
						//ESP_LOGW(TAG, "location request");
					}
					if(Flag_config_sms)
					{
						//ESP_LOGW(TAG, "send config");
					}
#endif
					loopmain_reason = MODE_CC1;
					saveLoopReason();
					Flag_location_send_2 = true;
					//					Flag_wifi_scan = true;
					if(Flag_wifi_init == false)
					{
						wifi_init();
					}
					wifi_scan();
					GetDeviceTimestamp();
					if(ap_count < 3)
					{
						GPS_Scan_Number = 45;
						GPS_Operation_Thread();
						if(GPS_Fix_Status != 1)
						{
							if(VTAG_Configure._lc == 1)
							{
								Flag_LBS_need = true;
							}
						}
						else
						{
							Flag_LBS_need = false;
						}
					}
					else
					{
						Flag_need_check_accuracy = true;
					}


					if(Flag_config_sms)
					{
						VTAG_MessType_G = DCF_ACK;
						MQTT_SendMessage_Thread(DCF_ACK);
					}
					else if(Flag_send_DAST == true || (Flag_sos == true && Flag_motion_detected == false))
					{
						VTAG_MessType_G = UNPAIR_GET;
						MQTT_SendMessage_Thread(UNPAIR_GET);
					}
					else
					{
						VTAG_MessType_G = LOCATION;
						MQTT_SendMessage_Thread(LOCATION);
					}
					if(ap_count >= 3 || Flag_need_check_accuracy == true)
					{
						Check_accuracy();
					}
					ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
					Flag_Unpair_Task = false;
					ESP_sleep(1);
					break;
				}
				if(Flag_motion_detected == true && Flag_send_DAST == false)
				{
					acc_counter = (uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - acc_capture, esp_clk_slowclk_cal_get())/1000000);
					//ESP_LOGI(TAG, "acc_counter: %d s",(int)acc_counter);
					if(acc_counter > 180 || Flag_send_DASP == true)
					{
						GetDeviceTimestamp();
						acc_counter = 0;
						Flag_motion_detected = false;
						flag_end_motion = true;
						Flag_send_DASP = true;
						Flag_StopMotion_led = true;
						//						Flag_wifi_scan = true;
						if(VTAG_Configure._SS == 1)
						{
							if(Flag_wifi_init == false)
							{
								wifi_init();
							}
							wifi_scan();
							VTAG_MessType_G = LOCATION;
							MQTT_SendMessage_Thread(LOCATION);
							Check_accuracy();
						}
						else
						{
							Flag_send_DASP = false;
						}
						ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
						ESP_sleep(1);
						break;
					}
				}
				CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
				break;
			}
			case ESP_SLEEP_WAKEUP_UNDEFINED: // First run, send DON message to get timestamp
			{
				loopmain_reason = WAKEUP_UNDEFINED;
				saveLoopReason();
				// MQTT operation
				//				Flag_wifi_scan = true;
				if(Flag_wifi_init == false)
				{
					wifi_init();
				}
				wifi_scan();
				wifi_detect_motion(wifi_ap_pre, wifi_ap_cur);
				//					get calib battery factor when DON
				Check_battery();
				read_battery();
				//ESP_LOGW(TAG,"VTAG_DeviceParameter =  %d mV\r\n",VTAG_DeviceParameter.Bat_Voltage);
				//ESP_LOGW(TAG,"read_battery =  %f mV\r\n",read_battery());
				Calib_adc_factor = (float) VTAG_DeviceParameter.Bat_Voltage * 1.0 / (float) read_battery();
				//ESP_LOGW(TAG,"Calib_adc_factor =  %f mV\r\n",Calib_adc_factor);
				//ESP_LOGW(TAG,"VBAT =  %f mV\r\n",read_battery());
				VTAG_MessType_G = STARTUP;
				MQTT_SendMessage_Thread(STARTUP);
				Check_accuracy();
				ESP_sleep(1);
				ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
				break;
			}
			default:
			{
				ProgramRun_Cause = ESP_SLEEP_WAKEUP_TIMER;
				// Go back to main thread
				goto MAIN_THREAD;
			}
			}
		}
		vTaskDelay(1000 / RTOS_TICK_PERIOD_MS);
	}
}
//void fota_ack(void *arg)
//{
//	bool add_wdt = false;
//	gpio_hold_dis(VCC_7070_EN);
//	gpio_set_level(VCC_7070_EN, 0);
//	vTaskDelay(500/RTOS_TICK_PERIOD_MS);
//	int retry_send = 0;
//	while(1)
//	{
//		if(Flag_Fota_fail == true && Flag_Fota == true)
//		{
//			if(add_wdt == false)
//			{
//				add_wdt = true;
//				esp_task_wdt_init(150, ESP_OK);
//				esp_task_wdt_add(NULL);
//				esp_task_wdt_status(NULL);
//
//			}
//			gpio_set_level(VCC_7070_EN, 1);
//			if(retry_send > 2)
//			{
//				//				esp_restart();
//				forcedReset();
//			}
//			retry_send++;
//			SEND_BACKUP:
//
//			//ESP_LOGW(TAG, "Power on Sim7070G\r\n");
//			TurnOn7070G();
//			vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);
//
//			//ESP_LOGE(TAG, "Start program\r\n");
//			gpio_set_level(UART_SW, 0);
//			// Check AT response
//			ATC_SendATCommand("AT\r\n", "OK", 1000, 5, ATResponse_Callback);
//			WaitandExitLoop(&Flag_Wait_Exit);
//
//			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
//			{
//				Flag_Cycle_Completed = true;
//				goto SEND_BACKUP;
//			}
//			Check_battery();
//			PreOperationInit();
//			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
//			{
//				Flag_Cycle_Completed = true;
//				goto SEND_BACKUP;
//			}
//			//Disable the RTC WDT timer
//			rtc_wdt_disable();
//			VTAG_MessType_G = FOTA_FAIL;
//			MQTT_SendMessage_Thread_Fota(FOTA_FAIL);
//			gpio_hold_dis(VCC_7070_EN);
//			gpio_set_level(VCC_7070_EN, 0);
//		}
//		vTaskDelay(500 / RTOS_TICK_PERIOD_MS);
//	}
//}
int isNumber(char *str)
{
    for (int i = 0; str[i]!= '\0'; i++)
    {
        if (isdigit(str[i]) == 0)
              return 0;
    }
    return 1;
}
void Fota2G_processing_task(void *arg)
{
	esp_task_wdt_add(NULL);
	esp_task_wdt_status(NULL);
	rtc_wdt_disable();
	esp_err_t ret;
	bool Flag_wdt_1time = false;
	//ESP_LOGI(TAG, "start fota by 2g");
	bool Flag_Fota2GStart = true;
	bool Flag_Fota2GSucc = false;
	bool Flag_Fota2GFail = false;
	int n= 0;
	while (1)
	{
		if(Flag_Fota == true && FlagFota2G == 1)
		{
			if(!Flag_wdt_1time)
			{
				Flag_wdt_1time = true;
				CHECK_ERROR_CODE(esp_task_wdt_init(200, true), ESP_OK);
				CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
				ESP32_Clock_Config(20, 20, false);
			}
			FlagFota2G = 0;
			ResetAllParameters();
			//ESP_LOGE(TAG, "Start program\r\n");
			// Check AT response
			CYCLE_FOTA2G:
			CheckAtCommand();
			// Get battery percent
			Flag_Wait_Exit = false;
			SelectNB_GSM_Network(GSM);
			WaitandExitLoop(&Flag_Wait_Exit);
			Check_battery();
			if(Flag_Fota2GStart == true)
			{
				Flag_Fota2GStart = false;
				VTAG_MessType_G = FOTA_START;
				MQTT_SendMessage_Thread(FOTA_START);
				if(Backup_reason != NORMAL)
				{
					FlagFota2G = 1;
					Flag_Fota2GStart = true;
					Flag_Wait_Exit = false;
					ATC_SendATCommand("AT+CFUN=0\r\n", "OK", 10000, 1, SetCFUN_0_Callback);
					WaitandExitLoop(&Flag_Wait_Exit);
					Flag_Wait_Exit = false;
					ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 10000, 1, SetCFUN_1_Callback);
					WaitandExitLoop(&Flag_Wait_Exit);
					n++;
					if(n > 1)
					{
						sprintf(Device_shutdown_ts,"%lld", VTAG_DeviceParameter.Device_Timestamp);
						writetofile(base_path, "vtag_off_ts.txt", Device_shutdown_ts);
						vTaskDelay(500/RTOS_TICK_PERIOD_MS);
						esp_restart();
					}
				}
				else
				{
					vTaskDelay(1000/RTOS_TICK_PERIOD_MS);
					CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
					ret = update_2G_handle();
					CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);
					if(ret == ESP_OK)
					{
						Flag_Fota2GSucc = true;
						//ESP_LOGE(TAG, "SEND DOSS\r\n");
						goto CYCLE_FOTA2G;
					}
					else
					{
						Flag_Fota2GFail = true;
						//ESP_LOGE(TAG, "SEND DOFA\r\n");
						goto CYCLE_FOTA2G;
					}
				}
			}
			if(Flag_Fota2GSucc == true)
			{
				Flag_Fota2GSucc = false;
				char str[150];
				sprintf(str, "{\"M\":%d,\"P\":%d,\"Type\":\"%s\",\"CC\":%d,\"N\":%d,\"a\":%d,\"T\":\"%s\",\"_ss\":%d,\"WM\":%d,\"_lc\":%d, \"MA\":%d}",\
						VTAG_Configure.Mode,VTAG_Configure.Period,VTAG_Configure.Type,VTAG_Configure.CC,VTAG_Configure.Network,\
						VTAG_Configure.Accuracy,VTAG_Configure.Server_Timestamp,VTAG_Configure._SS, VTAG_Configure.WM, 1, 1);
				writetofile(base_path, "test.txt", str);
				//				Flag_test_connectSimcom = true;
				VTAG_MessType_G = FOTA_SUCCESS;
				MQTT_SendMessage_Thread(FOTA_SUCCESS);
				//ESP_LOGI(TAG, "Restarting...");
				esp_restart();
			}
			if(Flag_Fota2GFail == true)
			{
				Flag_Fota2GSucc = false;
				VTAG_MessType_G = FOTA_FAIL;
				MQTT_SendMessage_Thread(FOTA_FAIL);
				//ESP_LOGI(TAG, "Restarting...");
				esp_restart();
			}
		}
		vTaskDelay(1000 / RTOS_TICK_PERIOD_MS);
	}
}
void test_main_task(void *arg)
{
#define THRES_GSM 		5 // 5
#define THRES_NB 		2 // 2
#define GPS_SCAN_TIME 	60
#define NB_SCAN_TIME 	80
#define	GSM_SCAN_TIME	30

	esp_task_wdt_init(10000, ESP_OK);
	esp_task_wdt_add(NULL);
	esp_task_wdt_status(NULL);
	rtc_wdt_disable();
	rtc_wdt_protect_off();
	GPS_Scan_Number = GPS_SCAN_TIME;
	bool Flag_GPS_ok = false;
	bool Flag_NB_ok = false;
	bool Flag_GSM_ok = false;
	int retry = 0;
	while(1)
	{
		CYCLE_START:
		gpio_hold_dis(VCC_7070_EN);
		gpio_set_level(VCC_7070_EN, 0);
		vTaskDelay(500/RTOS_TICK_PERIOD_MS);
		gpio_set_level(VCC_7070_EN, 1);
		vTaskDelay(500/RTOS_TICK_PERIOD_MS);
		//ESP_LOGW(TAG, "Power on Sim7070G\r\n");
		TurnOn7070G();
		vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);

		//ESP_LOGE(TAG, "Start program\r\n");
		gpio_set_level(UART_SW, 0);
		// Check AT response
		ATC_SendATCommandOptimize("AT\r\n", "OK", 1000, 3, ATResponse_Callback);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			Flag_Cycle_Completed = true;
			goto CYCLE_START;
		}
		// Set clock
		SelectNB_GSM_Network(NB_IoT);

		ATC_SendATCommandOptimize("AT+CSCLK=0\r\n", "OK", 3000, 3, ATResponse_Callback);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			Flag_Cycle_Completed = true;
			goto CYCLE_START;
		}

		ATC_SendATCommandOptimize("AT+CMNB=3\r\n", "OK", 1000, 3, ATResponse_Callback);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			Flag_Cycle_Completed = true;
			goto CYCLE_START;
		}

		//Fix band for nb-iot
		ATC_SendATCommandOptimize("AT+CBANDCFG=\"NB-IOT\",3\r\n", "OK", 1000, 3, ATResponse_Callback);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			Flag_Cycle_Completed = true;
			goto CYCLE_START;
		}

		//Select optimizeation for band scan
		ATC_SendATCommandOptimize("AT+CNBS=2\r\n", "OK", 1000, 3, ATResponse_Callback);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			Flag_Cycle_Completed = true;
			goto CYCLE_START;
		}

		//
		ATC_SendATCommandOptimize("AT+CBAND=\"ALL_MODE\"\r\n", "OK", 1000, 2, ATResponse_Callback);
		if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
		{
			Flag_Cycle_Completed = true;
			goto CYCLE_START;
		}
		//Check gps scan
		if(Flag_GPS_ok == false)
		{
			GPS_Operation_Thread();
			//			vTaskDelete(gps_scan_task_handle);
			//			vTaskDelete(gps_processing_task_handle);

			gpio_set_level(LED_2, 0);
			//vTaskDelete(led_indicator_handle);
			vTaskDelay(1000/ RTOS_TICK_PERIOD_MS);
			if(GPS_Fix_Status == 1)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				Flag_GPS_ok = true;
			}
			else
			{
				gpio_set_level(LED_2, 1);
				vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				Flag_GPS_ok = false;
			}
		}

		if(Flag_NB_ok == false)
		{
			//ESP_LOGW(TAG, "Scan NB_IoT network\r\n");
			gpio_hold_dis(VCC_7070_EN);
			gpio_set_level(VCC_7070_EN, 0);
			vTaskDelay(500/RTOS_TICK_PERIOD_MS);
			gpio_set_level(VCC_7070_EN, 1);
			vTaskDelay(500/RTOS_TICK_PERIOD_MS);
			CYCLE_START_1:
			//ESP_LOGW(TAG, "Power on Sim7070G\r\n");
			TurnOn7070G();
			vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);

			ATC_SendATCommandOptimize("AT\r\n", "OK", 1000, 3, ATResponse_Callback);
			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
			{
				Flag_Cycle_Completed = true;
				goto CYCLE_START_1;
			}

			gpio_set_level(UART_SW, 0);
			Flag_CFUN_1_OK = false;
			ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 3000, 3, SetCFUN_1_Callback);
			WaitandExitLoop(&Flag_CFUN_1_OK);

			Flag_Wait_Exit = false;
			SelectNB_GSM_Network(NB_IoT);
			WaitandExitLoop(&Flag_Wait_Exit);

			for(int i = 0; i < NB_SCAN_TIME; i++)
			{
				ATC_SendATCommandOptimize("AT+CSQ\r\n", "OK", 1000, 3, ATResponse_Callback);
				gpio_set_level(LED_2, 1);
				vTaskDelay(300 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				vTaskDelay(700 / RTOS_TICK_PERIOD_MS);
				if(CSQ_value > THRES_NB && CSQ_value < 32)
				{
					break;
				}
			}
			vTaskDelay(1000/ RTOS_TICK_PERIOD_MS);
			if(CSQ_value > THRES_NB && CSQ_value < 32)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				Flag_NB_ok = true;
			}
			else
			{
				gpio_set_level(LED_2, 1);
				vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				Flag_NB_ok = false;
			}
		}

		if(Flag_GSM_ok == false)
		{
			//ESP_LOGW(TAG, "Scan GSM network\r\n");
			gpio_hold_dis(VCC_7070_EN);
			gpio_set_level(VCC_7070_EN, 0);
			vTaskDelay(500/RTOS_TICK_PERIOD_MS);
			gpio_set_level(VCC_7070_EN, 1);
			vTaskDelay(500/RTOS_TICK_PERIOD_MS);
			CYCLE_START_2:
			//ESP_LOGW(TAG, "Power on Sim7070G\r\n");
			TurnOn7070G();
			vTaskDelay(4000 / RTOS_TICK_PERIOD_MS);

			ATC_SendATCommandOptimize("AT\r\n", "OK", 1000, 3, ATResponse_Callback);
			if(AT_RX_event == EVEN_TIMEOUT || AT_RX_event == EVEN_ERROR)
			{
				Flag_Cycle_Completed = true;
				goto CYCLE_START_2;
			}

			CSQ_value = 0;
			Flag_CFUN_1_OK = false;
			ATC_SendATCommand("AT+CFUN=1\r\n", "OK", 3000, 3, SetCFUN_1_Callback);
			WaitandExitLoop(&Flag_CFUN_1_OK);

			Flag_Wait_Exit = false;
			SelectNB_GSM_Network(GSM);
			WaitandExitLoop(&Flag_Wait_Exit);
			int count_skip_NB = 0;
			for(int i = 0; i < GSM_SCAN_TIME; i++)
			{
				count_skip_NB++;
				ATC_SendATCommandOptimize("AT+CSQ\r\n", "OK", 1000, 3, ATResponse_Callback);
				gpio_set_level(LED_2, 1);
				vTaskDelay(300 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				vTaskDelay(700 / RTOS_TICK_PERIOD_MS);
				if(CSQ_value > THRES_GSM && CSQ_value < 32)
				{
					break;
				}
			}
			vTaskDelay(1000/ RTOS_TICK_PERIOD_MS);
			if(CSQ_value > THRES_GSM && CSQ_value < 32)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				Flag_GSM_ok = true;
			}
			else
			{
				gpio_set_level(LED_2, 1);
				vTaskDelay(2000 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_2, 0);
				Flag_GSM_ok = false;
			}
		}

		if(Flag_GPS_ok == true && Flag_NB_ok == true && Flag_GSM_ok == true)
		{
			vTaskDelete(check_timeout_task_handle);
			while(Flag_GPS_ok == true && Flag_NB_ok == true && Flag_GSM_ok == true)
			{
				gpio_set_level(LED_1, 1);
				vTaskDelay(200 / RTOS_TICK_PERIOD_MS);
				gpio_set_level(LED_1, 0);
				vTaskDelay(200 / RTOS_TICK_PERIOD_MS);
			}
		}
		if(Flag_GPS_ok == false || Flag_NB_ok == false || Flag_GSM_ok == false)
		{
			if(retry < 2)
			{
				retry++;
				//				xTaskCreatePinnedToCore(gps_scan_task, "gps scan", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &gps_scan_task_handle, tskNO_AFFINITY);
				//				xTaskCreatePinnedToCore(gps_processing_task, "gps processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &gps_processing_task_handle, tskNO_AFFINITY);
				goto CYCLE_START;
			}
			else
			{
				//				vTaskDelete(check_timeout_task_handle);
				while(Flag_GPS_ok == false || Flag_NB_ok == false || Flag_GSM_ok == false)
				{
					gpio_set_level(LED_2, 1);
					vTaskDelay(200 / RTOS_TICK_PERIOD_MS);
					gpio_set_level(LED_2, 0);
					vTaskDelay(200 / RTOS_TICK_PERIOD_MS);
				}
			}
		}
		vTaskDelay(1000 / RTOS_TICK_PERIOD_MS);
	}
}

//------------------------------------------------------------------------------------------------------------------------// App main
void app_main(void)
{
	esp_int_wdt_init();
	// Init for MWDT
	//ESP_LOGW(TAG,"Initialize MWDT\r\n");
	CHECK_ERROR_CODE(esp_task_wdt_init(100, true), ESP_OK);
	esp_task_wdt_add(NULL);
	// Init for RWDT
	//ESP_LOGW(TAG,"Initialize RWDT\r\n");
	rtc_wdt_protect_off();
	rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
	rtc_wdt_set_time(RTC_WDT_STAGE0, 60000);
	rtc_wdt_enable();           //Start the RTC WDT timer
	rtc_wdt_protect_on();       //Enable RTC WDT write protection
	rtc_wdt_disable();
	xMutex_LED = xSemaphoreCreateMutex();
	ESP32_GPIO_Output_Init();
	ESP32_GPIO_Input_Init();
	//ESP_LOGI(TAG, "VTAG ESP32 Application Running x\r\n");
	// Mount flash
	mountSPIFFS();
	if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
	{
		Reboot_reason = esp_reset_reason();
		//ESP_LOGI(TAG, "esp_reset_reason: %d\r\n", esp_reset_reason());
		Flag_calib = true;
		char Calib_str[50] = "";
		readfromfile(base_path, "calib.txt", Calib_str);
		Calib_factor = atof(Calib_str);
		//ESP_LOGI(TAG, "Calib_factor read from file: %f s\r\n", Calib_factor);
		readfromfile(base_path, "test.txt", config_buff);
		JSON_Analyze_init_config(config_buff, &VTAG_Configure);
		struct timeval epoch = {0, 0};
		struct timezone utc = {0,0};
		settimeofday(&epoch, &utc);

		//---------------------//
		int bool_check = 0;
		readfromfile(base_path, "vtag_loop.txt", loopString);
		//ESP_LOGI(TAG, "Loop reason: %s \r\n", loopString);
		bool_check = isNumber(loopString);
		if(bool_check == 0)
		{
			memset(loopString, 0, strlen(loopString));
		}
		writetofile(base_path, "vtag_loop.txt", "00");
	}
	// Read device ID
#if	 	0
	//		WritingFATFlash(base_path, "vtag_id.txt", Device_ID_TW);
	writetofile(base_path, "vtag_id.txt", Device_ID_TW);
	//WritingFATFlash(base_path, "vtag_pr.txt", "U");
#endif
	// Check sleep wakeup cause
	CheckSleepWakeUpCause();
	// Reconfigure ESP32 clock
	ESP32_Clock_Config(20, 20, false);
	esp_log_level_set("wifi", ESP_LOG_WARN);
	Flag_check_run = true;
	// Init ESP32 ADC
	Bat_ADC_Init();

	i2c_master_init();
	acc_config(60, 2);
	gpio_init();
	rtc_config_t rtc_configure =
	{
			.ck8m_wait = RTC_CNTL_CK8M_WAIT_DEFAULT,
			.xtal_wait = RTC_CNTL_XTL_BUF_WAIT_DEFAULT,
			.pll_wait  = RTC_CNTL_PLL_BUF_WAIT_DEFAULT,
			.clkctl_init = 1,
			.pwrctl_init = 1,
			.rtc_dboost_fpd = 1
	};
	rtc_init(rtc_configure);
	;

	struct timeval now;
	gettimeofday(&now, NULL);
	using_device_time_s = now.tv_sec;
	//ESP_LOGW(TAG, "Time now before calib: %ld\r\n", now.tv_sec);

	// Init for normal timer
	tg0_timer0_init();
	TrackingPeriod_Timer_Init();
	timer_start(timer_group, TrackingPeriod_timer_idx);

	if(esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
	{
		time_slept = ((uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - t_stop, esp_clk_slowclk_cal_get())/1000000));
		//ESP_LOGI(TAG, "time_slept: %d s\r\n",(int)time_slept);
	}

	// add slept_time to calib gettimeofday value
	t_slept_calib = ((uint64_t)round(rtc_time_slowclk_to_us(rtc_time_get() - t_stop_calib, esp_clk_slowclk_cal_get())/1000000));
	//ESP_LOGI(TAG, "time_slept_calib: %d s\r\n",(int)(t_slept_calib));

	if(Calib_factor < 1) Calib_factor = 1;
	//	Calib_factor = 1;
	//ESP_LOGI(TAG, "Calib_factor: %f \r\n", Calib_factor);
	struct timeval epoch = {(long)(now.tv_sec + (long)( time_slept * (Calib_factor - 1))), 0};
	struct timezone utc = {0,0};
	settimeofday(&epoch, &utc);
	gettimeofday(&now, NULL);
	//ESP_LOGW(TAG, "Time now after calib: %ld\n", now.tv_sec);
	//ESP_LOGW(TAG, "wifi loop count: %d",count_loop);
	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );
	//	char config_buff[100];
	ResetAllParameters();
	// Flash LED for charging indication
	if(gpio_get_level(CHARGE) == 0)
	{
		// Init for ULP
		init_ulp_program();
		/* Start the ULP program */
		esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
		ESP_ERROR_CHECK(err);
		state = IN_CHARGING;
	}
	// Init for GPS processing
	gps_init(&hgps);
	//Create semaphores to synchronize
	sync_main_task = xSemaphoreCreateBinary();
	sync_uart_rx_task = xSemaphoreCreateBinary();
	sync_check_timeout_task = xSemaphoreCreateBinary();
	sync_gps_scan_task = xSemaphoreCreateBinary();
	sync_gps_processing_task = xSemaphoreCreateBinary();
	sync_mqtt_submessage_processing_task = xSemaphoreCreateBinary();
	sync_charging_task = xSemaphoreCreateBinary();

	//Create and start main task
	xTaskCreatePinnedToCore(main_task, "main", 4096*2, NULL, MAIN_TASK_PRIO, &main_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(uart_rx_task, "uart", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &uart_rx_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(check_timeout_task, "check_timeout", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &check_timeout_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(gps_scan_task, "gps scan", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &gps_scan_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(gps_processing_task, "gps processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &gps_processing_task_handle, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(mqtt_submessage_processing_task, "mqtt submessage processing", 4096*2, NULL, CHECKTIMEOUT_TASK_PRIO, &mqtt_submessage_processing_handle, tskNO_AFFINITY);
	//  xTaskCreatePinnedToCore(led_indicator, "led_indicator", 4096, NULL, CHECKTIMEOUT_TASK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(charging_task, "charging", 4096, NULL, CHECKTIMEOUT_TASK_PRIO, NULL, tskNO_AFFINITY);

	esp_task_wdt_delete(NULL);
	// SemaphoreGive to start running tasks
	xSemaphoreGive(sync_main_task);
	xSemaphoreGive(sync_uart_rx_task);
	xSemaphoreGive(sync_check_timeout_task);
	xSemaphoreGive(sync_gps_scan_task);
	xSemaphoreGive(sync_gps_processing_task);
	xSemaphoreGive(sync_mqtt_submessage_processing_task);
	xSemaphoreGive(sync_charging_task);
}
