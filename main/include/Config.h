#include "esp_sleep.h"
#include"CJson.h"
#include"lora.h"
extern int ConfigDevID;
extern int OTACN;
extern char NEMS_ID[20];
#define NVS_KEY "set_temp_f"
#define NVS_KEY_TEMP "temp_f"
#define NVS_KEY_HUMIDITY "humidity_f"
#define NVS_KEY_DATE "date_f"
#define NVS_KEY_TIME "time_f"
void ConnectWifi();
void InitTempButton(void);
void print_wakeup_reason();
static void init_littlefs();
void start_lora_queue_task(void);
void init_lora_queue(void);
char* CreateLoraMessage(int device_id, int hub_id, const char* message, int total_length);
esp_err_t send_message_with_ack(int DevID, int HubID, const char* message, int Length);
esp_err_t send_lora_message(int DevID, int HubID, const char* message,int Length);
void TemperaturedataRS(void);
void enter_sleep_mode_Timmer();
void enter_sleep_mode();
cJSON* load_ack_json(void);
void save_ack_json(cJSON* json);
void create_or_update_ack(const char* message_id, const char* status);
const char* check_ack_status(const char* message_id);
void ulp_component(int ulp_timer_wakeup_time_in_min);
void configure_gpio();
void print_wakeup_reason();
void configure_wakeup();
void process_and_send_hdc1080_data(void);
void Temperaturedata(void);
void InitalizeProgram();
/////ULP
// // This is the ponter which stores the address of the incoming LoRa message
// extern uint32_t *address_tx;
// // This is the ponter which stores the address of the incoming LoRa message
// extern uint32_t *address;
// // This is the ponter which stores the address of the incoming hdc1080 sensor temperature buffer
// extern uint32_t *address_t;
// // This is the ponter which stores the address of the incoming hdc1080 sensor humidity buffer
// extern uint32_t *address_h;
////////////
#ifndef CONFIG_H
#define CONFIG_H
 
#ifdef __cplusplus
extern "C" {
#endif
extern float set_temperature;
// Declare the function prototype
void InitMain();
void InitTempButton(void);
void InitNVS();
esp_err_t read_float_from_nvs(const char *key, float *value);
esp_err_t read_string_from_nvs(const char *key, char *value, size_t max_len);
esp_err_t read_int_from_nvs(const char *key, int *value);
void TemperaturedataRS(void);
#ifdef __cplusplus
}
#endif
 
#endif // MY_C_FUNCTIONS_H

//////////////CPP//////////////////
#ifndef CPP_FUNCTIONS_H
#define CPP_FUNCTIONS_H
 
#ifdef __cplusplus
extern "C" {
#endif
 
void setTemperature_partial_update();
void inactive_screen_call(void);
void roomTemperature_partial_update(float sensor_temperature_value);
void humidity_partial_update(int humidity);
void date_partial_update(char *date);
void time_partial_update(char *time);
void Inactive_message_Init(void);
void Display_Main(void);
#ifdef __cplusplus
}
#endif
 
#endif // CPP_FUNCTIONS_H