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
extern float voltage;
extern float R1; // 205K ohm resistor
extern float R2; 
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
// Function prototypes without extern
 void led_control_task(void *pvParameters);
 void init_led_control();
 void set_led_fade(bool red, bool green, bool blue);
 void set_led_solid(bool red, bool green, bool blue);
 void set_led_blink(bool red, bool green, bool blue, int blink_interval_ms);
 void set_led_blink_alternate(bool red1, bool green1, bool blue1, 
                             bool red2, bool green2, bool blue2, 
                             int blink_interval_ms);
 void stop_leds();
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
esp_err_t save_int_to_nvs(const char *key, int value);
void TemperaturedataRS(void);
void init_led_control(void);
void set_led_fade(bool red, bool green, bool blue);
void set_led_solid(bool red, bool green, bool blue);
void set_led_blink(bool red, bool green, bool blue, int blink_interval_ms);
void set_led_blink_alternate(bool red1, bool green1, bool blue1,
                            bool red2, bool green2, bool blue2,
                            int blink_interval_ms);
void stop_leds(void);
void ConnectWifi();
float read_battery_voltage(void);
int calculate_battery_percentage(float voltage);

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
#define MAX_MSG_LENGTH 256
#define MAX_LINE_LENGTH 49  // Maximum length for each individual line
#define CHARS_PER_LINE 49   // Target characters per line for wrapping
#define MAX_LINES 3         // Maximum number of lines supported
#define BACK_MAX_MSG_LENGTH 512    // Total message buffer
#define BACK_LINE_LENGTH 64        // Each line buffer
#define BACK_CHARS_PER_LINE 30     // Actual characters per line
#define BACK_MAX_LINES 5           // Maximum supported lines
RTC_DATA_ATTR static int number_of_inactive_msg_lines = 1;

// Global variables for storing processed lines
RTC_DATA_ATTR static char Back_line1[BACK_LINE_LENGTH] = {0};
RTC_DATA_ATTR static char Back_line2[BACK_LINE_LENGTH] = {0};
RTC_DATA_ATTR static char Back_line3[BACK_LINE_LENGTH] = {0};
RTC_DATA_ATTR static char Back_line4[BACK_LINE_LENGTH] = {0};
RTC_DATA_ATTR static char Back_line5[BACK_LINE_LENGTH] = {0};

typedef struct {
    char lines[BACK_MAX_LINES][BACK_LINE_LENGTH];
    int num_lines;
} back_text_t;

// Function declarations
void update_display_message(void);
typedef struct {
    char lines[MAX_LINES][MAX_LINE_LENGTH];
    int num_lines;
} wrapped_text_t;

wrapped_text_t process_front_message(void);
#ifdef __cplusplus
}
#endif
 
#endif // CPP_FUNCTIONS_H