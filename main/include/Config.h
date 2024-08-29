#include "esp_sleep.h"
#include"CJson.h"
extern int ConfigDevID;
extern int OTACN;
extern char NEMS_ID[20];
void ConnectWifi();
void InitTempButton(void);
void print_wakeup_reason();
static void init_littlefs();
void start_lora_queue_task(void);
void init_lora_queue(void);
char* CreateLoraMessage(int device_id, int hub_id, const char* message, int total_length);
esp_err_t send_message_with_ack(int DevID, int HubID, const char* message, int Length);
esp_err_t send_lora_message(int DevID, int HubID, const char* message,int Length);
void Temperaturedata(void);
cJSON* load_ack_json(void);
void save_ack_json(cJSON* json);
void create_or_update_ack(const char* message_id, const char* status);
const char* check_ack_status(const char* message_id);
void ulp_component(int ulp_timer_wakeup_time_in_min);
void configure_gpio();
void print_wakeup_reason();
void configure_wakeup();
void process_and_send_hdc1080_data(void);
/////ULP
// This is the ponter which stores the address of the incoming LoRa message
extern uint32_t *address_tx;
// This is the ponter which stores the address of the incoming LoRa message
extern uint32_t *address;
// This is the ponter which stores the address of the incoming hdc1080 sensor temperature buffer
extern uint32_t *address_t;
// This is the ponter which stores the address of the incoming hdc1080 sensor humidity buffer
extern uint32_t *address_h;
////////////