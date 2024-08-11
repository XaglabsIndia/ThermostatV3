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