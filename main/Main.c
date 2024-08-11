#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_rom_sys.h"
#include "lora.h"
#include <esp_system.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <time.h>
#include <string.h>
#include "freertos/queue.h"
#include "sdkconfig.h"
#include <stdlib.h>
#include "esp_task_wdt.h"
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include "esp_littlefs.h"
#include <esp_timer.h>
#include "ProvisionHeader.h"
#include "xag_nvs_component.h"
#include "MultiButtonHeader.h"
#include"Config.h"
#include "xag_ota_component.h"
#include <ctype.h>
#include "nvs_flash.h"
#include "xag_wifi_component.h"
#include "hdc1080.h"

#define MAX_FLOAT_STR_LEN 20
#define FLOAT_PRECISION 2 

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(int)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define TWDT_TIMEOUT_MS         8000
#define TASK_RESET_PERIOD_MS    2000
#define configUSE_IDLE_HOOK 1   // In your FreeRTOSConfig.h file
#define MessageLenGTH 255
#define END_CHAR ')'
#define PADDING_CHAR '*'
#define SEPARATOR ','
#define ACK_RETRY_COUNT 5
#define ACK_RETRY_DELAY_MS 1000
#define LITTLEFS_BASE_PATH "/littlefs"
#define MAX_PATH_LENGTH 256
#define MAX_FILENAME_LENGTH 64
#define MAX_RETRY_COUNT 3
#define RETRY_DELAY_MS 1000
TaskHandle_t LoraRXContiniousTaskHandle = NULL;
TaskHandle_t LoraRXContiniousMessageQueueTaskHandle = NULL;
QueueHandle_t LoraRXCOntiniousQueue;
SemaphoreHandle_t RXDoneSemaphore;
TaskHandle_t AutoProvision = NULL;
static SemaphoreHandle_t MessageMutex;
esp_task_wdt_user_handle_t LoraRXContiniousTaskHandleWDT,LoraRXContiniousMessageQueueTaskHandleWDT;


void IRAM_ATTR RXDone(void* arg);
void IRAM_ATTR MultiButtonISR(void *arg);
void InitalizeProgram();
void AutoProvisionTask();
void processMessage(const char *message);
void floatToString(float value, char* buffer, int bufferSize, int precision);
void HandleMatchingMessage(int DevID, int HubID, const char* message);
bool ParseMessage(const char* input);


char* TAG = "Main";
int ConfigDevID = 201;
int OTACN = 0;
char NEMS_ID[20] = "NEMS";
/////////Lora RX Continious Mode//////////////// 
int TimeIntervalRX = 5;
uint8_t LoraContiRXLength[] = {25,220,27,9};
char txBuffer[255];
////////Lora RX Single Mode//////////////// 
uint8_t LoraSingleRXLength =20;

char HUBKeyMain[10] = "HUBID";
char DEVKeyMain[10] = "DEVID";
char THKeyMain[10] = "THID";
char RelayKeyMain[10] = "RELAYID";
float set_temperature = 20.0f; // Default temperature, now a float
// char OTAC[10] = "OTAC";
int ValueMain;
char mqttTopic[20];
int InitValueMain =0;
uint32_t notification;
static char CurrentMessage[255];
int StoredDevID, StoredHubID;
float Globaltemperature = 0.0;
int TaskResetDecision = 0;
static char g_message_id[3] = {0};


// Helper function to send message with retry
static esp_err_t send_thermostat_message_with_retry(const char* message) {
    int HUBID = 100;
    for (int i = 0; i < MAX_RETRY_COUNT; i++) {
        esp_err_t result = send_message_with_ack(ConfigDevID, HUBID, message, 35);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Message sent successfully to thermostat %d: %s (attempt %d)", ConfigDevID, message, i + 1);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Failed to send message to thermostat %d: %s (attempt %d)", ConfigDevID, message, i + 1);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }
    return ESP_FAIL;
}

void handle_mqtt_data(const char *data, size_t len)
{
    ESP_LOGI(TAG, "Entering handle_mqtt_data");
}
//////////////////General Funtions ////////////////////////////
void floatToString(float value, char* buffer, int bufferSize, int precision) {
    int intPart = (int)value;
    float fracPart = fabsf(value - intPart);
    for (int i = 0; i < precision; i++) {
        fracPart *= 10;
    }
    snprintf(buffer, bufferSize, "%d.%0*d", intPart, precision, (int)(fracPart + 0.5f));
}

char* IntStrCoverter(int value, int max_length) {
    // Allocate memory for the string
    char* str = (char*)malloc(11 * sizeof(char));
    if (str == NULL) {
        // Handle memory allocation failure
        return NULL;
    }
 
    // Convert integer to string
    snprintf(str, 11, "%d", value);
 
    // Check if the string length exceeds the maximum allowed length
    if (strlen(str) > max_length) {
        // If it does, truncate the string
        str[max_length] = '\0';
    }
 
    return str;
}
 
// Function to free the allocated memory
void free_int_string(char* str) {
    free(str);
}


 
char* CreateLoraMessage(int device_id, int hub_id, const char* message, int total_length) {
    // Allocate memory for the Lora message (+1 for null terminator)
    char* lora_message = (char*)malloc((total_length + 1) * sizeof(char));
    if (lora_message == NULL) {
        return NULL;  // Memory allocation failed
    }
    
    // Convert device_id and hub_id to strings
    char device_id_str[12] = "0";  // Default to "0" if no device_id
    char hub_id_str[12] = "0";     // Default to "0" if no hub_id
    if (device_id != -1) {
        sprintf(device_id_str, "%d", device_id);
    }
    if (hub_id != -1) {
        sprintf(hub_id_str, "%d", hub_id);
    }

    // Calculate the length of the main parts of the message
    int message_length = (message != NULL) ? strlen(message) : 0;
    int main_length = strlen(NEMS_ID) + strlen(device_id_str) + strlen(hub_id_str) + message_length + 4;  // +4 for three separators and END_CHAR

    // Check if the main parts fit within the total_length
    if (main_length > total_length) {
        // The message is too long, we need to truncate it
        int available_length = total_length - strlen(NEMS_ID) - strlen(device_id_str) - strlen(hub_id_str) - 4;
        snprintf(lora_message, total_length + 1, "%s%c%s%c%s%c%.*s%c", 
                 NEMS_ID, SEPARATOR, device_id_str, SEPARATOR, hub_id_str, SEPARATOR, available_length, (message != NULL) ? message : "", END_CHAR);
    } else {
        // The message fits, so we can add padding
        snprintf(lora_message, total_length + 1, "%s%c%s%c%s%c%s%c", 
                 NEMS_ID, SEPARATOR, device_id_str, SEPARATOR, hub_id_str, SEPARATOR, (message != NULL) ? message : "", END_CHAR);
        
        // Pad the rest of the message with PADDING_CHAR
        for (int i = main_length; i < total_length; i++) {
            lora_message[i] = PADDING_CHAR;
        }
        lora_message[total_length] = '\0';
    }
    printf("Generated message is %s",lora_message);
    return lora_message;
}
// Function to free the allocated memory
void free_lora_message(char* message) {
    free(message);
}

void RXContiniousMessageQueue(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    //ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    char rxBuffer[255];
    uint32_t notification;
 
    while (1)
    {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        if (xQueueReceive(LoraRXCOntiniousQueue, &(rxBuffer), (TickType_t)10) == pdTRUE)
        {
            ESP_LOGI("RecieveMessageQueue", "Received Message From LoraRXContiniousTask : %s", rxBuffer);
            if (xSemaphoreTake(MessageMutex, portMAX_DELAY) == pdTRUE) {
                strncpy(CurrentMessage, rxBuffer, sizeof(CurrentMessage) - 1);
                CurrentMessage[sizeof(CurrentMessage) - 1] = '\0'; // Ensure null-termination
                if (ParseMessage(CurrentMessage)) {
                    ESP_LOGI("RecieveMessageQueue", "Message processed successfully");
                } else {
                    ESP_LOGW("RecieveMessageQueue", "Message not processed (no match or error)");
                }
                xSemaphoreGive(MessageMutex);
            }
        }
        memset(rxBuffer, 0, sizeof(rxBuffer));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
bool ParseMessage(const char* Input) {
    if (Input == NULL) {
        ESP_LOGE("ParseMessage", "Input is NULL");
        return false;
    }

    char* InputCopy = strdup(Input);
    ESP_LOGI("ParseMessage", "Raw input: %s", Input);
    if (InputCopy == NULL) {
        ESP_LOGE("ParseMessage", "Memory allocation failed");
        return false;
    }

    bool processed = false;
    char* NEMS_ID = NULL;
    char* token = NULL;
    char* saveptr = NULL;
    int DevID = 0, HubID = 0;
    char message[MessageLenGTH + 1] = {0};

    // Parse NEMS_ID
    NEMS_ID = strtok_r(InputCopy, ",", &saveptr);
    if (NEMS_ID == NULL || strcmp(NEMS_ID, "NEMS") != 0) {
        ESP_LOGE("ParseMessage", "Invalid or missing NEMS_ID");
        goto cleanup;
    }

    // Parse DevID
    token = strtok_r(NULL, ",", &saveptr);
    if (token == NULL) {
        ESP_LOGE("ParseMessage", "Failed to parse DevID");
        goto cleanup;
    }
    DevID = atoi(token);

    // Parse HubID
    token = strtok_r(NULL, ",", &saveptr);
    if (token == NULL) {
        ESP_LOGE("ParseMessage", "Failed to parse HubID");
        goto cleanup;
    }
    HubID = atoi(token);

    // Get the message content
    token = strtok_r(NULL, ")", &saveptr);
    if (token == NULL) {
        ESP_LOGE("ParseMessage", "Failed to parse message content");
        goto cleanup;
    }
    ESP_LOGI("ParseMessage", "MessageLenGTH: %d", MessageLenGTH);
    // Copy the message content
    strncpy(message, token, MessageLenGTH);
    message[MessageLenGTH] = '\0';

    // Trim any padding characters
    size_t MessageLen = strlen(message);
    while (MessageLen > 0 && message[MessageLen - 1] == PADDING_CHAR) {
        message[--MessageLen] = '\0';
    }
    ESP_LOGI("ParseMessage", "Parsed message content: %s", message);

    // Check if DevID and HubID match
    if ((DevID == StoredDevID) && HubID == StoredHubID) {
        HandleMatchingMessage(DevID, HubID, message);
        processed = true;
    } else {
        ESP_LOGW("ParseMessage", "DevID or HubID mismatch. Received: %d,%d, Stored: %d,%d", 
                 DevID, HubID, StoredDevID, StoredHubID);
    }

cleanup:
    if (InputCopy != NULL) {
        free(InputCopy);
        InputCopy = NULL;
    }
    return processed;
}


void SetHeaterState(bool on) {
    gpio_set_level(CONFIG_BLUE_LED_GPIO, 0);
    gpio_set_level(CONFIG_RED_LED_GPIO, !on);
    gpio_set_level(CONFIG_GREEN_LED_GPIO, on);
    gpio_set_level(CONFIG_RELAY_GPIO, on);
    save_int_to_nvs(RelayKeyMain, on ? 1 : 0);
    printf(" *************   Heater %s   *************\n", on ? "On" : "Off");
}
void HandleAckMessage(const char* ack_message) {
    ESP_LOGI(TAG, "Received ACK message: %s", ack_message);

    char* message_copy = strdup(ack_message);
    if (message_copy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for message copy");
        return;
    }

    char* id_str = strtok(message_copy, "|");
    char* ack_str = strtok(NULL, "|");

    if (id_str != NULL && ack_str != NULL && strcmp(ack_str, "ACK") == 0) {
        ESP_LOGI(TAG, "Valid ACK format. Message ID: %s", id_str);

        cJSON* json = load_ack_json();
        if (json == NULL) {
            ESP_LOGE(TAG, "Failed to load ACK JSON");
            free(message_copy);
            return;
        }

        cJSON* status = cJSON_GetObjectItem(json, id_str);
        if (status != NULL) {
            if (cJSON_IsString(status) && strcmp(status->valuestring, "WAITING") == 0) {
                cJSON_DeleteItemFromObject(json, id_str);
                cJSON_AddStringToObject(json, id_str, "RECEIVED");
                save_ack_json(json);
                ESP_LOGI(TAG, "Updated ACK status for message %s: RECEIVED", id_str);
            } else {
                ESP_LOGW(TAG, "Unexpected status for message %s: %s", id_str, 
                         cJSON_IsString(status) ? status->valuestring : "Not a string");
            }
        } else {
            ESP_LOGW(TAG, "No existing entry found for message ID: %s", id_str);
        }

        cJSON_Delete(json);
    } else {
        ESP_LOGW(TAG, "Invalid ACK message format: %s", ack_message);
    }
        // Make sure to release the mutex at the end
    xSemaphoreGive(MessageMutex);
    free(message_copy);
}

void HandleOTAMessage( const char* message_id,int DevID,int HubID){
                // Create and send ACK message
            char ack_message[20];
            snprintf(ack_message, sizeof(ack_message), "%s|ACK", message_id);
            
            char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, 30);
            if (lora_message != NULL) {
                esp_err_t send_result;
                int retry_count = 0;
                do {
                    send_result = SendMessageWithCAD(lora_message);
                    if (send_result != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to send ACK, retrying... (attempt %d)", retry_count + 1);
                        vTaskDelay(pdMS_TO_TICKS(ACK_RETRY_DELAY_MS));
                    }
                    retry_count++;
                } while (send_result != ESP_OK && retry_count < ACK_RETRY_COUNT);

                if (send_result == ESP_OK) {
                    ESP_LOGI(TAG, "ACK sent successfully");
                } else {
                    ESP_LOGE(TAG, "Failed to send ACK after %d attempts", ACK_RETRY_COUNT);
                }
                
                free(lora_message);
            } else {
                ESP_LOGE(TAG, "Failed to create LoRa message for ACK");
            }
            esp_task_wdt_delete(LoraRXContiniousTaskHandle);
            esp_task_wdt_delete(LoraRXContiniousMessageQueueTaskHandle);
        if (connectWifi() == ESP_OK)
                {
                    printf("Wifi connected, checking for updates\n");
                    xag_ota_init();
                }
}

void HandleThIdMessage(const char* numStart, const char* message_id, int DevID, int HubID) {
    char numStr[10] = {0};
    int i = 0;
    while (isdigit((unsigned char)*numStart) && i < 9) {
        numStr[i++] = *numStart++;
    }
    if (i > 0) {
        int thidValue = atoi(numStr);
        esp_err_t err = save_int_to_nvs("THID", thidValue);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save ThID to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Saved ThID %d to NVS", thidValue);
            
            // Create and send ACK message
            char ack_message[20];
            snprintf(ack_message, sizeof(ack_message), "%s|ACK", message_id);
            
            char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, 30);
            if (lora_message != NULL) {
                esp_err_t send_result;
                int retry_count = 0;
                do {
                    send_result = SendMessageWithCAD(lora_message);
                    if (send_result != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to send ACK, retrying... (attempt %d)", retry_count + 1);
                        vTaskDelay(pdMS_TO_TICKS(ACK_RETRY_DELAY_MS));
                    }
                    retry_count++;
                } while (send_result != ESP_OK && retry_count < ACK_RETRY_COUNT);

                if (send_result == ESP_OK) {
                    ESP_LOGI(TAG, "ACK sent successfully");
                } else {
                    ESP_LOGE(TAG, "Failed to send ACK after %d attempts", ACK_RETRY_COUNT);
                }
                
                free(lora_message);
            } else {
                ESP_LOGE(TAG, "Failed to create LoRa message for ACK");
            }
        }
    } else {
        ESP_LOGE(TAG, "Invalid ThID format");
    }
}

void HandleStatusMessage() {
    int retry = 0;
    vTaskSuspend(LoraRXContiniousTaskHandle);
    int valueMain;
    esp_err_t checkRelStatus = CheckStoredKeyStatus(RelayKeyMain, &valueMain);
    if (checkRelStatus != ESP_OK) {
        ESP_LOGE("HandleStatusMessage", "Failed to check relay status");
        goto cleanup;
    }

    char* relayStatus = IntStrCoverter(valueMain, 2);
    if (relayStatus == NULL) {
        ESP_LOGE("HandleStatusMessage", "Failed to convert relay status to string");
        goto cleanup;
    }

    char* statusMessage = CreateLoraMessage(ConfigDevID, StoredHubID, relayStatus, 30);
    if (statusMessage == NULL) {
        ESP_LOGE("HandleStatusMessage", "Failed to create status message");
        free(relayStatus);
        goto cleanup;
    }

    esp_err_t sendResult;
    do {
        sendResult = SendMessageWithCAD(statusMessage);
        if (sendResult != ESP_OK) {
            ESP_LOGW("SendStatus", "Send attempt %d failed. Retrying...", ++retry);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (sendResult != ESP_OK && retry < 3);

    if (sendResult == ESP_OK) {
        ESP_LOGI("SendStatus", "Message sent successfully after %d attempt(s)", retry + 1);
    } else {
        ESP_LOGE("SendStatus", "Failed to send message after %d attempts", retry);
    }

    free(relayStatus);
    free(statusMessage);

cleanup:
    vTaskResume(LoraRXContiniousTaskHandle);
    RecivedLoraContiniousModeInit();
}

void HandleWiFiChangeMessage(const char* ssid, const char* password, const char* message_id, int DevID, int HubID) {
    ESP_LOGI(TAG, "Handling WiFi change message. SSID: %s, Password: %s", ssid, password);
    StoreHardcodeWiFiData(ssid,password);
    char ack_message[20];
    snprintf(ack_message, sizeof(ack_message), "%s|ACK", message_id);
    
    char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, 30);
    if (lora_message != NULL) {
        esp_err_t send_result;
        int retry_count = 0;
        do {
            send_result = SendMessageWithCAD(lora_message);
            if (send_result != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send ACK, retrying... (attempt %d)", retry_count + 1);
                vTaskDelay(pdMS_TO_TICKS(ACK_RETRY_DELAY_MS));
            }
            retry_count++;
        } while (send_result != ESP_OK && retry_count < ACK_RETRY_COUNT);

        if (send_result == ESP_OK) {
            ESP_LOGI(TAG, "ACK sent successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send ACK after %d attempts", ACK_RETRY_COUNT);
        }
        
        free(lora_message);
    } else {
        ESP_LOGE(TAG, "Failed to create LoRa message for ACK");
    }

}
void HandleMatchingMessage(int DevID, int HubID, const char* message) {
    if (message == NULL) {
        ESP_LOGE(TAG, "Received NULL message");
        return;
    }

    ESP_LOGI(TAG, "Matching message from Dev %d, Hub %d: %s", DevID, HubID, message);

    char message_id[3] = {0};

    // Check if the message starts with two digits and a pipe
    if (strlen(message) >= 3 && isdigit((unsigned char)message[0]) && isdigit((unsigned char)message[1]) && message[2] == '|') {
        strncpy(message_id, message, 2);
        message_id[2] = '\0';
        ESP_LOGI(TAG, "Message ID: %s", message_id);

        // Move past the "XX|" part
        const char* message_content = message + 3;
        if (strncmp(message_content, "ACK", 4) == 0) {
            HandleAckMessage(message);  // Pass the full message
            return;  // Exit after handling ACK
        } 
    }

    // Check if the message is an OTA message
    if (strncmp(message, "OTA", 3) == 0) {
        ESP_LOGI(TAG, "OTA Message: %s", message);
        HandleOTAMessage(message_id, DevID, HubID);
        return;  // Exit after handling OTA
    }

    // Check for WiFi change message
    if (strlen(message) > 2 && message[0] == 'W' && message[1] == '@') {
        ESP_LOGI(TAG, "WiFi change message received: %s", message);
        
        // Parse SSID and password
        const char* wifi_params = message + 2;  // Skip "W@"
        char ssid[33] = {0};  // Max SSID length is 32 characters
        char password[65] = {0};  // Max WPA2 password length is 64 characters
        
        const char* separator = strchr(wifi_params, '@');
        if (separator != NULL) {
            size_t ssid_length = separator - wifi_params;
            if (ssid_length < sizeof(ssid)) {
                strncpy(ssid, wifi_params, ssid_length);
                ssid[ssid_length] = '\0';
                
                const char* password_start = separator + 1;
                strncpy(password, password_start, sizeof(password) - 1);
                
                ESP_LOGI(TAG, "Parsed SSID: %s, Password: %s", ssid, password);
                HandleWiFiChangeMessage(ssid, password, message_id, DevID, HubID);
            } else {
                ESP_LOGE(TAG, "SSID too long");
            }
        } else {
            ESP_LOGE(TAG, "Invalid WIFI change message format");
        }
        return;  // Exit after handling WiFi change
    }

    // If we reach here, it's an unknown message type
    ESP_LOGW(TAG, "Unknown message type: %s", message);
}
 
///////////////////////LoRa Continious task ISR/////////////////

void IRAM_ATTR RXDone(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpio_isr_handler_remove(CONFIG_LORA_DIO0_GPIO);
    xSemaphoreGiveFromISR(RXDoneSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}



// Lora Continous task 

void LoraRXContiniousTask(void *arg)
{     
ESP_ERROR_CHECK(esp_task_wdt_add_user("LoraRXContinious",&LoraRXContiniousTaskHandleWDT));


  while(1) {        
        esp_err_t res = esp_task_wdt_reset_user(LoraRXContiniousTaskHandleWDT);
        if (res != ESP_OK) {
            printf("Failed to reset TWDT for RX continpus mode\n");
        }
// Wait for the RX done semaphore
     if (xSemaphoreTake(RXDoneSemaphore,  pdMS_TO_TICKS(10)) == pdTRUE) {
     esp_err_t err =  RecivedLoraContiniousMode(TimeIntervalRX, LoraContiRXLength, sizeof(LoraContiRXLength) / sizeof(LoraContiRXLength[0]), LoraRXCOntiniousQueue);
        }
  }
}
///////////////////////////LEDInit //////////////////////////////
void LedInit(){
esp_rom_gpio_pad_select_gpio(CONFIG_GREEN_LED_GPIO);
gpio_set_direction(CONFIG_GREEN_LED_GPIO, GPIO_MODE_OUTPUT);

esp_rom_gpio_pad_select_gpio(CONFIG_RED_LED_GPIO);
gpio_set_direction(CONFIG_RED_LED_GPIO, GPIO_MODE_OUTPUT);

esp_rom_gpio_pad_select_gpio(CONFIG_BLUE_LED_GPIO);
gpio_set_direction(CONFIG_BLUE_LED_GPIO, GPIO_MODE_OUTPUT);

gpio_set_level(CONFIG_BLUE_LED_GPIO, 0);
gpio_set_level(CONFIG_RED_LED_GPIO, 0); 
gpio_set_level(CONFIG_GREEN_LED_GPIO, 0);

}
///////////////////////END///////////////////////////////////////
void AutoProvisionTask()
{
    time_t CurrentTime, Timeout;
    struct tm* local_time;
    char LoraData[255];
    char LoraDataACK[255];
    vTaskSuspend(NULL);
    CurrentTime = time(NULL);
    Timeout = CurrentTime + 500;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while(1)
    {
        esp_task_wdt_reset();
        CurrentTime = time(NULL);
        gpio_set_level(CONFIG_GREEN_LED_GPIO, 1);
        gpio_set_level(CONFIG_RED_LED_GPIO, 1);
        gpio_set_level(CONFIG_BLUE_LED_GPIO, 0);
         if (TaskResetDecision == 0){
            printf("entere user reset");
        esp_err_t res2 = esp_task_wdt_reset_user(LoraRXContiniousTaskHandleWDT);
        if (res2 != ESP_OK) {
            printf("Failed to reset TWDT for RX continpus mode\n");
        }
        }

        esp_err_t LoraDataStatus = LoraSingleModeRX(1000, LoraSingleRXLength, LoraData);
        if (LoraDataStatus == ESP_OK) {
            char* token = strtok(LoraData, ",)");
            if (token != NULL && strcmp(token, NEMS_ID) == 0) {
                token = strtok(NULL, ",)");
                if (token != NULL) {
                    int pDevID = atoi(token);
                    token = strtok(NULL, ",)");
                    if (token != NULL) {
                        int rHubID = atoi(token);
                        token = strtok(NULL, ",)");
                        const char* receivedMessage = token ? token : "";

                        printf("Parsed values: NEMS_ID = %s, PDevID = %d, RHubID = %d, Message = %s\n",
                               NEMS_ID, pDevID, rHubID, receivedMessage);

                        char* lora_string = CreateLoraMessage(ConfigDevID, rHubID, NULL,20);
                        printf("CreatedLora string for sending line 620 is %s",lora_string);
                        if (lora_string != NULL) {
                            vTaskDelay(40);
                            for (int i = 0; i < 5; i++) {
                                vTaskDelay(40);
                                esp_task_wdt_reset();
                                esp_err_t SendResult = SendMessageWithCAD(lora_string);
                                while (SendResult != ESP_OK) {
                                    SendResult = SendMessageWithCAD(lora_string);
                                }
                            }
                            for (int i = 0; i < 5; i++) {
                                esp_task_wdt_reset();
                                  if (TaskResetDecision == 0){
                                        printf("entere user reset");
                                    esp_err_t res2 = esp_task_wdt_reset_user(LoraRXContiniousTaskHandleWDT);
                                    if (res2 != ESP_OK) {
                                        printf("Failed to reset TWDT for RX continpus mode\n");
                                    }
                                    }
                                esp_err_t LoraDataStatusACK = LoraSingleModeRX(1000, LoraSingleRXLength, LoraDataACK);
                                if (LoraDataStatusACK == ESP_OK) {
                                    if (LoraDataACK[0] != '\0') {
                                        printf("Lora message is %s\n", LoraDataACK);
                                        token = strtok(LoraDataACK, ",)");
                                        if (token != NULL && strcmp(token, NEMS_ID) == 0) {
                                            token = strtok(NULL, ",)");
                                            if (token != NULL) {
                                                int pDevID = atoi(token);
                                                token = strtok(NULL, ",)");
                                                if (token != NULL) {
                                                    int rHubID = atoi(token);
                                                    token = strtok(NULL, ",)");
                                                    const char* receivedMessage = token ? token : "";

                                                    printf("Parsed ACK values: NEMS_ID = %s, PDevID = %d, RHubID = %d, Message = %s\n",
                                                           NEMS_ID, pDevID, rHubID, receivedMessage);

                                                    if (token != NULL && strcmp(receivedMessage, "ACK") == 0 && pDevID == ConfigDevID) {
                                                        printf("Handling ACK message...\n");
                                                        printf("DEVi ID (int): %d\n", pDevID);
                                                        printf("HUBID (int): %d\n", rHubID);

                                                        if (rHubID != 0) {
                                                            esp_err_t RetValue2 = save_int_to_nvs(HUBKeyMain, rHubID);
                                                            if (RetValue2 == ESP_OK) {
                                                                lora_end_rx();
                                                                esp_restart();
                                                                vTaskDelay(200);
                                                            }
                                                        }
                                                    } else {
                                                        printf("String value is not 'ACK'.\n");
                                                    }
                                                } else {
                                                    printf("Invalid message format (missing second integer).\n");
                                                }
                                            } else {
                                                printf("Invalid message format (missing first integer).\n");
                                            }
                                        } else {
                                            printf("No 'NEMS_ID' found in ACK data.\n");
                                        }
                                    }
                                }
                            }
                            free(lora_string);
                        } else {
                            printf("Failed to create LoRa message.\n");
                        }
                    }
                }
            } else {
                printf("Invalid NEMS_ID or no NEMS_ID found in received data.\n");
            }
        } else {
            printf("Error receiving data from LoRa.\n");
        }

        local_time = localtime(&CurrentTime);
        printf("Current time: %s", asctime(local_time));
        printf("Timeout time: %lld\n", Timeout);
        esp_task_wdt_reset();
       
        if ((CurrentTime > Timeout) && (Timeout != 0)) {
            lora_end_rx();
            printf("Auto Provision ended due to 3 Minute timeout\n");
            ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
            // ErrorProvisionLight(); run as a task
            if (TaskResetDecision == 0){
            vTaskResume(LoraRXContiniousTaskHandle);
            vTaskDelay(50);
            }
            vTaskSuspend(NULL);
            CurrentTime = time(NULL);
            Timeout = CurrentTime + 20;
            ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
        }
    }
}

static void init_littlefs()
{

    esp_vfs_littlefs_conf_t conf = {
        .base_path = LITTLEFS_BASE_PATH,
        .partition_label = "storage",
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret == ESP_FAIL) {
    ESP_LOGE(TAG, "Failed to mount LittleFS. Attempting to format...");
    ret = esp_littlefs_format("storage");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format LittleFS");
        return;
    }
    ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS after formatting");
        return;
    }
}
}

//////////////////////Temperature data message formation //////////////////
void Temperaturedata(void) {
    // Convert integer temperature to float (assuming 2 decimal places precision)
    float temperatureF = (float)HDC1080_temp_data() / 100.0f;
    float humidityF = (float)HDC1080_humid_data() / 100.0f;
    
    printf("Temperature: %.2f, Humidity: %.2f\n", temperatureF, humidityF);

    char Temperaturechar[MAX_FLOAT_STR_LEN];
    char Humidity[MAX_FLOAT_STR_LEN];
    char settemperature[MAX_FLOAT_STR_LEN];
    const char* Battery = "30";  // Kept as const char*

    floatToString(temperatureF, Temperaturechar, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
    floatToString(humidityF, Humidity, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
    floatToString(set_temperature, settemperature, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);

    size_t message_len = strlen(Temperaturechar) + strlen(Humidity) + 
                         strlen(settemperature) + strlen(Battery) + 10;  // Extra space for separators and null terminator
    char* GenMessageTemp = malloc(message_len);
    
    if (GenMessageTemp == NULL) {
        printf("Memory allocation failed for GenMessageTemp\n");
        return;
    }

    snprintf(GenMessageTemp, message_len, "T%s@%s@%s@%s", 
             Temperaturechar, settemperature, Humidity, Battery);

    printf("Temperaturedata: %s\n", GenMessageTemp);

    if (GenMessageTemp) {
        printf("Temperature to Generate str func %s\n", GenMessageTemp);
        esp_err_t result  = send_thermostat_message_with_retry(GenMessageTemp);
        if (result == ESP_OK){
         printf("Sucess to create Lora message\n");// Assuming CreateLoraMessage allocates memory
        }
    } else {
        printf("Failed to create Lora message\n");
    }

    free(GenMessageTemp);
}
///////////////////////END////////////////////////////////////////////////


void app_main()
{
    esp_err_t err;
    printf("*******************************\n");
    printf("*******Device ID and INIt Thermo ID check Starts********\n");
    printf("*******************************\n");
    esp_err_t CheckDEVStatus = CheckStoredKeyStatus(DEVKeyMain, &ValueMain);
    if(CheckDEVStatus == 4354 || ValueMain != ConfigDevID )
    {
      esp_err_t RetDevValue = save_int_to_nvs(DEVKeyMain,ConfigDevID);
      if (RetDevValue == ESP_OK)
      {
        esp_restart();
      }  
    }
    err = read_int_from_nvs(DEVKeyMain, &StoredDevID);
    if (err != ESP_OK) {
        ESP_LOGE("StoreID", "Failed to read DevID from NVS");
    }
    ESP_LOGI(TAG, "DevID written: %d", StoredDevID);
    err = read_int_from_nvs(HUBKeyMain, &StoredHubID);
    if (err != ESP_OK) {
        ESP_LOGE("HubID", "Failed to read HubID from NVS");
    }
    ESP_LOGI(TAG, "DevID written: %d", StoredHubID);
    printf("*******************************\n");
    printf("*******Device ID check Stop********\n");
    printf("*******************************\n");

    printf("*******************************\n");
    printf("*******Lora Init Starts********\n");
    printf("*******************************\n");
   err = lora_init();
  if (err != ESP_OK)
  {
    ESP_LOGI("lora_init", "lora_init %d", err);
    return;
  }
  LedInit();
    printf("*****************************\n");
    printf("*******Lora Init Ends********\n");
    printf("*****************************\n");
  ///////////// Initalize watchdog timmer//////////////////////////
    printf("*******************************\n");
    printf("*******WatchDog Init Starts****\n");
    printf("*******************************\n");
  #if !CONFIG_ESP_TASK_WDT_INIT
  printf("Configured");
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = TWDT_TIMEOUT_MS,
      .idle_core_mask = (1 << 2) - 1,    // Bitmask of all cores
      .trigger_panic = true,
  };
  ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
  printf("TWDT initialized\n");
  #endif 
    printf("*******************************\n");
    printf("*******WatchDog Init Ends*****\n");
    printf("*******************************\n");
 ///////////////////////END//////////////////////////////////////////

  //////////////////////////////Multi-Button////////////////////////
    printf("*****************************\n");
    printf("***MultiButton Init Starts***\n");
    printf("*****************************\n");
    SetupMultiButton();
    printf("*****************************\n");
    printf("****MultiButton Init Ends****\n");
    printf("*****************************\n"); 
///////////////////////////////////END///////////////////////////
  /////////////////////////Provision Check//////////////////////////////////////

    xTaskCreate(AutoProvisionTask, "AutoProvisionTask", 10096, NULL, 0, &AutoProvision);
    esp_err_t CheckStatus = CheckStoredKeyStatus(HUBKeyMain, &ValueMain);
    if(CheckStatus == 4354){
      esp_err_t RetValue1 = save_int_to_nvs(HUBKeyMain,InitValueMain);
      if (RetValue1 == ESP_OK){
        esp_restart();
      }  
    }
    snprintf(mqttTopic, sizeof(mqttTopic), "%d_P", ValueMain);
    printf("MQTT Topic is %s", mqttTopic);
    if(ValueMain != 100){
    //   printf("*******************************\n");
    //   printf("*******Auto Provision Starts***\n");
    //   printf("*******************************\n");
    //   TaskResetDecision = 1;
    //   vTaskResume(AutoProvision);
    }
    else
    {

      printf("********************************\n");
      printf("*******System Provisioned******\n");
      printf("********************************\n");
      TaskResetDecision = 0;
       for(int i=0;i<1;i++)
        {
            gpio_set_level(CONFIG_GREEN_LED_GPIO, 1);
            gpio_set_level(CONFIG_RED_LED_GPIO, 0);
            gpio_set_level(CONFIG_BLUE_LED_GPIO, 0); 
             vTaskDelay(30);  
            gpio_set_level(CONFIG_GREEN_LED_GPIO, 1);
            gpio_set_level(CONFIG_RED_LED_GPIO, 1);
            gpio_set_level(CONFIG_BLUE_LED_GPIO, 0);
            vTaskDelay(30);     
        }
      InitalizeProgram();
    }
 
 }
   
void InitalizeProgram(){
//////////////////////////Lora Recieving Mode init and Queue////////
    printf("*******************************\n");
    printf("**Receive Continiousn Starts***\n");
    printf("*******************************\n");


  RecivedLoraContiniousModeInit();
  LoraRXCOntiniousQueue = xQueueCreate(7, sizeof(txBuffer));
  if (LoraRXCOntiniousQueue == NULL)
  {
    ESP_LOGE("QUEUE", "Queue creation failed");
    return;
  }
  RXDoneSemaphore = xSemaphoreCreateBinary();
  MessageMutex = xSemaphoreCreateMutex();
  if (MessageMutex == NULL) {
      ESP_LOGE(TAG, "Failed to create mutex");
      return;
  }
  xTaskCreatePinnedToCore(LoraRXContiniousTask, "LoraRXContiniousTask", 4096, NULL, 3, &LoraRXContiniousTaskHandle,1);
  xTaskCreatePinnedToCore(RXContiniousMessageQueue, "RXContiniousMessageQueue", 4096, NULL, 2, &LoraRXContiniousMessageQueueTaskHandle,1);
    printf("*******************************\n");
    printf("**Receive Continiousn Srops***\n");
    printf("*******************************\n");

        printf("*******************************\n");
    printf("*******Temp Button check Start********\n");
    printf("*******************************\n");
       InitTempButton();
    printf("*******************************\n");
    printf("*******Temp Button check Stop********\n");
    printf("*******************************\n");

    // Check wake-up reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    print_wakeup_reason();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
        case ESP_SLEEP_WAKEUP_EXT1:
            // Handle button wake-up
            //InitTempButton();
            printf("Button press");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            // Handle ULP wake-up
            // Add your ULP handling code here
             printf("ULP");
            break;
        default:
            // First boot or unexpected wake-up
            // Initialize everything
           // InitTempButton();
            printf(" default Button press");
           
            // Add any other initialization needed
            break;
    }
    printf("*******************************\n");
    printf("**Transmit Queue Starts***\n");
    printf("*******************************\n");
     init_lora_queue();
    start_lora_queue_task();  
    printf("*******************************\n");
    printf("**Transmit Queue Stop***\n");
    printf("*******************************\n");
        /////////////////Initlize LiitleFS//////////////////////
    // Initialize LittleFS
   init_littlefs();
   send_lora_message(201, 100, "AT",30);
    ///////////////////////////END////////////////////
  ///////////////////////////////////END///////////////////////////
  //Temperaturedata();
}