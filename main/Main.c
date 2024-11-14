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

#define TWDT_TIMEOUT_MS         80000
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
#define MAX_RETRY_COUNT 1
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
void HandleMatchingMessage(int DevID, int HubID, const char* message);
bool ParseMessage(const char* input);

bool Thermostatdatasent = false;
bool ResponseStatus = false;
char* TAG = "Main";
int ConfigDevID = 100;
int OTACN = 0;
char NEMS_ID[20] = "NEMS";
/////////Lora RX Continious Mode//////////////// 
int TimeIntervalRX = 5;
const int OTARXMessageHub = 31;
const int OTAWifiAckTXMessageHub = 30;
const int WIFRXMessageLength =220;
const int ThermostatRXACK = 27;
uint8_t LoraContiRXLength[] = {ThermostatRXACK,WIFRXMessageLength,OTARXMessageHub,38,250};
char txBuffer[255];
////////Lora RX Single Mode//////////////// 
uint8_t LoraSingleRXLength =20;

char HUBKeyMain[10] = "HUBID";
char DEVKeyMain[10] = "DEVID";
char TempKeyMain[10] = "TempC";
char RelayKeyMain[10] = "RELAYID";
float set_temperature = 20.0f; // Default temperature, now a float
// char OTAC[10] = "OTAC";
int ValueMain;
char mqttTopic[20];
int InitValueMain =0;
uint32_t notification;
static char CurrentMessage[255];
int StoredDevID, StoredHubID,StroredTempCheck;
float Globaltemperature = 0.0;
int TaskResetDecision = 0;
static char g_message_id[3] = {0};
/////////////////////Temp Task//////////////
#define MAX_FLOAT_STR_LEN 10
#define FLOAT_PRECISION 2
#define STACK_SIZE 4096
#define TASK_PRIORITY 5

// Event group to signal task control
static EventGroupHandle_t temp_task_events;
#define TEMP_TASK_STOP_BIT BIT0
#define TEMP_TASK_SUSPEND_BIT BIT1

// Task handle for temperature monitoring
static TaskHandle_t temp_task_handle = NULL;

// Function prototypes
static void temperature_task(void* pvParameters);
esp_err_t start_temperature_monitoring(void);
esp_err_t stop_temperature_monitoring(void);
esp_err_t suspend_temperature_monitoring(void);
esp_err_t resume_temperature_monitoring(void);
esp_err_t floatToString(float value, char* buffer, int bufferSize, int precision) ;
static esp_err_t send_thermostat_message_with_retry(const char* message);
// Modified floatToString function with error handling
esp_err_t floatToString(float value, char* buffer, int bufferSize, int precision) {
    if (buffer == NULL || bufferSize <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int intPart = (int)value;
    float fracPart = fabsf(value - intPart);
    for (int i = 0; i < precision; i++) {
        fracPart *= 10;
    }

    int written = snprintf(buffer, bufferSize, "%d.%0*d", intPart, precision, (int)(fracPart + 0.5f));
    if (written < 0 || written >= bufferSize) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

// Updated temperature task
static void temperature_task(void* pvParameters) {
    while (1) {
        // Check for stop request
        EventBits_t bits = xEventGroupGetBits(temp_task_events);
        if (bits & TEMP_TASK_STOP_BIT) {
            ESP_LOGI(TAG, "Temperature task stopping");
            vTaskDelete(NULL);
            return;
        }

        // Get sensor data
        float temperatureF = 50.0; // Replace with HDC1080_temp_data();
        int humidityF = 50;        // Replace with HDC1080_humid_data();
        
        ESP_LOGI(TAG, "Temperature: %.2f, Humidity: %d", temperatureF, humidityF);
        
        // Allocate buffers on stack to save heap
        char Temperaturechar[MAX_FLOAT_STR_LEN];
        char settemperature[MAX_FLOAT_STR_LEN];
        char humidity_str[MAX_FLOAT_STR_LEN];
        const char* Battery = "30";
        bool conversion_error = false;

        // Convert values to strings with error checking
        esp_err_t temp_result = floatToString(temperatureF, Temperaturechar, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
        esp_err_t settemp_result = floatToString(set_temperature, settemperature, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
        int humid_result = snprintf(humidity_str, MAX_FLOAT_STR_LEN, "%d", humidityF);

        if (temp_result != ESP_OK || settemp_result != ESP_OK || humid_result < 0) {
            ESP_LOGE(TAG, "String conversion failed");
            conversion_error = true;
        }

        if (!conversion_error) {
            // Calculate required buffer size
            size_t message_len = strlen(Temperaturechar) + strlen(humidity_str) + 
                                strlen(settemperature) + strlen(Battery) + 10;

            // Allocate message buffer
            char* GenMessageTemp = heap_caps_malloc(message_len, MALLOC_CAP_8BIT);
            if (GenMessageTemp == NULL) {
                ESP_LOGE(TAG, "Memory allocation failed for GenMessageTemp");
                continue;
            }

            // Format message
            snprintf(GenMessageTemp, message_len, "R%s@%s@%s@%s", 
                    Temperaturechar, settemperature, humidity_str, Battery);
            
            // Send message
            esp_err_t result = send_thermostat_message_with_retry(GenMessageTemp);
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "Successfully sent: %s", GenMessageTemp);
            } else {
                ESP_LOGE(TAG, "Failed to send message");
                // Optionally suspend task on failure
                xEventGroupSetBits(temp_task_events, TEMP_TASK_SUSPEND_BIT);
            }

            // Free allocated memory
            heap_caps_free(GenMessageTemp);
        }

        // Check for suspend request
        bits = xEventGroupGetBits(temp_task_events);
        if (bits & TEMP_TASK_SUSPEND_BIT) {
            ESP_LOGI(TAG, "Temperature task suspending");
            vTaskSuspend(NULL);
        }

        // Task delay
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed
    }
}
// Start temperature monitoring task
esp_err_t start_temperature_monitoring(void) {
    if (temp_task_handle != NULL) {
        ESP_LOGE(TAG, "Task already running");
        return ESP_ERR_INVALID_STATE;
    }

    // Create event group
    temp_task_events = xEventGroupCreate();
    if (temp_task_events == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // Clear all bits
    xEventGroupClearBits(temp_task_events, TEMP_TASK_STOP_BIT | TEMP_TASK_SUSPEND_BIT);

    // Create task
    BaseType_t ret = xTaskCreate(
        temperature_task,
        "temp_task",
        STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        &temp_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        vEventGroupDelete(temp_task_events);
        temp_task_events = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Temperature monitoring started");
    return ESP_OK;
}

// Stop temperature monitoring task
esp_err_t stop_temperature_monitoring(void) {
    if (temp_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xEventGroupSetBits(temp_task_events, TEMP_TASK_STOP_BIT);
    vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to clean up

    vEventGroupDelete(temp_task_events);
    temp_task_events = NULL;
    temp_task_handle = NULL;

    return ESP_OK;
}

// Suspend temperature monitoring task
esp_err_t suspend_temperature_monitoring(void) {
    if (temp_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xEventGroupSetBits(temp_task_events, TEMP_TASK_SUSPEND_BIT);
    return ESP_OK;
}

// Resume temperature monitoring task
esp_err_t resume_temperature_monitoring(void) {
    if (temp_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xEventGroupClearBits(temp_task_events, TEMP_TASK_SUSPEND_BIT);
    vTaskResume(temp_task_handle);
    return ESP_OK;
}


///////END////////////////////////

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

// 
static TaskHandle_t hdc1080_task_handle = NULL;

// void process_and_send_hdc1080_data_task(void *pvParameters) {
//     while(1) {
//         // Suspend the task at the beginning of each iteration
//         vTaskSuspend(NULL);

//         // Retrieve temperature and humidity data from ULP
//         uint8_t temperature = 0;
//         uint8_t humidity = 0;
        
//         // Assuming we want to use the first non-zero values
//         for (uint8_t i = 0; (*(address_t + i) != 0); i++) {
//             temperature = (uint8_t) * (address_t + i);
//             humidity = (uint8_t) * (address_h + i);
//             break;  // Use the first non-zero values
//         }
        
//         // Convert uint8_t temperature to float (assuming 2 decimal places precision)
//         float temperatureF = (float)temperature / 100.0;  // Adjust this conversion as needed
//         int humidityF = humidity;  // Humidity is already an integer percentage
        
//         ESP_LOGI(TAG, "Temperature: %.2f, Humidity: %d", temperatureF, humidityF);

//         char Temperaturechar[MAX_FLOAT_STR_LEN];
//         char settemperature[MAX_FLOAT_STR_LEN];
//         const char* Battery = "30";  // Kept as const char*

//         floatToString(temperatureF, Temperaturechar, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
//         char* Humidity = IntStrCoverter(humidityF, 3);
//         floatToString(set_temperature, settemperature, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);

//         size_t message_len = strlen(Temperaturechar) + strlen(Humidity) + 
//                              strlen(settemperature) + strlen(Battery) + 10;  // Extra space for separators and null terminator
//         char* GenMessageTemp = malloc(message_len);
        
//         if (GenMessageTemp == NULL) {
//             ESP_LOGE(TAG, "Memory allocation failed for GenMessageTemp");
//             free(Humidity);
//             continue;  // Skip to the next iteration
//         }

//         snprintf(GenMessageTemp, message_len, "H%s@%s@%s@%s", 
//                  Temperaturechar, settemperature, Humidity, Battery);

//         ESP_LOGI(TAG, "HDC1080 data: %s", GenMessageTemp);
        
//             esp_err_t result = send_thermostat_message_with_retry(GenMessageTemp);
            
//             if (result == ESP_OK) {
//                 ESP_LOGI(TAG, "Successfully sent HDC1080 data via Lora");
//             } else {
//                 ESP_LOGE(TAG, "Failed to send HDC1080 data via Lora");
//             }
       
//         free(GenMessageTemp);
//         free(Humidity);
//        // ulp_component(0);
//     }
// }

// Function to create and start the HDC1080 task
// void start_hdc1080_task() {
//     xTaskCreate(process_and_send_hdc1080_data_task, "HDC1080_Task", 4096, NULL, 5, &hdc1080_task_handle);
// }

// Function to resume the HDC1080 task
void resume_hdc1080_task() {
    if (hdc1080_task_handle != NULL) {
        vTaskResume(hdc1080_task_handle);
    }
}
void RXContiniousMessageQueue(void *arg)
{
    ESP_LOGI(TAG, "RXContiniousMessageQueue task started");
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    
    char rxBuffer[255];
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms frequency

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xQueueReceive(LoraRXCOntiniousQueue, rxBuffer, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Received Message From LoraRXContiniousTask : %s", rxBuffer);
            if (xSemaphoreTake(MessageMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                strncpy(CurrentMessage, rxBuffer, sizeof(CurrentMessage) - 1);
                CurrentMessage[sizeof(CurrentMessage) - 1] = '\0'; // Ensure null-termination
                if (ParseMessage(CurrentMessage)) {
                    ESP_LOGI(TAG, "Message processed successfully");
                } else {
                    ESP_LOGW(TAG, "Message not processed (no match or error)");
                }
                xSemaphoreGive(MessageMutex);
            } else {
                ESP_LOGW(TAG, "Failed to acquire MessageMutex");
            }
        } else {
            ESP_LOGV(TAG, "No message in queue"); // Verbose logging
        }

        memset(rxBuffer, 0, sizeof(rxBuffer));
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
            
            char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, OTAWifiAckTXMessageHub);
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

void HandleWiFiChangeMessage(const char* ssid, const char* password, const char* message_id, int DevID, int HubID) {
    ESP_LOGI(TAG, "Handling WiFi change message. SSID: %s, Password: %s", ssid, password);
    StoreHardcodeWiFiData(ssid,password);
    char ack_message[20];
    snprintf(ack_message, sizeof(ack_message), "%s|ACK", message_id);
    
    char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, OTAWifiAckTXMessageHub);
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
void HandleTemperatureMessage(const char* message_id ,int DevID, int HubID, const char* temperature_str) {
    if (temperature_str == NULL) {
        ESP_LOGE(TAG, "Received NULL temperature string");
        return;
    }

    ESP_LOGI(TAG, "Received temperature from Dev %d, Hub %d: %s", DevID, HubID, temperature_str);
    char ack_message[20];
    snprintf(ack_message, sizeof(ack_message), "%s|ACK", message_id);
    char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, OTAWifiAckTXMessageHub);
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
    // Convert temperature string to float
    float temperature = atof(temperature_str);

    ESP_LOGI(TAG, "Parsed temperature: %.1f", temperature);
    esp_err_t RetDevValueT = save_float_to_nvs("set_temp_f", temperature);
    if (RetDevValueT != ESP_OK)
    {
        RetDevValueT = save_float_to_nvs("set_temp_f", temperature);
    }
    float Rtemperature;
    esp_err_t ret = read_float_from_nvs("set_temp_f", &Rtemperature);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Successfully read temperature from NVS: %.2f", Rtemperature);
        set_temperature = Rtemperature;
        // Use the temperature value here
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Temperature value not found in NVS");
    } else {
        ESP_LOGE(TAG, "Error reading temperature from NVS: %s", esp_err_to_name(ret));
    }
    resume_hdc1080_task();
}

void HandleFMBMMessage(const char* message_id, int DevID, int HubID, const char* message_type, const char* content) {
  char ack_message[20];
    snprintf(ack_message, sizeof(ack_message), "%s|ACK", message_id);
    char* lora_message = CreateLoraMessage(DevID, HubID, ack_message, OTAWifiAckTXMessageHub);
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
    if (strcmp(message_type, "FM") == 0) {
        ESP_LOGI(TAG, "Forward Message - ID: %s, Dev: %d, Hub: %d, Content: %s", message_id, DevID, HubID, content);
    } else if (strcmp(message_type, "BM") == 0) {
        ESP_LOGI(TAG, "Backward Message - ID: %s, Dev: %d, Hub: %d, Content: %s", message_id, DevID, HubID, content);
    }
    //ulp_component(0);
}




// Pre-define common string literals to save flash memory
static const char RS_MARKER[] = "RS|";
static const char ACK_MARKER[] = "|ACK|";
static const char DELIMITER = '|';

inline static char* find_next_delimiter(const char* str) {
    return strchr(str, DELIMITER);
}

inline static float fast_atof(const char* str) {
    float result = 0.0f;
    float fraction = 0.0f;
    bool negative = false;
    
    // Handle negative numbers
    if (*str == '-') {
        negative = true;
        str++;
    }
    
    // Parse integer part
    while (*str >= '0' && *str <= '9') {
        result = result * 10.0f + (*str - '0');
        str++;
    }
    
    // Parse decimal part
    if (*str == '.') {
        str++;
        float multiplier = 0.1f;
        while (*str >= '0' && *str <= '9') {
            result += (*str - '0') * multiplier;
            multiplier *= 0.1f;
            str++;
        }
    }
    
    return negative ? -result : result;
}
void save_environmental_data(float room_temp, int humidity, float set_temp, const char* date_str, const char* time_str) {
    // Save to NVS
    save_float_to_nvs("room_temp_f", room_temp);
    save_int_to_nvs("humidity_f", humidity);
    save_float_to_nvs("set_temp_f", set_temp);
    save_string_to_nvs("date_str", date_str);
    save_string_to_nvs("time_str", time_str);

    ESP_LOGD(TAG, "Updated and saved to NVS: %.1f|%d|%.1f|%s|%s",
             room_temp, humidity, set_temp, date_str, time_str);
}

// void HandleRSMessage(const char* message) {
//     if (!message || strlen(message) < 5) {  // Minimum valid length check
//         ESP_LOGE(TAG, "Invalid message");
//         return;
//     }
//     ResponseStatus = true;
//     // Quick validation of message ID format (2 digits + delimiter)
//     if (!isdigit((unsigned char)message[0]) || 
//         !isdigit((unsigned char)message[1]) || 
//         message[2] != DELIMITER) {
//         ESP_LOGE(TAG, "Invalid message ID format");
//         return;
//     }

//     // Handle ACK if present (using direct comparison instead of strstr)
//     if (memcmp(message + 3, ACK_MARKER + 1, 4) == 0) {  // +1 to skip the first '|' in ACK_MARKER
//         HandleAckMessage(message);
//     }

//     // Find RS marker (using direct comparison instead of strstr)
//     const char* rs_start = message;
//     bool found = false;
//     while (*rs_start) {
//         if (memcmp(rs_start, RS_MARKER, 3) == 0) {
//             found = true;
//             break;
//         }
//         rs_start++;
//     }

//     if (!found) {
//         ESP_LOGE(TAG, "RS marker not found");
//         return;
//     }

//     // Move pointer to start of values
//     rs_start += 3;  // Skip "RS|"

//     // Parse values directly from the string without temporary buffers
//     const char *current = rs_start;
//     const char *next;
//     float values[3];  // [setTemp, roomTemp, humidity]
//     char date[11];    // DD/MM/YY\0
//     char time[8];     // HH:MMAM\0
    
//     // Parse floating point values
//     for (int i = 0; i < 3; i++) {
//         next = find_next_delimiter(current);
//         if (!next) {
//             ESP_LOGE(TAG, "Missing delimiter after value %d", i);
//             return;
//         }
        
//         // Validate that we have a valid number
//         if (next - current > 10) {  // Reasonable max length for a float
//             ESP_LOGE(TAG, "Value %d too long", i);
//             return;
//         }
        
//         values[i] = fast_atof(current);
//         current = next + 1;
//     }

//     // Parse date
//     next = find_next_delimiter(current);
//     if (!next || (next - current) > 9) {  // DD/MM/YY
//         ESP_LOGE(TAG, "Invalid date format");
//         return;
//     }
//     memcpy(date, current, next - current);
//     date[next - current] = '\0';
//     current = next + 1;

//     // Parse time (look for ending parenthesis or null)
//     next = strchr(current, ')');
//     if (!next) next = strchr(current, '\0');
//     if (!next || (next - current) > 7) {  // HH:MMAM
//         ESP_LOGE(TAG, "Invalid time format");
//         return;
//     }
//     memcpy(time, current, next - current);
//     time[next - current] = '\0';

//     // // Update values only if all parsing was successful
//     // roomTemperature_partial_update(values[1]);
//     // humidity_partial_update(values[2]);
//     // date_partial_update(date);
//     // time_partial_update(time);
//     save_environmental_data(values[1],(int)values[2],set_temperature,date,time);
//     ESP_LOGD(TAG, "Parsed: %.1f|%.1f|%.1f|%s|%s", 
//              values[0], values[1], values[2], date, time);
// }

void save_display_messages(const char* front_msg, const char* back_msg) {
    if (front_msg) {
        save_string_to_nvs("front_msg", front_msg);
    }
    if (back_msg) {
        save_string_to_nvs("back_msg", back_msg);
    }
    ESP_LOGD(TAG, "Messages saved - Front: %s, Back: %s", 
             front_msg ? front_msg : "NULL", 
             back_msg ? back_msg : "NULL");
}

void HandleRSMessage(const char* message) {
    if (!message || strlen(message) < 5) {
        ESP_LOGE(TAG, "Invalid message");
        return;
    }
    ResponseStatus = true;
    if (!isdigit((unsigned char)message[0]) || 
        !isdigit((unsigned char)message[1]) || 
        message[2] != DELIMITER) {
        ESP_LOGE(TAG, "Invalid message ID format");
        return;
    }

    if (memcmp(message + 3, ACK_MARKER + 1, 4) == 0) {
        HandleAckMessage(message);
    }

    const char* rs_start = message;
    bool found = false;
    while (*rs_start) {
        if (memcmp(rs_start, RS_MARKER, 3) == 0) {
            found = true;
            break;
        }
        rs_start++;
    }

    if (!found) {
        ESP_LOGE(TAG, "RS marker not found");
        return;
    }

    rs_start += 3;  // Skip "RS|"

    const char *current = rs_start;
    const char *next;
    float values[3];
    char date[11];
    char time[8];
    char front_message[256] = {0};  // For Welcome message
    char back_message[256] = {0};   // For Thank you message
    
    // Parse floating point values
    for (int i = 0; i < 3; i++) {
        next = find_next_delimiter(current);
        if (!next) {
            ESP_LOGE(TAG, "Missing delimiter after value %d", i);
            return;
        }
        
        if (next - current > 10) {
            ESP_LOGE(TAG, "Value %d too long", i);
            return;
        }
        
        values[i] = fast_atof(current);
        current = next + 1;
    }

    // Parse date
    next = find_next_delimiter(current);
    if (!next || (next - current) > 9) {
        ESP_LOGE(TAG, "Invalid date format");
        return;
    }
    memcpy(date, current, next - current);
    date[next - current] = '\0';
    current = next + 1;

    // Parse time
    next = find_next_delimiter(current);
    if (!next || (next - current) > 7) {
        ESP_LOGE(TAG, "Invalid time format");
        return;
    }
    memcpy(time, current, next - current);
    time[next - current] = '\0';
    current = next + 1;

    // Parse front message (Welcome)
    next = find_next_delimiter(current);
    if (next && (next - current) < sizeof(front_message)) {
        memcpy(front_message, current, next - current);
        front_message[next - current] = '\0';
        current = next + 1;
    }

    next = strchr(current, ')');
    if (next && (next - current) < sizeof(back_message)) {
        memcpy(back_message, current, next - current);
        back_message[next - current] = '\0';
    }

    // Save environmental data
    save_environmental_data(values[1], (int)values[2], set_temperature, date, time);
    
    // Save display messages
    save_display_messages(front_message, back_message);

    ESP_LOGD(TAG, "Parsed: %.1f|%.1f|%.1f|%s|%s|%s|%s", 
             values[0], values[1], values[2], date, time, 
             front_message, back_message);
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
        if (strlen(message_content) >= 5 && 
                message_content[0] == 'T' && 
                message_content[1] == '|' && 
                strcmp(message_content + 2, "ACK") == 0) {
                    
                ESP_LOGI(TAG, "ACK Status: true");
                Thermostatdatasent = true;
                char new_message[8];
                snprintf(new_message, sizeof(new_message), "%s|ACK", message_id);
                HandleAckMessage(new_message);
                xSemaphoreGive(MessageMutex);
                return;
            } 
        if (strstr(message_content, "RS|")) {
            HandleRSMessage(message);
            xSemaphoreGive(MessageMutex);
            return;
        }
                // Check if the remaining message starts with "ST"
        if (strncmp(message_content, "ST", 2) == 0) {
            // Extract the temperature value (everything after "ST")
            const char* temperature_str = message_content + 2;
            HandleTemperatureMessage(message_id, DevID, HubID, temperature_str);
            xSemaphoreGive(MessageMutex);
            return;
        }
        if (strncmp(message_content, "FM", 2) == 0 || strncmp(message_content, "BM", 2) == 0) {
            char message_type[3] = {message_content[0], message_content[1], '\0'};
            const char* content = message_content + 2;
            HandleFMBMMessage(message_id, DevID, HubID, message_type, content);
            xSemaphoreGive(MessageMutex);
            return;
        }
        if (strncmp(message_content, "ACK", 4) == 0) {
            HandleAckMessage(message);  // Pass the full message
            xSemaphoreGive(MessageMutex);
            return;  // Exit after handling ACK
        } 
    }

    // Check if the message is an OTA message
    if (strncmp(message, "OTA", 3) == 0) {
        ESP_LOGI(TAG, "OTA Message: %s", message);
        HandleOTAMessage(message_id, DevID, HubID);
        xSemaphoreGive(MessageMutex);
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
                xSemaphoreGive(MessageMutex);
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
void Temperaturedata(void) {
    // Convert integer temperature to float (assuming 2 decimal places precision)
    // float temperatureF = HDC1080_temp_data();
    // int humidityF = HDC1080_humid_data();
    float temperatureF = 50.0;
    int humidityF = 50;
    
    printf("Temperature: %.2f, Humidity: %d\n", temperatureF, humidityF);

    char Temperaturechar[MAX_FLOAT_STR_LEN];
    //char Humidity[MAX_FLOAT_STR_LEN];
    char settemperature[MAX_FLOAT_STR_LEN];
    const char* Battery = "30";  // Kept as const char*

    floatToString(temperatureF, Temperaturechar, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
    char* Humidity = IntStrCoverter(humidityF,3);
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
        for(int Retry = 0; ((Retry < ACK_RETRY_COUNT) && (Thermostatdatasent == false)); Retry++){
        esp_err_t result  = send_thermostat_message_with_retry(GenMessageTemp);
        if ((result == ESP_OK )&& (Thermostatdatasent == true)){
         printf("Sucess to create Lora message\n");// Assuming CreateLoraMessage allocates memory
         Thermostatdatasent = false;
         break;
        }
        }
    } else {
        printf("Failed to create Lora message\n");
    }

    free(GenMessageTemp);
    free(Humidity);
    gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
}

//////////////////////Temperature data message formation //////////////////
void TemperaturedataRS(void) {
    // Convert integer temperature to float (assuming 2 decimal places precision)
    // float temperatureF = HDC1080_temp_data();
    // int humidityF = HDC1080_humid_data();
    float temperatureF = 50.0;
    int humidityF = 50;
    
    printf("Temperature: %.2f, Humidity: %d\n", temperatureF, humidityF);

    char Temperaturechar[MAX_FLOAT_STR_LEN];
    //char Humidity[MAX_FLOAT_STR_LEN];
    char settemperature[MAX_FLOAT_STR_LEN];
    const char* Battery = "30";  // Kept as const char*

    floatToString(temperatureF, Temperaturechar, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);
    char* Humidity = IntStrCoverter(humidityF,3);
    floatToString(set_temperature, settemperature, MAX_FLOAT_STR_LEN, FLOAT_PRECISION);

    size_t message_len = strlen(Temperaturechar) + strlen(Humidity) + 
                         strlen(settemperature) + strlen(Battery) + 10;  // Extra space for separators and null terminator
    char* GenMessageTemp = malloc(message_len);
    
    if (GenMessageTemp == NULL) {
        printf("Memory allocation failed for GenMessageTemp\n");
        return;
    }

    snprintf(GenMessageTemp, message_len, "R%s@%s@%s@%s", 
             Temperaturechar, settemperature, Humidity, Battery);

    printf("Temperaturedata: %s\n", GenMessageTemp);

    if (GenMessageTemp) {
        printf("Temperature to Generate str func %s\n", GenMessageTemp);
        for(int Retry = 0; ((Retry < ACK_RETRY_COUNT) && (ResponseStatus == false)); Retry++){
        esp_err_t result  = send_thermostat_message_with_retry(GenMessageTemp);
        if ((result == ESP_OK )&& (ResponseStatus == true)){
         printf("Sucess to create Lora message\n");// Assuming CreateLoraMessage allocates memory
         ResponseStatus = false;
         break;
        }
        }
    } else {
        printf("Failed to create Lora message\n");
    }

    free(GenMessageTemp);
    free(Humidity);
    gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
}

void InitMain()
{
    esp_err_t err;
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
 }


 void check_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask & (1ULL << CONFIG_BUTTON_INC_GPIO)) {
                ESP_LOGI("ButtonTag", "Wake up from BUTTON_INC");
            } else if (wakeup_pin_mask & (1ULL << CONFIG_BUTTON_DEC_GPIO)) {
                ESP_LOGI("ButtonTag", "Wake up from BUTTON_DEC");
            }
            Display_Main();
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wakeup caused by timer");
            InitMain();
            esp_err_t err2 = lora_init();
            if (err2 != ESP_OK)
            {
                ESP_LOGI("lora_init", "lora_init %d", err2);
                return;
            }
            InitalizeProgram();
            TemperaturedataRS();
            enter_sleep_mode_Timmer();
            break;
        default:
            ESP_LOGI("ButtonTag", "Wake up not caused by deep sleep: %d", wakeup_reason);
            InitMain();
            esp_err_t err = lora_init();
            if (err != ESP_OK)
            {
                ESP_LOGI("lora_init", "lora_init %d", err);
                return;
            }
            InitalizeProgram();
            TemperaturedataRS();
            Inactive_message_Init();
            enter_sleep_mode();
            break;
    }
}
   
void InitalizeProgram(){
       
//////////////////////////Lora Recieving Mode init and Queue////////
  init_littlefs();
  init_lora_queue();
  start_lora_queue_task();  
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
  xTaskCreatePinnedToCore(LoraRXContiniousTask, "LoraRXContiniousTask", 8096, NULL, 3, &LoraRXContiniousTaskHandle,0);
  xTaskCreatePinnedToCore(RXContiniousMessageQueue, "RXContiniousMessageQueue", 9096, NULL, 1, &LoraRXContiniousMessageQueueTaskHandle,1);
}

void app_main(void){
    check_wakeup_reason();
}