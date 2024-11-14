#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "xag_wifi_component.h"
#include "xag_nvs_component.h"
#include "Config.h"
#include "MultiButtonHeader.h"
#include "lora.h"
#include"CJson.h"
#include"ProvisionHeader.h"
#include"xag_mqtt_component.h"
#include "cJSON.h"
#include "esp_littlefs.h"
#include "esp_sntp.h"
#include <time.h>
#include "esp_attr.h"
#include <sys/stat.h>
#include <dirent.h>
#include <math.h>
#include "nvs_flash.h"



#include "cJSON.h"

#define ACK_FILE_PATH "/littlefs/ack_statuses.json"
#define ACK_RETRY_COUNT 2
#define ACK_RETRY_DELAY_MS 1000
#define ACK_CHECK_DELAY_MS 2000
#define MAX_QUEUE_SIZE 10
#define MAX_MESSAGE_SIZE 255
#define LORA_MESSAGE_LENGTH 30 
#define QUEUE_WAIT_TIME_MS 1000

char* LoraCommTag = "Main";

// Function to load or create the ACK JSON file
cJSON* load_ack_json() {
    FILE* f = fopen(ACK_FILE_PATH, "r");
    if (f == NULL) {
        ESP_LOGW(LoraCommTag, "ACK file not found, creating new JSON object");
        return cJSON_CreateObject();
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize == 0) {
        ESP_LOGW(LoraCommTag, "ACK file is empty, creating new JSON object");
        fclose(f);
        return cJSON_CreateObject();
    }

    // Allocate memory and read the file
    char* content = malloc(fsize + 1);
    if (content == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to allocate memory for file content");
        fclose(f);
        return cJSON_CreateObject();
    }

    size_t read_size = fread(content, 1, fsize, f);
    fclose(f);

    if (read_size != fsize) {
        ESP_LOGW(LoraCommTag, "File read size mismatch, expected %ld, got %d", fsize, read_size);
    }

    content[read_size] = 0; // Null terminate the string

    // Parse JSON
    cJSON* json = cJSON_Parse(content);
    free(content);

    if (json == NULL) {
        const char* error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGE(LoraCommTag, "JSON Parse Error: %s", error_ptr);
        }
        return cJSON_CreateObject();
    }

    ESP_LOGI(LoraCommTag, "Successfully loaded ACK JSON");
    return json;
}

void save_ack_json(cJSON* json) {
    char* json_str = cJSON_Print(json);
    if (json_str == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to print JSON to string");
        return;
    }

    FILE* f = fopen(ACK_FILE_PATH, "w");
    if (f == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to open ACK file for writing");
        free(json_str);
        return;
    }

    size_t written = fwrite(json_str, 1, strlen(json_str), f);
    fclose(f);

    if (written != strlen(json_str)) {
        ESP_LOGW(LoraCommTag, "File write size mismatch, expected %d, wrote %d", strlen(json_str), written);
    }

    free(json_str);
    ESP_LOGI(LoraCommTag, "ACK JSON saved successfully");
}

// Function to create or update an ACK entry
void create_or_update_ack(const char* message_id, const char* status) {
    cJSON* json = load_ack_json();
    cJSON_AddStringToObject(json, message_id, status);
    save_ack_json(json);
    cJSON_Delete(json);
    ESP_LOGI("ACK", "Updated ACK status for message %s: %s", message_id, status);
}


static QueueHandle_t lora_queue = NULL;

// Function to initialize the LoRa message queue
void init_lora_queue(void) {
    lora_queue = xQueueCreate(MAX_QUEUE_SIZE, MAX_MESSAGE_SIZE);
    if (lora_queue == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to create LoRa message queue");
    }
}

// Function to add a message to the queue
esp_err_t queue_lora_message(const char* message) {
    if (lora_queue == NULL) {
        ESP_LOGE(LoraCommTag, "LoRa queue not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (strlen(message) >= MAX_MESSAGE_SIZE) {
        ESP_LOGE(LoraCommTag, "Message too long for queue");
        return ESP_ERR_INVALID_SIZE;
    }

    if (xQueueSend(lora_queue, message, 0) != pdTRUE) {
        ESP_LOGE(LoraCommTag, "Failed to queue LoRa message. Queue might be full.");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

// Task to process the LoRa message queue
void lora_queue_task(void *pvParameters) {
    char message[MAX_MESSAGE_SIZE];
    while (1) {
        if (xQueueReceive(lora_queue, message, portMAX_DELAY) == pdTRUE) {
            esp_err_t result;
            do {
                result = SendMessageWithCAD(message);
                RecivedLoraContiniousModeInit();
                if (result == ESP_OK) {
                    ESP_LOGI(LoraCommTag, "Message sent successfully: %s", message);
                } else {
                    ESP_LOGW(LoraCommTag, "Failed to send message, retrying: %s", esp_err_to_name(result));
                     // Wait for 1 second before retrying
                }
                vTaskDelay(pdMS_TO_TICKS(1000));

            } while (result != ESP_OK);
        }
    }
}

// Function to start the LoRa queue task
void start_lora_queue_task(void) {
    xTaskCreatePinnedToCore(lora_queue_task, "lora_queue_task", 8192, NULL,2, NULL,1);
}



/////////////////////////SEND MessaGE with ACK /////////////////////////////////
esp_err_t send_message_with_ack(int DevID, int HubID, const char* message, int Length) {
    char message_id[3];
    cJSON *json = NULL;
    esp_err_t result = ESP_FAIL;
    char* lora_message = NULL;

    // Load existing ACK JSON file
    json = load_ack_json();
    if (json == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to load ACK JSON file");
        return ESP_FAIL;
    }

    // Generate a unique message ID
    do {
        int message_id_int = (esp_random() % 99) + 1;
        snprintf(message_id, sizeof(message_id), "%02d", message_id_int);
    } while (cJSON_GetObjectItem(json, message_id) != NULL);

    // Add new message ID to JSON
    cJSON_AddStringToObject(json, message_id, "WAITING");
    save_ack_json(json);
    ESP_LOGI(LoraCommTag, "Added message ID %s with WAITING status", message_id);

    // Prepare the message with ID
    char message_with_id[MAX_MESSAGE_SIZE];
    snprintf(message_with_id, sizeof(message_with_id), "%s|%s", message_id, message);

    // Create LoRa message using your function
    lora_message = CreateLoraMessage(DevID, HubID, message_with_id, Length);
    if (lora_message == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to create LoRa message");
        cJSON_DeleteItemFromObject(json, message_id);
        save_ack_json(json);
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    // Queue the message for transmission
    if (queue_lora_message(lora_message) != ESP_OK) {
        ESP_LOGE(LoraCommTag, "Failed to queue message");
        cJSON_DeleteItemFromObject(json, message_id);
        save_ack_json(json);
        cJSON_Delete(json);
        free(lora_message);
        return ESP_FAIL;
    }

    // Free the LoRa message as it's now queued
    free(lora_message);
    ESP_LOGI(LoraCommTag, "Message queued for transmission: %s", message_with_id);
    RecivedLoraContiniousModeInit();

    // Wait a short time before first ACK check
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Wait and check for ACK
    for (int retry = 0; retry < ACK_RETRY_COUNT; retry++) {
        // Load a fresh copy of the JSON for each check
        cJSON_Delete(json);
        json = load_ack_json();
        if (json == NULL) {
            ESP_LOGE(LoraCommTag, "Failed to load ACK JSON file during ACK check");
            continue;
        }

        cJSON *status = cJSON_GetObjectItem(json, message_id);
        if (status && cJSON_IsString(status)) {
            ESP_LOGI(LoraCommTag, "Current status for message %s: %s", message_id, status->valuestring);
            if (strcmp(status->valuestring, "RECEIVED") == 0) {
                ESP_LOGI(LoraCommTag, "ACK received for message %s", message_id);
                result = ESP_OK;
                break;
            }
        } else {
            ESP_LOGW(LoraCommTag, "No status found for message %s", message_id);
        }

        if (retry < ACK_RETRY_COUNT - 1) {
            ESP_LOGW(LoraCommTag, "No ACK received for message %s, retrying... (attempt %d/%d)", 
                     message_id, retry + 1, ACK_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(ACK_CHECK_DELAY_MS));
        }
    }

    // Clean up
    cJSON_DeleteItemFromObject(json, message_id);
    save_ack_json(json);
    cJSON_Delete(json);

    if (result != ESP_OK) {
        ESP_LOGE(LoraCommTag, "Failed to receive ACK for message %s after %d attempts", message_id, ACK_RETRY_COUNT);
    }

    return result;
}
// esp_err_t result = send_message_with_ack(1, 100, "ack");
// if (result == ESP_OK) {
//     printf("Message sent and acknowledged successfully\n");
// } else {
//     printf("Failed to send message or receive acknowledgment\n");
// }
/////////////////////////END /////////////////////////////////

////////////////////////Normal Send Funtion///////////////

esp_err_t send_lora_message(int DevID, int HubID, const char* message,int Length) {
    char* lora_message = NULL;
    esp_err_t result = ESP_FAIL;

    // Create LoRa message using your function
    lora_message = CreateLoraMessage(DevID, HubID, message, Length);
    if (lora_message == NULL) {
        ESP_LOGE(LoraCommTag, "Failed to create LoRa message");
        return ESP_FAIL;
    }

    // Try to queue the message
    result = queue_lora_message(lora_message);
    
    if (result != ESP_OK) {
        // If queuing failed (probably because the queue is full), wait and try once more
        ESP_LOGW(LoraCommTag, "Failed to queue message, waiting and retrying...");
        vTaskDelay(pdMS_TO_TICKS(QUEUE_WAIT_TIME_MS));
        
        result = queue_lora_message(lora_message);
        
        if (result != ESP_OK) {
            ESP_LOGE(LoraCommTag, "Failed to queue message after retry");
            free(lora_message);
            return ESP_FAIL;
        }
    }

    // Message queued successfully
    ESP_LOGI(LoraCommTag, "Message queued successfully");
    free(lora_message);
    return ESP_OK;
}

// esp_err_t result = send_lora_message(1, 100, "Your message here");
// if (result == ESP_OK) {
//     printf("Message queued successfully\n");
// } else {
//     printf("Failed to queue message\n");
// }
/////////////////////////END/////////////////////////////