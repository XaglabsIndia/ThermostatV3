/******************************************************************************************************
 * Filename: xag_ota.c
 * Author: Aman
 * Date: 2024-05-23
 * Description: This project demonstrates how to perform an Over-the-Air (OTA)
                firmware update on an ESP32 device.
                It periodically checks a specified server for a new firmware version
                and updates the firmware if a newer version is available.
 * Original Author: Aman
 * Source of the Code: Owned Module
*******************************************************************************************************/

#include <string.h>            // Standard library for string operations
#include "freertos/FreeRTOS.h" // FreeRTOS header
#include "freertos/task.h"     // FreeRTOS task header
#include "esp_system.h"        // ESP-IDF system functions
#include "esp_event.h"         // ESP-IDF event handling
#include "esp_log.h"           // ESP-IDF logging
#include "esp_ota_ops.h"       // ESP-IDF OTA operations
#include "esp_http_client.h"   // ESP-IDF HTTP client
#include "esp_https_ota.h"     // ESP-IDF HTTPS OTA functions
#include "xag_ota_component.h" // Custom OTA header
#include "cJSON.h"             // cJSON library for JSON parsing
#include "xag_mqtt_component.h"
#include "xag_nvs_component.h"
#define TOPIC_BUFFER_SIZE 64
#define PREFIX "XAGlab"
// Logging tag for the OTA component
extern const char* HUBKeyMain;
extern const char* DEVKeyMain;
static const char *TAG = "xag_ota";
// External variables for the server certificate
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");
extern void set_led_fade(bool red, bool green, bool blue);
extern void set_led_solid(bool red, bool green, bool blue);
extern void set_led_blink(bool red, bool green, bool blue, int blink_interval_ms);
extern void set_led_blink_alternate(bool red1, bool green1, bool blue1,
                                    bool red2, bool green2, bool blue2,
                                    int blink_interval_ms);
extern void stop_leds();
extern bool DeepSleepFlag;
char FirmwareVer[32];
extern bool DeepSleepFlag;
char pub_topic_ota[TOPIC_BUFFER_SIZE] = {0};
int stored_id = 0;
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    // Event handler code here
    return ESP_OK;
}


void update_mqtt_topics(void) {
    // Format the topic string
    snprintf(pub_topic_ota, TOPIC_BUFFER_SIZE, "%s_%d_P", PREFIX, stored_id);
}

esp_err_t initialize_mqtt_topics(void) {
    esp_err_t err;

    // Initialize NVS
    err = init_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        return err;
    }

    // Read stored ID using your existing NVS function
    err = read_int_from_nvs(HUBKeyMain, &stored_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read stored_id from NVS: %d", err);
        // You might want to set a default value if read fails
        stored_id = 0;
    }

    // Update the topic string
    update_mqtt_topics();

    ESP_LOGI(TAG, "MQTT topics initialized with stored_id: %d", stored_id);
    ESP_LOGI(TAG, "OTA topic: %s", pub_topic_ota);

    return ESP_OK;
}
void mqtt_ota_success(const esp_app_desc_t *new_app_info) {
    // Get device ID
    int device_id = 0;
    read_int_from_nvs(DEVKeyMain, &device_id);

    // Get current version
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t current_app_info;
    esp_ota_get_partition_description(running, &current_app_info);

    // // Create topic
    // char pub_topic[TOPIC_BUFFER_SIZE];
    // snprintf(pub_topic, TOPIC_BUFFER_SIZE, "%s_%d_P", PREFIX, device_id);

    // Create JSON message
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "success");
    cJSON_AddStringToObject(root, "message", "Update successful - rebooting");
    cJSON_AddNumberToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "current_version", current_app_info.version);

    if (new_app_info != NULL) {
        cJSON_AddStringToObject(root, "new_version", new_app_info->version);
        cJSON_AddStringToObject(root, "project_name", new_app_info->project_name);
        cJSON_AddStringToObject(root, "time", new_app_info->time);
        cJSON_AddStringToObject(root, "date", new_app_info->date);
    }

    char *json_string = cJSON_PrintUnformatted(root);
    
    // Success message with retain flag = 1
    mqtt_publish(pub_topic_ota, json_string, 1, 1);
    ESP_LOGI(TAG, "OTA update successful");

    cJSON_Delete(root);
    free(json_string);
}

void mqtt_ota_failure(const char* error_msg) {
    // Get device ID
    int device_id = 0;
    read_int_from_nvs(DEVKeyMain, &device_id);

    // Get current version
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t current_app_info;
    esp_ota_get_partition_description(running, &current_app_info);

    // Create topic
    char pub_topic[TOPIC_BUFFER_SIZE];
    snprintf(pub_topic, TOPIC_BUFFER_SIZE, "%s_%d_P", PREFIX, device_id);

    // Create JSON message
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "failed");
    cJSON_AddStringToObject(root, "message", error_msg);
    cJSON_AddNumberToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "current_version", current_app_info.version);

    char *json_string = cJSON_PrintUnformatted(root);
    
    // Failure message with retain flag = 0
    mqtt_publish(pub_topic_ota, json_string, 1, 0);
    ESP_LOGE(TAG, "OTA update failed: %s", error_msg);

    cJSON_Delete(root);
    free(json_string);
}

/**
 * @brief Event handler for HTTPS OTA events.
 *
 * This function handles various events during the OTA process,
 * logging information based on the event type.
 *
 * @param arg User argument, unused in this case.
 * @param event_base The event base, should be ESP_HTTPS_OTA_EVENT.
 * @param event_id The ID of the event.
 * @param event_data Additional data associated with the event.
 */

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == ESP_HTTPS_OTA_EVENT) // Check if the event base is ESP_HTTPS_OTA_EVENT
    {
        switch (event_id) // Switch based on the event ID
        {
        case ESP_HTTPS_OTA_START:         // OTA started event
            ESP_LOGI(TAG, "OTA started"); // Log the event
            break;
        case ESP_HTTPS_OTA_CONNECTED:             // Connected to server event
            ESP_LOGI(TAG, "Connected to server"); // Log the event
            break;
        case ESP_HTTPS_OTA_GET_IMG_DESC:                // Reading Image Description event
            ESP_LOGI(TAG, "Reading Image Description"); // Log the event
            break;
        case ESP_HTTPS_OTA_VERIFY_CHIP_ID:                                                     // Verifying chip ID event
            ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data); // Log the event with chip ID
            break;
        case ESP_HTTPS_OTA_DECRYPT_CB:                     // Decrypt callback event
            ESP_LOGI(TAG, "Callback to decrypt function"); // Log the event
            break;
        case ESP_HTTPS_OTA_WRITE_FLASH:                                        // Writing to flash event
            ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data); // Log the event with bytes written
            break;
        case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:                                                                // Boot partition updated event
            ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data); // Log the event with partition info
            break;
        case ESP_HTTPS_OTA_FINISH:       // OTA finish event
            break;
        case ESP_HTTPS_OTA_ABORT:       // OTA abort event
            ESP_LOGI(TAG, "OTA abort"); // Log the event
            DeepSleepFlag = false;
            mqtt_ota_failure("Update verification failed");
            esp_restart(); 
            break;
        }
    }
}

/**
 * @brief Validates the header of the new firmware image.
 *
 * This function compares the new firmware version with the currently running version
 * to determine if an update is necessary.
 *
 * @param new_app_info The application descriptor of the new firmware.
 * @return ESP_OK if the new firmware is valid and different from the running version,
 *         otherwise returns an error code.
 */
static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) // Check if new_app_info is NULL
    {
        return ESP_ERR_INVALID_ARG; // Return error if argument is invalid
    }

    const esp_partition_t *running = esp_ota_get_running_partition(); // Get the running partition
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) // Get the running app description
    {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version); // Log the running firmware version
    }

    if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) // Compare versions
    {
        stop_leds();
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update."); // Log the warning
        return ESP_FAIL;                                                                                 // Return fail if versions are the same
    }

    return ESP_OK; // Return OK if validation passes
}

/**
 * @brief HTTP client initialization callback.
 *
 * This function is a placeholder for any additional HTTP client initialization
 * that might be required.
 *
 * @param http_client The HTTP client handle.
 * @return ESP_OK if the initialization is successful.
 */
static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK; // Declare error variable
    return err;             // Return OK
}

/**
 * @brief Task to perform advanced OTA update.
 *
 * This task manages the OTA update process, including connecting to the server,
 * downloading the new firmware, and validating the new image.
 *
 * @param pvParameter Task parameter, unused in this case.
 */
void advanced_ota_example_task(void *pvParameter)
{
    set_led_blink(true, false, true,200);
    ESP_LOGI(TAG, "New firmware Path: %s", CONFIG_FIRMWARE_UPGRADE_URL);                         // Log the firmware path
    ESP_LOGI(TAG, "                  **************** Starting OTA Operation ****************"); // Log the start of OTA operation
    esp_err_t ota_finish_err = ESP_OK;                                                           // Declare OTA finish error variable

    esp_http_client_config_t config = {
        .url = CONFIG_FIRMWARE_UPGRADE_URL,        // Set the firmware URL
        .cert_pem = (char *)server_cert_pem_start, // Set the server certificate
        .timeout_ms = 5000,                        // Set the timeout
        .keep_alive_enable = true,                 // Enable keep-alive
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,                      // Set the HTTP config
        .http_client_init_cb = _http_client_init_cb, // Set the HTTP client init callback
    };

    esp_https_ota_handle_t https_ota_handle = NULL;                      // Declare OTA handle
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle); // Begin OTA process
    if (err != ESP_OK)                                                   // Check if OTA begin failed
    {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed"); // Log the error
        stop_leds();
        set_led_blink_alternate(true, true, false, true, false, false, 100);
        mqtt_ota_failure("ESP HTTPS OTA Begin failed");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        
        esp_restart();
        DeepSleepFlag = false;
        vTaskDelete(NULL); // Delete the task
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc); // Get the image description
    if (err != ESP_OK)                                             // Check if getting image description failed
    {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed"); // Log the error
        mqtt_ota_failure("esp_https_ota_read_img_desc failed");
        goto ota_end;                                        // Jump to ota_end
    }

    // Print the new firmware version
    ESP_LOGI(TAG, "New firmware version: %s", app_desc.version); // Log the new firmware version

    err = validate_image_header(&app_desc); // Validate the image header
    if (err != ESP_OK)                      // Check if validation failed
    {
        ESP_LOGE(TAG, "image header verification failed");
        mqtt_ota_failure("image header verification failed");
         // Log the error
        goto ota_end;                                      // Jump to ota_end
    }

    while (1) // Perform the OTA update
    {
        err = esp_https_ota_perform(https_ota_handle); // Perform OTA step
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)      // Check if OTA is not in progress
        {
            break; // Break the loop if OTA is complete
        }
        ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle)); // Log bytes read
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) // Check if complete data was received
    {
        ESP_LOGE(TAG, "Complete data was not received."); // Log the error
        goto ota_end;
    }
    else
    {
        ota_finish_err = esp_https_ota_finish(https_ota_handle); // Finish the OTA process
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK))       // Check if OTA was successful
        {
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ..."); // Log the success
            stop_leds();
            set_led_blink_alternate(true, true, false, false, true, false, 100);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            mqtt_ota_success(&app_desc);
            esp_restart();                         // Restart the ESP32
        }
        else
        {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) // Check if image validation failed
            {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted"); // Log the error
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err); // Log the OTA failure
            stop_leds();
            set_led_blink_alternate(true, true, false, true, false, false, 100);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            mqtt_ota_failure("Update verification failed");
            esp_restart(); 
            vTaskDelete(NULL); // Delete the task
        }
    }

ota_end:
    esp_https_ota_abort(https_ota_handle);         // Abort the OTA process
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed"); // Log the OTA failure
    stop_leds();
    DeepSleepFlag = false;
    set_led_blink_alternate(true, true, false, true, false, false, 100);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    mqtt_ota_failure("Update verification failed");
    esp_restart(); 
    vTaskDelete(NULL); // Delete the task
}

/**
 * @brief Checks for OTA update by creating the OTA task.
 *
 * This function initializes the OTA task which will perform the update.
 */

void compare_versions(const char *current_version, const char *new_version)
{
    int current_major, current_minor;
    int new_major, new_minor;

    // Parse the major and minor components of the current version
    sscanf(current_version, "%d.%d", &current_major, &current_minor);

    // Parse the major and minor components of the new version
    sscanf(new_version, "%d.%d", &new_major, &new_minor);

    // Compare the major and minor components
    if (new_major > current_major || (new_major == current_major && new_minor > current_minor))
    {
        ESP_LOGI(TAG, "New firmware version available. Starting OTA update...");
        ESP_LOGI(TAG, "Starting OTA example task");
        xTaskCreate(&advanced_ota_example_task, "advanced_ota_example_task", 1024 * 8, NULL, 5, NULL); // Create the OTA task
    }
    else
    {
        ESP_LOGI(TAG, "Firmware version is up to date. No need for an update.");
    }
}

void check_ota_update()
{
    xTaskCreate(&advanced_ota_example_task, "advanced_ota_example_task", 1024 * 8, NULL, 5, NULL); // Create the OTA task
}

/**
 * @brief Initializes the OTA component.
 *
 * This function registers the event handler for OTA events and checks for updates.
 */
void xag_ota_init(void)
{
    ESP_LOGI(TAG, "Initializing OTA component"); // Log the initialization
    stop_leds();
    ESP_ERROR_CHECK(initialize_mqtt_topics());
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); // Register the event handler
    check_ota_update();                                                                                       // Check for OTA update
}
