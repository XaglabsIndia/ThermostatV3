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
#include "nvs.h"               // ESP-IDF NVS functions
#include "nvs_flash.h"         // ESP-IDF NVS flash functions
#include "xag_ota_component.h" // Custom OTA header
#include "xag_mqtt_component.h"

// Logging tag for the OTA component
static const char *TAG = "xag_ota";
extern char mqttTopic[20];
// External variables for the server certificate
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

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
        case ESP_HTTPS_OTA_FINISH: // OTA finish event
            // stop_fade_ota();
            // ota_successful();
            // vTaskDelay(10000 / portTICK_PERIOD_MS);                            // Delay for 1 second
            ESP_LOGI(TAG, "OTA finish"); // Log the event
            break;
        case ESP_HTTPS_OTA_ABORT: // OTA abort event
            // stop_fade_ota();
            // ota_failed();
            // vTaskDelay(10000 / portTICK_PERIOD_MS);                            // Delay for 1 second
            ESP_LOGI(TAG, "OTA abort"); // Log the event
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
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update."); // Log the warning
        // turn_off_leds(RED_CHANNEL, GREEN_CHANNEL, BLUE_CHANNEL);
        // no_ota();
        // vTaskDelay(12000 / portTICK_PERIOD_MS);
        return ESP_FAIL; // Return fail if versions are the same
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
void advanced_ota_example_task()
{
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
        // vTaskDelete(NULL);                           // Delete the task
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc); // Get the image description
    if (err != ESP_OK)                                             // Check if getting image description failed
    {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed"); // Log the error
        goto ota_end;                                        // Jump to ota_end
    }

    // Print the new firmware version
    ESP_LOGI(TAG, "New firmware version: %s", app_desc.version); // Log the new firmware version
    err = validate_image_header(&app_desc); // Validate the image header
    if (err != ESP_OK)                      // Check if validation failed
    {
        ESP_LOGE(TAG, "image header verification failed"); // Log the error
        goto ota_end;                                      // Jump to ota_end
    }

    while (1) // Perform the OTA update
    {
        // start_fade_ota();
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
    }
    else
    {
        ota_finish_err = esp_https_ota_finish(https_ota_handle); // Finish the OTA process
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK))       // Check if OTA was successful
        {
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ..."); // Log the success
            // Initialize MQTT after successful Wi-Fi connection
            ESP_LOGI(TAG, "Initializing MQTT");
            esp_err_t MQTTSTatus = mqtt_component_init();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            mqtt_publish(mqttTopic,app_desc.version,1,1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);                            // Delay for 1 second
            esp_restart(); // Restart the ESP32
        }
        else
        {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) // Check if image validation failed
            {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted"); // Log the error
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err); // Log the OTA failure
            // // stop_fade_ota();
            // // ota_failed();
            // vTaskDelay(10000 / portTICK_PERIOD_MS);
            // vTaskDelete(NULL); // Delete the task
        }
    }

ota_end:
    esp_https_ota_abort(https_ota_handle);         // Abort the OTA process
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
    esp_restart(); // Log the OTA failure
    // vTaskDelete(NULL);                             // Delete the task
}

/**
 * @brief Checks for OTA update by creating the OTA task.
 *
 * This function initializes the OTA task which will perform the update.
 */
void check_ota_update()
{
    // xTaskCreate(&advanced_ota_example_task, "advanced_ota_example_task", 1024 * 8, NULL, 5, NULL); // Create the OTA task
    advanced_ota_example_task();
}

/**
 * @brief Initializes the OTA component.
 *
 * This function registers the event handler for OTA events and checks for updates.
 */
void xag_ota_init(void)
{
    ESP_LOGI(TAG, "Initializing OTA component"); // Log the initialization

    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL)); // Register the event handler
    check_ota_update();                                                                                       // Check for OTA update
}