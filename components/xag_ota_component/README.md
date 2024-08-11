Over-the-Air Firmware Update for ESP32

Project Overview
This project demonstrates how to perform an Over-the-Air (OTA) firmware update on an ESP32 device. It periodically checks a specified server for a new firmware version and updates the firmware if a newer version is available. The OTA process is managed using HTTPS for secure communication and validation of the firmware image.


Features
Secure OTA updates using HTTPS
Periodic firmware version checks
Validation of new firmware before applying the update
Automatic reboot upon successful update


Requirements
ESP32 development board
ESP-IDF (Espressif IoT Development Framework) version 4.0 or higher
HTTPS server hosting the new firmware image
Server certificate for HTTPS communication


File Structure
xag_ota/
├── xag_ota.c             # Main implementation file for OTA updates
├── xag_ota.h             # Header file for OTA update functions
└── README.md             # This README file

Configuration
Firmware Upgrade URL
The URL for the new firmware image should be specified in the project configuration. To set the firmware upgrade URL, follow these steps:

1. Open the project configuration menu : idf.py menuconfig

2. Navigate to OTA Configuration -> Firmware Upgrade URL and set the URL to the location of your new firmware image.



Server Certificate
Ensure that the server certificate is included in the project. The certificate should be placed inside te component directory of your ESP-IDF project.


Usage
Initialization
To use the OTA functionality, initialize the OTA component by calling xag_ota_init in your application code:

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize OTA
    xag_ota_init();
}


Event Handling
The event_handler function manages various OTA events such as connection status, image description retrieval, and flash writing progress. These events are logged for debugging and monitoring purposes.


OTA Update Task
The advanced_ota_example_task function handles the OTA update process, including downloading the new firmware image, validating it, and applying the update if the new version is valid. This task is created and managed by the check_ota_update function.

Function Documentation
void xag_ota_init(void)
Initializes the OTA component. This includes registering the event handler for OTA events and starting the OTA update check.

void check_ota_update()
Creates the OTA update task, which will perform the OTA update.

void advanced_ota_example_task(void *pvParameter)
Task function that manages the OTA update process. It connects to the server, downloads the new firmware, validates it, and applies the update if valid.

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
Validates the header of the new firmware image by comparing its version with the currently running version.

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
HTTP client initialization callback. Currently a placeholder for any additional initialization required for the HTTP client.

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
Handles various HTTPS OTA events and logs information based on the event type.

Author - Aman
Date - May 23, 2024
