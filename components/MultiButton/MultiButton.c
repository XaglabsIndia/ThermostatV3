#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"  // For semaphores
#include "freertos/queue.h"   // If you need queues
#include "freertos/event_groups.h"
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
#include "MultiButtonHeader.h"
#include "ProvisionHeader.h"
#include "xag_nvs_component.h"
#include "esp_sleep.h"
#include "esp_timer.h"

TaskHandle_t MultiButtonTaskHandle = NULL;
TaskHandle_t autoProvisionTaskHandle = NULL;
SemaphoreHandle_t MultiButtonSemaphore;
extern TaskHandle_t Provision;
esp_timer_handle_t timeout_timer;
static bool isTaskRunning = false;
extern char NEMS_ID;
int StoredDeviceID;
//extern const char* THKeyMain;
extern const char* DEVKeyMain;
extern int ValueMain;
extern char* CreateLoraMessage(int device_id, int hub_id, const char* message, int total_length);
////////Lora RX Single Mode//////////////// 
extern uint8_t LoraSingleRXLength;
bool timer_active = false;
char* TAGMultiButton = "MultiButton";

void IRAM_ATTR MultiButtonISR(void *arg)
{
    xTaskResumeFromISR(MultiButtonTaskHandle);
}

void ConfigureGPIOSleep(){
// Configure GPIO16 for wakeup
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_MULTIBUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Enable EXT0 wakeup on GPIO16
    esp_err_t err = esp_sleep_enable_ext0_wakeup(CONFIG_MULTIBUTTON_GPIO, 0);  // 0 for LOW level trigger
    if (err != ESP_OK) {
        ESP_LOGE(TAGMultiButton, "Failed to configure EXT0 wakeup on GPIO16: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAGMultiButton, "EXT0 wakeup configured on GPIO16");
    }
}
void MultiButton(void *pvParameter)
{
    float pressLength_milliSeconds = 0;
    vTaskSuspend(NULL);
    while (1)
    {

        while (gpio_get_level(CONFIG_MULTIBUTTON_GPIO) == 0)
        {
            if (timer_active)
            {
                esp_timer_stop(timeout_timer);
                timer_active = false;
            }
            stop_leds();
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for button debounce
            pressLength_milliSeconds += 100;
            ESP_LOGI("MultiButton", "Button press duration: %.2f ms", pressLength_milliSeconds);
            
            if (pressLength_milliSeconds > 10 && pressLength_milliSeconds < 4999)
            {
                set_led_solid(true,true,true);

            }
            if (pressLength_milliSeconds > 5000 && pressLength_milliSeconds < 9999)
            {
                set_led_solid(false,true,true);

            }
            
            if (pressLength_milliSeconds > 10000 && pressLength_milliSeconds < 14999)
            {  
                set_led_solid(true,true,false);
            }
        }
        stop_leds();
        ESP_LOGI("MultiButton", "Total Value pressed is %.2f", pressLength_milliSeconds);
        int DurationPassed = (int)pressLength_milliSeconds;
        ESP_LOGI("MultiButton", "converted Value pressed is %d", DurationPassed);
        PerformAction(DurationPassed);
        pressLength_milliSeconds = 0;
        vTaskSuspend(NULL);
    }
}


void PerformAction(uint32_t duration) {
    if (duration >= 5000 && duration <= 9999) {  // 2-5 seconds
        if (!isTaskRunning && autoProvisionTaskHandle != NULL) {
            isTaskRunning = true;
            vTaskResume(autoProvisionTaskHandle);
            ESP_LOGI(TAGMultiButton, "Starting Auto Provision Task");
        }
    } else if (duration >= 10000 && duration <= 14999) {  // Keep existing restart functionality
        HandleOTAMessage(StoredDeviceID,StoredHubID);
    }
    else if (duration < 5000) {  // Keep existing restart functionality
            ConfigureGPIOSleep();
            stop_leds();
            set_led_solid(true,true,false);
            vTaskDelay(pdMS_TO_TICKS(2000));
            // Enter deep sleep
           // esp_err_t status = CheckStoredKeyStatus(THKeyMain, &ValueMain);
            esp_err_t err = CheckStoredKeyStatus("HUBID", &StoredHubID);

            if ( err == 4354) {
                stop_leds();
                ESP_LOGE("StoreID", "Failed to read from NVS");
                ConfigureGPIOSleep();
                stop_leds();
                set_led_solid(true,true,false);
                vTaskDelay(pdMS_TO_TICKS(2000));
                // Enter deep sleep
                esp_deep_sleep_start();
            }
            else{
                esp_restart();
            }
    }
    else if (duration > 14999) {  // Keep existing restart functionality
        esp_restart();
    }
}

static bool validate_lora_message(const char* message, int expected_dev_id, char* out_hub_id) {
    ESP_LOGI(TAGMultiButton, "Starting message validation: %s", message);
    if (!message || !out_hub_id) {
        ESP_LOGE(TAGMultiButton, "Invalid input parameters");
        return false;
    }

    // Make a copy of the message that we can modify
    char message_copy[255];
    memset(message_copy, 0, sizeof(message_copy));
    strncpy(message_copy, message, sizeof(message_copy) - 1);

    // Clean up the message by removing trailing characters after first ')'
    char* end_ptr = strchr(message_copy, ')');
    if (end_ptr) {
        *(end_ptr + 1) = '\0';  // Null terminate after the first ')'
    }

    ESP_LOGI(TAGMultiButton, "Cleaned message: %s", message_copy);

    // Parse NEMS ID
    char* saveptr;  // For thread-safe strtok_r
    char* token = strtok_r(message_copy, ",", &saveptr);
    if (!token) {
        ESP_LOGE(TAGMultiButton, "Failed to get NEMS ID token");
        return false;
    }

    if (strcmp(token, "NEMS") != 0) {
        ESP_LOGE(TAGMultiButton, "Invalid NEMS ID: %s", token);
        return false;
    }

    // Parse device ID
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) {
        ESP_LOGE(TAGMultiButton, "Failed to get device ID token");
        return false;
    }

    int received_dev_id = atoi(token);
    if (received_dev_id != expected_dev_id) {
        ESP_LOGE(TAGMultiButton, "Device ID mismatch: expected %d, got %d", expected_dev_id, received_dev_id);
        return false;
    }

    // Parse hub ID
    token = strtok_r(NULL, ",", &saveptr);
    if (!token) {
        ESP_LOGE(TAGMultiButton, "Failed to get hub ID token");
        return false;
    }

    int hub_id = atoi(token);
    ESP_LOGI(TAGMultiButton, "Parsed hub ID: %d", hub_id);

    // Parse TH value (now handling potential trailing characters)
    token = strtok_r(NULL, ")", &saveptr);
    if (!token) {
        ESP_LOGE(TAGMultiButton, "Failed to get TH value token");
        return false;
    }

    // Look for P| prefix
    if (strncmp(token, "T|", 2) != 0) {
        ESP_LOGE(TAGMultiButton, "Invalid TH format: missing P| prefix");
        return false;
    }

    // Extract numeric value after P|
    const char* value_str = token + 2;  // Skip "P|"
    int th_value = atoi(value_str);
    
    ESP_LOGI(TAGMultiButton, "Extracted TH value: %d", th_value);
    if (th_value <= 0) {
        ESP_LOGE(TAGMultiButton, "Invalid TH value: %d", th_value);
        return false;
    }

    // Save TH value first
    // esp_err_t err = save_int_to_nvs(THKeyMain, th_value);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAGMultiButton, "Failed to save TH value: %d, error: %s", th_value, esp_err_to_name(err));
    //     return false;
    // }
    ESP_LOGI(TAGMultiButton, "Successfully saved TH value: %d", th_value);

    // Then save hub ID
    esp_err_t err = save_int_to_nvs("HUBID", hub_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAGMultiButton, "Failed to save hub ID: %d, error: %s", hub_id, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAGMultiButton, "Successfully saved hub ID: %d", hub_id);

    *out_hub_id = hub_id;
    return true;
}
void AutoProvisionTask(void *pvParameters) {
    static char lora_rx_buffer[255];
    const int LISTEN_TIME_MS = 2000;  // 5 seconds listening time
    ESP_LOGI(TAGMultiButton, "Starting Auto Provision Task");
    vTaskSuspend(NULL);  // Start suspended
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        esp_task_wdt_reset();

        if (!isTaskRunning) {
            ESP_LOGI(TAGMultiButton, "Task not running, suspending");
            ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
            lora_end_rx();
            vTaskSuspend(NULL);
            ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
            continue;
        }
        bool valid_response_received = false;
        int attempt_count = 0;
        const int MAX_ATTEMPTS = 5;
        stop_leds();
        vTaskDelay(pdMS_TO_TICKS(100));
        set_led_blink(true,true,true,200);
        while (!valid_response_received && attempt_count < MAX_ATTEMPTS && isTaskRunning) {
            esp_task_wdt_reset();
            ESP_LOGI(TAGMultiButton, "Attempt %d/%d", attempt_count + 1, MAX_ATTEMPTS);
            // Send provision message
            char* lora_msg = CreateLoraMessage(StoredDeviceID, 0, "ThermostatProvision", 40);
            if (lora_msg != NULL) {
                esp_task_wdt_reset();
                ESP_LOGI(TAGMultiButton, "Sending message: %s", lora_msg);
                esp_err_t send_result = SendMessageWithCAD(lora_msg);
                if (send_result != ESP_OK) {
                    ESP_LOGW(TAGMultiButton, "Failed to send message, error: %d", send_result);
                    free(lora_msg);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    attempt_count++;
                    continue;
                }
                free(lora_msg);
                ESP_LOGI(TAGMultiButton, "Message sent successfully");
            }

            int listen_time = 0;
            memset(lora_rx_buffer, 0, sizeof(lora_rx_buffer));

            while (listen_time < LISTEN_TIME_MS && !valid_response_received && isTaskRunning) {
                esp_task_wdt_reset();
                esp_err_t ret = LoraSingleModeRX(1000, LoraSingleRXLength, lora_rx_buffer);
                
                if (ret == ESP_OK && lora_rx_buffer[0] != '\0') {
                    ESP_LOGI(TAGMultiButton, "Received message: %s", lora_rx_buffer);
                    
                    char message_copy[255];
                    strncpy(message_copy, lora_rx_buffer, sizeof(message_copy) - 1);
                    message_copy[sizeof(message_copy) - 1] = '\0';
                    
                    int hub_id = 0;
                    ESP_LOGI(TAGMultiButton, "Validating message: %s", message_copy);
                    
                    if (validate_lora_message(message_copy, StoredDeviceID, &hub_id)) {
                        ESP_LOGI(TAGMultiButton, "Message validated, hub_id: %d", hub_id);
                        if (hub_id != 0) {
                            ESP_LOGI(TAGMultiButton, "Valid hub_id received, saving to NVS");
                            if (save_int_to_nvs("HUBID", hub_id) == ESP_OK) {
                                // Remove watchdog before restart
                                ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
                                valid_response_received = true;
                                lora_end_rx();
                                ESP_LOGI(TAGMultiButton, "Hub ID and TH value saved successfully, restarting...");
                                stop_leds();
                                set_led_blink_alternate(true,true,true,false,true,false,200);
                                vTaskDelay(pdMS_TO_TICKS(3000));
                                esp_restart();
                            }
                        }
                    }
                }
                
                vTaskDelay(pdMS_TO_TICKS(1000));
                listen_time += 1000;
            }

            lora_end_rx();
            attempt_count++;
            esp_task_wdt_reset();
            
            if (!valid_response_received) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        if (!valid_response_received) {
            ESP_LOGW(TAGMultiButton, "Failed to receive valid response after %d attempts", MAX_ATTEMPTS);
            stop_leds();
            set_led_blink_alternate(true,true,true,true,false,false,200);
            vTaskDelay(pdMS_TO_TICKS(3000));
            // Clean up before deep sleep
            isTaskRunning = false;
            ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
            // Clean up LoRa
            lora_end_rx();
            ESP_LOGI(TAGMultiButton, "Entering deep sleep");
           // esp_err_t status = CheckStoredKeyStatus(THKeyMain, &ValueMain);
            esp_err_t err = CheckStoredKeyStatus("HUBID", &StoredHubID);

            if (err == 4354) {
                stop_leds();
                ESP_LOGE("StoreID", "Failed to read from NVS");
                ConfigureGPIOSleep();
                stop_leds();
                set_led_solid(true,true,false);
                vTaskDelay(pdMS_TO_TICKS(2000));
                // Enter deep sleep
                esp_deep_sleep_start();
            }
            else{
                esp_restart();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

////////////////////Initalize the timmer for sleep mode/////////////
static void timer_callback(void *arg)
{
    if (timer_active)
    {
        //esp_err_t status = CheckStoredKeyStatus(THKeyMain, &ValueMain);
        esp_err_t err = CheckStoredKeyStatus("HUBID", &StoredHubID);
        if (err == 4354) {
            stop_leds();
            ESP_LOGE("StoreID", "Failed to read from NVS");
            ConfigureGPIOSleep();
            set_led_solid(true,true,false);
            vTaskDelay(pdMS_TO_TICKS(2000));
            // Enter deep sleep
            esp_deep_sleep_start();
        }
        else{
            esp_restart();
        }
    }
}

// Initialize timer
void init_timeout_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "timeout_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timeout_timer));
}

// Start timer
void start_timeout_timer(void)
{
    timer_active = true;
    ESP_ERROR_CHECK(esp_timer_start_once(timeout_timer, 6000000)); // 6s in microseconds
}

/////////////////////////////////////////////////////////////////////////////////////////
void SetupMultiButton() {
    printf("CONFIG_MULTIBUTTON_GPIO %d", CONFIG_MULTIBUTTON_GPIO);
    AttachInterrupt(CONFIG_MULTIBUTTON_GPIO, MultiButtonISR, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0); 
    gpio_isr_handler_add(CONFIG_MULTIBUTTON_GPIO, MultiButtonISR, (void *)CONFIG_MULTIBUTTON_GPIO);
    MultiButtonSemaphore = xSemaphoreCreateBinary();
    esp_err_t err = lora_init();
      if (err != ESP_OK)
      {
        ESP_LOGI("lora_init", "lora_init %d", err);
            stop_leds();
            set_led_blink_alternate(false,true,true,true,false,false,200);
            // Configure GPIO16 for wakeup
                gpio_config_t io_conf = {
                    .pin_bit_mask = (1ULL << CONFIG_MULTIBUTTON_GPIO),
                    .mode = GPIO_MODE_INPUT,
                    .pull_up_en = GPIO_PULLUP_ENABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE
                };
                gpio_config(&io_conf);

                // Enable EXT0 wakeup on GPIO16
                esp_err_t err = esp_sleep_enable_ext0_wakeup(CONFIG_MULTIBUTTON_GPIO, 0);  // 0 for LOW level trigger
                if (err != ESP_OK) {
                    ESP_LOGE(TAGMultiButton, "Failed to configure EXT0 wakeup on GPIO16: %s", esp_err_to_name(err));
                } else {
                    ESP_LOGI(TAGMultiButton, "EXT0 wakeup configured on GPIO16");
                }
            // Enter deep sleep
            //esp_err_t status = CheckStoredKeyStatus(THKeyMain, &ValueMain);
             err = CheckStoredKeyStatus("HUBID", &StoredHubID);

            if (err == 4354) {
                stop_leds();
                ESP_LOGE("StoreID", "Failed to read from NVS");
                ConfigureGPIOSleep();
                stop_leds();
                set_led_solid(true,true,false);
                vTaskDelay(pdMS_TO_TICKS(2000));
                // Enter deep sleep
                esp_deep_sleep_start();
            }
            else{
                esp_restart();
            }
      }
    esp_err_t RetValue = init_nvs();
    if (RetValue != ESP_OK) 
    {
        printf("Error initializing NVS: %d\n", RetValue);
            ConfigureGPIOSleep();
            stop_leds();
            set_led_blink_alternate(false,true,true,true,false,false,200);
            vTaskDelay(pdMS_TO_TICKS(2000));
            // Enter deep sleep
           //esp_err_t status = CheckStoredKeyStatus(THKeyMain, &ValueMain);
            esp_err_t err = CheckStoredKeyStatus("HUBID", &StoredHubID);
            esp_err_t statusDEV = CheckStoredKeyStatus("DEVID", &StoredDeviceID);

            if (err == 4354) {
                stop_leds();
                ESP_LOGE("StoreID", "Failed to read from NVS");
                // Configure GPIO16 for wakeup
                ConfigureGPIOSleep();
                set_led_solid(true,true,false);
                vTaskDelay(pdMS_TO_TICKS(2000));
                // Enter deep sleep
                esp_deep_sleep_start();
            }
            else{
                esp_restart();
            }
    }
    esp_err_t DEVKeyMainStatus = CheckStoredKeyStatus("DEVID", &StoredDeviceID);
    xTaskCreate(MultiButton, "MultiButton", 5000, NULL, configMAX_PRIORITIES - 1, &MultiButtonTaskHandle);
    xTaskCreate(AutoProvisionTask, "auto_provision", 9096, NULL, 5, &autoProvisionTaskHandle);
    init_timeout_timer();
    start_timeout_timer();
    printf("Button duration detection started\n");

}