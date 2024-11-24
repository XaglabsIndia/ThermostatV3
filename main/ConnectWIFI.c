
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "xag_wifi_component.h"
#include "xag_nvs_component.h"
#include"xag_ota_component.h"
#include"xag_mqtt_component.h"
#include "cJSON.h"
#include "config.h"
#include "nvs_flash.h"
extern char mqttTopic[20];
char* TAG1 = "Connect WIFI";
#define MAX_RETRIES_WIFI   5
void ConnectWifi(){
    stop_leds();
    set_led_blink(false,false,true,200);
    // Attempt to connect to WiFi
    int RetryCount = 0;
    while (RetryCount < MAX_RETRIES_WIFI) {
        ESP_LOGI(TAG1, "Attempting to connect to WiFi (Attempt %d of %d)...", RetryCount + 1, MAX_RETRIES_WIFI);
        esp_err_t wifi_result = connectWifi();

        if (wifi_result == ESP_OK) {
            ESP_LOGI(TAG1, "Successfully connected to WiFi");
            stop_leds();
            set_led_blink_alternate(false,true,false,false,false,true,200);
            vTaskDelay(pdMS_TO_TICKS(3000));
            // Initialize MQTT after successful Wi-Fi connection
            ESP_LOGI(TAG1, "Initializing MQTT");
            esp_err_t MQTTSTatus = mqtt_component_init();
            if (MQTTSTatus!=ESP_OK){
            stop_leds();
            set_led_blink_alternate(false,true,false,true,false,false,200);
            vTaskDelay(pdMS_TO_TICKS(3000));
            enter_sleep_mode();
            }
            xag_ota_init();
             break;
        } else {
            ESP_LOGE(TAG1, "Failed to connect to WiFi");
            RetryCount++;
            if (RetryCount < MAX_RETRIES_WIFI) {
            stop_leds();
            set_led_blink_alternate(false,false,true,true,false,false,200);
            vTaskDelay(pdMS_TO_TICKS(3000));
            enter_sleep_mode();

            }
        }
    }
    if (RetryCount == MAX_RETRIES_WIFI) {
        ESP_LOGE(TAG1, "Failed to connect to WiFi after maximum retries");
        // Blink red and blue LEDs alternately
        enter_sleep_mode();
        }

}