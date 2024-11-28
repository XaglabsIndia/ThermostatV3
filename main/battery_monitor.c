
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <string.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

float voltage = 0.0;
// battery related initialisations
float voltage_battery = 0.0;
 float R1 = 240000.0; // 205K ohm resistor
 float R2 = 68000.0;  // 56K ohm resistor
const float correctionFactor = 0;
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_8  // Change this to your actual ADC channel
#define BATTERY_ADC_UNIT ADC_UNIT_2
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_0
#define BATTERY_MIN_VOLTAGE 2.7f
#define BATTERY_MAX_VOLTAGE 4.2f
#define NUM_READINGS 100 
#define delay(x)  vTaskDelay(x/portTICK_PERIOD_MS)

static const char* TAG = "BATTERY_ADC";
static adc_oneshot_unit_handle_t adc2_handle;
static int temperature,humidity,battery_percentage;

TaskHandle_t battery_handle = NULL;


adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = BATTERY_ADC_UNIT,
    };


esp_err_t init_adc(void) {
    ESP_LOGI(TAG, "Initializing ADC2");

    esp_err_t err = adc_oneshot_new_unit(&init_config2, &adc2_handle);  // Using adc2_handle
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC2 unit initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(adc2_handle, BATTERY_ADC_CHANNEL, &config);  // Using adc2_handle
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC2 channel configuration failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ADC2 initialized successfully");
    return ESP_OK;
}

float read_battery_voltage(void)
{
    esp_err_t err = init_adc();
    int adc_raw;

    float avgVoltage = 0.0;

    for (int i = 0; i < NUM_READINGS; i++)
    {
        esp_err_t err = adc_oneshot_read(adc2_handle, BATTERY_ADC_CHANNEL, &adc_raw); // Using adc2_handle
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "ADC2 read failed: %s", esp_err_to_name(err));
            return -1.0f;
        }
        voltage_battery = adc_raw;
        avgVoltage += voltage_battery;
        delay(10);
    }
    avgVoltage /= NUM_READINGS;
    //    avgVoltage = (float)adc_raw / 4095.0f * 4.2f;
       avgVoltage += correctionFactor;
       ESP_LOGI(TAG, "ADC VALUE  : %.2fV", avgVoltage);
    return avgVoltage;
}

int calculate_battery_percentage(float voltage_battery)
{
    float percentage = ((voltage_battery - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)*100.0f);
    if (percentage > 100.0f)
        percentage = 100.0f;
    if (percentage < 0.0f)
        percentage = 0.0f;
    return (int)percentage;
}

void battery_task(void *pvParameters) {
    ESP_LOGI(TAG, "Battery task started");
    while (1) {
        // Ensure Wi-Fi is not using the ADC2
      //  esp_wifi_stop();
        
        float battery_voltage = read_battery_voltage();
        if (battery_voltage >= 0) {
            float gpio_voltage=(((battery_voltage * 100)/4095)/100);
            float battery_voltage=((gpio_voltage*((R1+R2)/R2)) + 0.212f);
            battery_percentage = calculate_battery_percentage(battery_voltage);
            ESP_LOGI(TAG, "gpio_pin voltage_battery: %.3fV,\nBattery voltage_battery: %.2fV,\n battery_percentage %d%%",gpio_voltage,((gpio_voltage*((R1+R2)/R2)) + 0.212f),battery_percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read battery voltage_battery");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
