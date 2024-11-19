#include <stdio.h>
#include "xag_battery_component.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
// battery related initialisations
float voltage = 0.0;

const float correctionFactor = 0;
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_5  // Change this to your actual ADC channel
#define BATTERY_ADC_UNIT ADC_UNIT_2
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_0
#define BATTERY_MIN_VOLTAGE 2.7f
#define BATTERY_MAX_VOLTAGE 4.5f
#define NUM_READINGS 100 
#define delay(x)  vTaskDelay(x/portTICK_PERIOD_MS)
static const char* TAG = "BATTERY_ADC";

static adc_oneshot_unit_handle_t adc2_handle;

    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = BATTERY_ADC_UNIT,
    };
 
char* int_to_string_for_lora(int value, int max_length) {
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
        voltage = adc_raw;
        avgVoltage += voltage;
        delay(10);
    }
    avgVoltage /= NUM_READINGS;
    //    avgVoltage = (float)adc_raw / 4095.0f * 4.2f;
       avgVoltage += correctionFactor;
       ESP_LOGI(TAG, "ADC VALUE  : %.2fV", avgVoltage);
    return avgVoltage;
}

int calculate_battery_percentage(float voltage)
{
    float percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)*100.0f);
    if (percentage > 100.0f)
        percentage = 100.0f;
    if (percentage < 0.0f)
        percentage = 0.0f;
    return (int)percentage;
}
