#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ProvisionHeader.h"
#include "xag_nvs_component.h"
#include "MultiButtonHeader.h"
#include "Config.h"

// #define CONFIGCONFIG_BUTTON_INC_GPIO 25
// #define CONFIG_BUTTON_DEC_GPIO 27
#define DEBOUNCE_TIME 100 // ms
#define MIN_TEMPERATURE 1.0f  // Minimum temperature limit
#define MAX_TEMPERATURE 30.0f  // Maximum temperature limit
#define NVS_KEY "set_temp_f"
#define BUTTON_TASK_TIMEOUT 5000 // 5 seconds in ms
#define TEMPERATURE_STEP 0.5f // Temperature change step
 const char* ButtonTag = "Button Press";
extern float set_temperature;
QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
    uint32_t gpio_num;
    int64_t time;
} gpio_event_t;

int64_t last_button_press[2] = {0, 0}; // Array to store last press time for each button

void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(ButtonTag, "Wake up caused by button press on GPIO %d", CONFIG_BUTTON_INC_GPIO);
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            {
                uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
                if (wakeup_pin_mask & (1ULL << CONFIG_BUTTON_DEC_GPIO)) {
                    ESP_LOGI(ButtonTag, "Wake up caused by button press on GPIO %d", CONFIG_BUTTON_DEC_GPIO);
                } else {
                    ESP_LOGI(ButtonTag, "Wake up caused by unexpected EXT1 GPIO");
                }
            }
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            ESP_LOGI(ButtonTag, "Wake up caused by ULP program");
            break;
        default:
            ESP_LOGI(ButtonTag, "Wake up was not caused by deep sleep: %d", wakeup_reason);
            break;
    }
}

 bool debounce_button(int button_index) {
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    if (current_time - last_button_press[button_index] > DEBOUNCE_TIME) {
        last_button_press[button_index] = current_time;
        return true;
    }
    return false;
}


void print_gpio_status() {
    ESP_LOGI(ButtonTag, "GPIO %d status: %d", CONFIG_BUTTON_INC_GPIO, gpio_get_level(CONFIG_BUTTON_INC_GPIO));
    ESP_LOGI(ButtonTag, "GPIO %d status: %d", CONFIG_BUTTON_DEC_GPIO, gpio_get_level(CONFIG_BUTTON_DEC_GPIO));
}

void configure_gpio() {
    gpio_config_t io_conf;
    // enable or disable interrupt
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // bit mask of the pins that you want to set
    io_conf.pin_bit_mask = ((1ULL<<CONFIG_BUTTON_INC_GPIO) | (1ULL<<CONFIG_BUTTON_DEC_GPIO));
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    ESP_LOGI(ButtonTag, "GPIO configuration complete");
    print_gpio_status();
}

void enter_sleep_mode(void) {
    ESP_LOGI(ButtonTag, "Preparing to enter deep sleep mode");
   
    print_gpio_status();

// Disable all wake-up sources first
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Configure EXT1 wake-up source for both buttons
    const uint64_t ext1_wakeup_pin_mask = (1ULL << CONFIG_BUTTON_INC_GPIO) | (1ULL << CONFIG_BUTTON_DEC_GPIO);
    esp_sleep_enable_ext1_wakeup(ext1_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Enable ULP wake-up
    esp_sleep_enable_ulp_wakeup();

    // Delay to allow for serial output
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(ButtonTag, "Entering deep sleep mode");
    esp_deep_sleep_start();
}
 void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    gpio_event_t event = {
        .gpio_num = gpio_num,
        .time = esp_timer_get_time()
    };
    xQueueSendFromISR(gpio_evt_queue, &event, NULL);
}

void save_temperature(void)
{
    esp_err_t RetDevValue = save_float_to_nvs(NVS_KEY, set_temperature);
    if (RetDevValue != ESP_OK)
    {
        RetDevValue = save_float_to_nvs(NVS_KEY, set_temperature);
    }
    ESP_LOGI(ButtonTag, "Temperature written: %.1f", set_temperature);
    Temperaturedata();
    
    // Enter sleep mode immediately after saving
    enter_sleep_mode();
}

void load_set_temperature(void)
{
    esp_err_t CheckSetTemp = read_float_from_nvs(NVS_KEY, &set_temperature);
    printf("CheckSetTemp %d",CheckSetTemp);
    if(CheckSetTemp == 4354)
    {
        esp_err_t RetDevValue = save_float_to_nvs(NVS_KEY, set_temperature);
        if (RetDevValue == ESP_OK)
        {
            esp_restart();
        }  
    }
    esp_err_t err = read_float_from_nvs(NVS_KEY, &set_temperature);
    if (err != ESP_OK) {
        ESP_LOGE("StoreID", "Failed to read set temperature from NVS");
    }

    // Ensure loaded temperature is within limits
    if (set_temperature < MIN_TEMPERATURE) set_temperature = MIN_TEMPERATURE;
    if (set_temperature > MAX_TEMPERATURE) set_temperature = MAX_TEMPERATURE;
}

bool handle_button_presses(int timeout_ms) {
    bool set_temperature_changed = false;
    bool button_pressed[2] = {false, false};
    int64_t last_press_time = 0;
    int64_t current_time;

    while (1) {
        current_time = esp_timer_get_time() / 1000; // Current time in milliseconds

        gpio_event_t event;
        if (xQueueReceive(gpio_evt_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            int button_index = (event.gpio_num == CONFIG_BUTTON_INC_GPIO) ? 0 : 1;
            
            if (!button_pressed[button_index] && debounce_button(button_index)) {
                button_pressed[button_index] = true;
                
                if (event.gpio_num == CONFIG_BUTTON_INC_GPIO && set_temperature < MAX_TEMPERATURE) {
                    set_temperature += TEMPERATURE_STEP;
                    if (set_temperature > MAX_TEMPERATURE) set_temperature = MAX_TEMPERATURE;
                    set_temperature_changed = true;
                    ESP_LOGI(ButtonTag, "Temperature increased to: %.1f", set_temperature);
                } else if (event.gpio_num == CONFIG_BUTTON_DEC_GPIO && set_temperature > MIN_TEMPERATURE) {
                    set_temperature -= TEMPERATURE_STEP;
                    if (set_temperature < MIN_TEMPERATURE) set_temperature = MIN_TEMPERATURE;
                    set_temperature_changed = true;
                    ESP_LOGI(ButtonTag, "Temperature decreased to: %.1f", set_temperature);
                }
                last_press_time = current_time;
                ESP_LOGI(ButtonTag, "Button pressed. Waiting 5 seconds for next press.");
            }
        } else {
            // No event received, check if buttons are released
            if (gpio_get_level(CONFIG_BUTTON_INC_GPIO) == 1) {
                button_pressed[0] = false;
            }
            if (gpio_get_level(CONFIG_BUTTON_DEC_GPIO) == 1) {
                button_pressed[1] = false;
            }
        }

        // Check if it's been 5 seconds since the last button press
        if (last_press_time > 0 && (current_time - last_press_time) >= timeout_ms) {
            ESP_LOGI(ButtonTag, "5 seconds passed since last button press. Exiting.");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to prevent tight looping
    }

    return set_temperature_changed;
}

void button_handling_task(void *pvParameters) {
    while (1) {
        bool set_temperature_changed = handle_button_presses(BUTTON_TASK_TIMEOUT);
        if (set_temperature_changed) {
            save_temperature();
        } else {
            // No button press for 5 seconds, enter sleep mode
            printf("Okay No issues ");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent tight looping
    }
}

void InitTempButton(void) {
    esp_log_level_set(ButtonTag, ESP_LOG_INFO);
    ESP_LOGI(ButtonTag, "Thermostat starting up");
    
    print_wakeup_reason();

    // Load temperature from NVS
    load_set_temperature();

    // Configure GPIOs
    configure_gpio();

    // Create a queue to handle GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_event_t));

    // Install gpio isr service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_BUTTON_INC_GPIO, gpio_isr_handler, (void*) CONFIG_BUTTON_INC_GPIO);
    gpio_isr_handler_add(CONFIG_BUTTON_DEC_GPIO, gpio_isr_handler, (void*) CONFIG_BUTTON_DEC_GPIO);

    // Create the button handling task
    xTaskCreate(button_handling_task, "button_handling_task", 4096, NULL, 5, NULL);
}