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
#include <string.h>
#include <math.h>

// #define CONFIGCONFIG_BUTTON_INC_GPIO 25
// #define CONFIG_BUTTON_DEC_GPIO 27
#define DEBOUNCE_TIME 50 // ms
#define MIN_TEMPERATURE 10.0f  // Minimum temperature limit
#define MAX_TEMPERATURE 30.0f  // Maximum temperature limit
#define BUTTON_TASK_TIMEOUT 5000 // 5 seconds in ms
#define TEMPERATURE_STEP 0.5f // Temperature change step
 const char* ButtonTag = "Button Press";
extern float set_temperature;
extern void setTemperature_partial_update(void);
extern void inactive_screen_call(void);
QueueHandle_t gpio_evt_queue = NULL;
void load_set_temperature(void);
typedef struct {
    uint32_t gpio_num;
    int64_t time;
} gpio_event_t;

int64_t last_button_press[2] = {0, 0}; // Array to store last press time for each button
void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    gpio_event_t event = {
        .gpio_num = gpio_num,
        .time = esp_timer_get_time()
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


 bool debounce_button(int button_index) {
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    if (current_time - last_button_press[button_index] > DEBOUNCE_TIME) {
        // Add button level check to prevent false triggers
        uint32_t gpio_num = (button_index == 0) ? CONFIG_BUTTON_INC_GPIO : CONFIG_BUTTON_DEC_GPIO;
        if (gpio_get_level(gpio_num) == 0) {  // Active low button
            last_button_press[button_index] = current_time;
            return true;
        }
    }
    return false;
}


void print_gpio_status() {
    ESP_LOGI(ButtonTag, "GPIO %d status: %d", CONFIG_BUTTON_INC_GPIO, gpio_get_level(CONFIG_BUTTON_INC_GPIO));
    ESP_LOGI(ButtonTag, "GPIO %d status: %d", CONFIG_BUTTON_DEC_GPIO, gpio_get_level(CONFIG_BUTTON_DEC_GPIO));
}
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_log.h"
#define WAKEUP_GPIO_PIN GPIO_NUM_16  // Using GPIO16 as wake-up pin

void enter_sleep_mode(void);
void configure_wakeup() {
    // Configure EXT1 wake-up source for increment button
    esp_sleep_enable_ext1_wakeup((1ULL << CONFIG_BUTTON_INC_GPIO), ESP_EXT1_WAKEUP_ALL_LOW);
    
    // // Configure EXT1 wake-up source for decrement button
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    // esp_sleep_enable_ext1_wakeup((1ULL << CONFIG_BUTTON_DEC_GPIO), ESP_EXT1_WAKEUP_ALL_LOW);

    // Configure RTC GPIOs
    rtc_gpio_init(CONFIG_BUTTON_INC_GPIO);
    rtc_gpio_init(CONFIG_BUTTON_DEC_GPIO);

    // Set as input with pull-up
    rtc_gpio_set_direction(CONFIG_BUTTON_INC_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(CONFIG_BUTTON_DEC_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(CONFIG_BUTTON_INC_GPIO);
    rtc_gpio_pullup_en(CONFIG_BUTTON_DEC_GPIO);

    ESP_LOGI(ButtonTag, "EXT1 wake-up configuration complete for both buttons");
    InitalizeProgram();
    vTaskDelay(100);
    Temperaturedata();
   enter_sleep_mode();
}
void configure_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = ((1ULL<<CONFIG_BUTTON_INC_GPIO) | (1ULL<<CONFIG_BUTTON_DEC_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Check if ISR service is already installed
    esp_err_t isr_status = gpio_install_isr_service(0);
    if (isr_status != ESP_OK && isr_status != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(ButtonTag, "Failed to install ISR service: %d", isr_status);
        return;
    }

    // Remove existing handlers if any
    gpio_isr_handler_remove(CONFIG_BUTTON_INC_GPIO);
    gpio_isr_handler_remove(CONFIG_BUTTON_DEC_GPIO);

    // Add new handlers
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_BUTTON_INC_GPIO, gpio_isr_handler, (void*) CONFIG_BUTTON_INC_GPIO));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_BUTTON_DEC_GPIO, gpio_isr_handler, (void*) CONFIG_BUTTON_DEC_GPIO));
    
    ESP_LOGI(ButtonTag, "GPIO configuration complete");
    print_gpio_status();
}

void enter_sleep_mode(void) {
    ESP_LOGI(ButtonTag, "Preparing to enter deep sleep mode");
   
    print_gpio_status();
   // LoraDeepSleepInit();
// Disable all wake-up sources first
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    // Configure GPIO wake-up pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    // Enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,    // Wake up on low level (button press)
    };
    
    // Configure GPIO
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure wake-up source
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO_PIN, 0));
    // Configure EXT1 wake-up source for both buttons
    const uint64_t ext1_wakeup_pin_mask = (1ULL << CONFIG_BUTTON_INC_GPIO) | (1ULL << CONFIG_BUTTON_DEC_GPIO);
    esp_sleep_enable_ext1_wakeup(ext1_wakeup_pin_mask, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_sleep_enable_timer_wakeup(5 * 60 * 1000000ULL);
    inactive_screen_call();
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_deep_sleep_start();
}

void enter_sleep_mode_Timmer(void) {
    ESP_LOGI(ButtonTag, "Preparing to enter deep sleep mode");
   
    print_gpio_status();
    //LoraDeepSleepInit();
// Disable all wake-up sources first
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    // Configure GPIO wake-up pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WAKEUP_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    // Enable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,    // Wake up on low level (button press)
    };
    
    // Configure GPIO
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure wake-up source
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO_PIN, 0));
    // Configure EXT1 wake-up source for both buttons
    const uint64_t ext1_wakeup_pin_mask = (1ULL << CONFIG_BUTTON_INC_GPIO) | (1ULL << CONFIG_BUTTON_DEC_GPIO);
    esp_sleep_enable_ext1_wakeup(ext1_wakeup_pin_mask, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_sleep_enable_timer_wakeup(5 * 60 * 1000000ULL);
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_deep_sleep_start();
}

void save_temperature(void) {
    set_temperature = round(set_temperature * 10.0) / 10.0;
    
    // Add retry mechanism
    const int max_retries = 3;
    esp_err_t RetDevValue;
    
    for (int i = 0; i < max_retries; i++) {
        RetDevValue = save_float_to_nvs(NVS_KEY, set_temperature);
        if (RetDevValue == ESP_OK) {
            ESP_LOGI(ButtonTag, "Temperature written successfully: %.1f", set_temperature);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Short delay between retries
    }
    
    ESP_LOGE(ButtonTag, "Failed to save temperature after %d attempts", max_retries);
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
    int64_t last_activity_time = esp_timer_get_time() / 1000;
    int64_t current_time;
    
    while (1) {
        current_time = esp_timer_get_time() / 1000;
        
        gpio_event_t event;
        if (xQueueReceive(gpio_evt_queue, &event, pdMS_TO_TICKS(50)) == pdTRUE) {
            int button_index = (event.gpio_num == CONFIG_BUTTON_INC_GPIO) ? 0 : 1;
            
            if (!button_pressed[button_index] && debounce_button(button_index)) {
                button_pressed[button_index] = true;
                
                // Handle temperature change with bounds checking
                if (event.gpio_num == CONFIG_BUTTON_INC_GPIO) {
                    if (set_temperature < MAX_TEMPERATURE) {
                        set_temperature = fmin(set_temperature + TEMPERATURE_STEP, MAX_TEMPERATURE);
                        set_temperature_changed = true;
                        ESP_LOGI(ButtonTag, "Temperature increased to: %.1f", set_temperature);
                        setTemperature_partial_update();
                    }
                } else if (event.gpio_num == CONFIG_BUTTON_DEC_GPIO) {
                    if (set_temperature > MIN_TEMPERATURE) {
                        set_temperature = fmax(set_temperature - TEMPERATURE_STEP, MIN_TEMPERATURE);
                        set_temperature_changed = true;
                        ESP_LOGI(ButtonTag, "Temperature decreased to: %.1f", set_temperature);
                        setTemperature_partial_update();
                    }
                }
                
                last_activity_time = current_time;
            }
        } else {
            // Reset button pressed state when button is released
            if (gpio_get_level(CONFIG_BUTTON_INC_GPIO) == 1) {
                button_pressed[0] = false;
            }
            if (gpio_get_level(CONFIG_BUTTON_DEC_GPIO) == 1) {
                button_pressed[1] = false;
            }
        }
        
        if ((current_time - last_activity_time) >= timeout_ms) {
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return set_temperature_changed;
}

void button_handling_task(void *pvParameters) {
    ESP_LOGI(ButtonTag, "Button handling task started");
    while (1) {
        ESP_LOGI(ButtonTag, "Button handling task loop starting");
        bool set_temperature_changed = handle_button_presses(BUTTON_TASK_TIMEOUT);
        ESP_LOGI(ButtonTag, "handle_button_presses returned: %d", set_temperature_changed);
        if (set_temperature_changed) {
            ESP_LOGI(ButtonTag, "Temperature changed, saving...");
            save_temperature();
        } else {
            ESP_LOGI(ButtonTag, "No temperature change in last 5 seconds");
            configure_wakeup();
           // ulp_component(0);
        }
        ESP_LOGI(ButtonTag, "Button handling task loop completed");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void InitTempButton(void) {
    esp_log_level_set(ButtonTag, ESP_LOG_INFO);
    ESP_LOGI(ButtonTag, "Thermostat starting up");
    
    // Initialize LoRa
    esp_err_t err = lora_init();
    if (err != ESP_OK) {
        ESP_LOGI("lora_init", "lora_init %d", err);
        return;
    }
    
    // Load temperature from NVS
    load_set_temperature();
    
    // Create queue first
    if (gpio_evt_queue == NULL) {  // Only create if not already created
        gpio_evt_queue = xQueueCreate(20, sizeof(gpio_event_t));
        if (gpio_evt_queue == NULL) {
            ESP_LOGE(ButtonTag, "Failed to create gpio_evt_queue");
            return;
        }
    }
    
    // Configure GPIOs
    configure_gpio();
    
    // Create the button handling task
    xTaskCreatePinnedToCore(button_handling_task, "button_handling_task", 8192, NULL, 5, NULL, 0);
}