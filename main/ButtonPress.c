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

// #define CONFIGCONFIG_BUTTON_INC_GPIO 25
// #define CONFIG_BUTTON_DEC_GPIO 27
#define DEBOUNCE_TIME 100 // ms
#define MIN_TEMPERATURE 1.0f  // Minimum temperature limit
#define MAX_TEMPERATURE 50.0f  // Maximum temperature limit
#define NVS_KEY "set_temp_f"
#define BUTTON_TASK_TIMEOUT 5000 // 5 seconds in ms
#define TEMPERATURE_STEP 0.5f // Temperature change step
 const char* ButtonTag = "Button Press";
extern float set_temperature;
QueueHandle_t gpio_evt_queue = NULL;
void load_set_temperature(void);
typedef struct {
    uint32_t gpio_num;
    int64_t time;
} gpio_event_t;

int64_t last_button_press[2] = {0, 0}; // Array to store last press time for each button
void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    gpio_event_t event = {
        .gpio_num = gpio_num,
        .time = esp_timer_get_time()
    };
    xQueueSendFromISR(gpio_evt_queue, &event, NULL);
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
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_log.h"

void configure_wakeup() {
    // Configure EXT1 wake-up source for increment button
    esp_sleep_enable_ext1_wakeup((1ULL << CONFIG_BUTTON_INC_GPIO), ESP_EXT1_WAKEUP_ALL_LOW);
    
    // // Configure EXT1 wake-up source for decrement button
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    // esp_sleep_enable_ext1_wakeup((1ULL << CONFIG_BUTTON_DEC_GPIO), 1);

    // Configure RTC GPIOs
    rtc_gpio_init(CONFIG_BUTTON_INC_GPIO);
    rtc_gpio_init(CONFIG_BUTTON_DEC_GPIO);

    // Set as input with pull-up
    rtc_gpio_set_direction(CONFIG_BUTTON_INC_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(CONFIG_BUTTON_DEC_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(CONFIG_BUTTON_INC_GPIO);
    rtc_gpio_pullup_en(CONFIG_BUTTON_DEC_GPIO);

    ESP_LOGI(ButtonTag, "EXT1 wake-up configuration complete for both buttons");
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
    vTaskDelay(pdMS_TO_TICKS(100));
    configure_wakeup();
    ulp_component(0);
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
    ESP_LOGI(ButtonTag, "handle_button_presses started");
    bool set_temperature_changed = false;
    bool button_pressed[2] = {false, false};
    int64_t last_activity_time = esp_timer_get_time() / 1000;
    int64_t current_time;
    int loop_count = 0;

    while (1) {
        current_time = esp_timer_get_time() / 1000;
        loop_count++;

        if (loop_count % 100 == 0) {  // Log every 100 iterations
            ESP_LOGI(ButtonTag, "handle_button_presses loop iteration %d", loop_count);
        }

        gpio_event_t event;
        if (xQueueReceive(gpio_evt_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(ButtonTag, "Button event received: GPIO %ld", event.gpio_num);
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
                last_activity_time = current_time;  // Reset the timer on button press
                ESP_LOGI(ButtonTag, "Button pressed. Resetting 5-second timer.");
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

        // Check if it's been timeout_ms milliseconds since the last activity
        if ((current_time - last_activity_time) >= timeout_ms) {
            ESP_LOGI(ButtonTag, "No activity for %d ms. Exiting handle_button_presses", timeout_ms);

            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(ButtonTag, "handle_button_presses ended. Temperature changed: %d", set_temperature_changed);
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
            ulp_component(0);
        }
        ESP_LOGI(ButtonTag, "Button handling task loop completed");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void InitTempButton(void) {
    esp_log_level_set(ButtonTag, ESP_LOG_INFO);
    ESP_LOGI(ButtonTag, "Thermostat starting up");
    
    // Load temperature from NVS
    load_set_temperature();

    // Configure GPIOs
    configure_gpio();

    // Create a queue to handle GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_event_t));

    // Install gpio isr service
    // gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_BUTTON_INC_GPIO, gpio_isr_handler, (void*) CONFIG_BUTTON_INC_GPIO);
    gpio_isr_handler_add(CONFIG_BUTTON_DEC_GPIO, gpio_isr_handler, (void*) CONFIG_BUTTON_DEC_GPIO);

    // Create the button handling task
xTaskCreatePinnedToCore(button_handling_task, "button_handling_task", 8192, NULL, 5, NULL,0);}