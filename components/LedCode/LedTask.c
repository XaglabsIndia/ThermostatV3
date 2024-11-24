#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LED_RED_GPIO CONFIG_RED_LED_GPIO
#define LED_GREEN_GPIO CONFIG_GREEN_LED_GPIO
#define LED_BLUE_GPIO CONFIG_BLUE_LED_GPIO

#define RED_CHANNEL LEDC_CHANNEL_3
#define GREEN_CHANNEL LEDC_CHANNEL_1
#define BLUE_CHANNEL LEDC_CHANNEL_2

#define TAG "LED_CONTROL"

typedef enum
{
    LED_CMD_SOLID,
    LED_CMD_BLINK,
    LED_CMD_BLINK_ALTERNATE,
    LED_CMD_FADE,
    LED_CMD_STOP
} led_command_type_t;

typedef struct
{
    led_command_type_t type;
    bool red;
    bool green;
    bool blue;
    bool red2;
    bool green2;
    bool blue2;
    int blink_interval_ms;
} led_command_t;

static QueueHandle_t led_queue = NULL;
static TaskHandle_t led_task_handle = NULL;

void led_control_task(void *pvParameters)
{
    led_command_t current_cmd = {LED_CMD_STOP, false, false, false, false, false, false, 0};
    bool is_first_state = true;
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));

    while (1)
    {
        esp_task_wdt_reset();

        if (xQueueReceive(led_queue, &current_cmd, 0) == pdTRUE)
        {
            is_first_state = true;                // Reset to start with first state on new command
            last_wake_time = xTaskGetTickCount(); // Reset the last wake time
        }

        switch (current_cmd.type)
        {
        case LED_CMD_SOLID:
            if (current_cmd.red){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 255, 0);}
            if (current_cmd.green){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 255, 0);}
            if (current_cmd.blue){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 255, 0);}

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
            break;

        case LED_CMD_BLINK:
            if (current_cmd.red){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 255, 0);}
            if (current_cmd.green){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 255, 0);}
            if (current_cmd.blue){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 255, 0);}

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(current_cmd.blink_interval_ms));

            if (current_cmd.red){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 0, 0);}
            if (current_cmd.green){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 0, 0);}
            if (current_cmd.blue){
                ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 0, 0);}

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(current_cmd.blink_interval_ms));
            break;

        case LED_CMD_BLINK_ALTERNATE:
            if (is_first_state)
            {
                if (current_cmd.red){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 255, 0);}
                if (current_cmd.green){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 255, 0);}
                if (current_cmd.blue){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 255, 0);}
            }
            else
            {
                if (current_cmd.red2){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 255, 0);}
                if (current_cmd.green2){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 255, 0);}
                if (current_cmd.blue2){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 255, 0);}
            }

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(current_cmd.blink_interval_ms));

            if (is_first_state)
            {
                if (current_cmd.red){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 0, 0);}
                if (current_cmd.green){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 0, 0);}
                if (current_cmd.blue){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 0, 0);}
            }
            else
            {
                if (current_cmd.red2){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 0, 0);}
                if (current_cmd.green2){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 0, 0);}
                if (current_cmd.blue2){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 0, 0);}
            }
            
            is_first_state = !is_first_state;
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(current_cmd.blink_interval_ms));
            break;

        case LED_CMD_FADE:
            for (int i = 0; i < 255; i++)
            {
                if (current_cmd.blue){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, i, 0);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                if (current_cmd.green){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, i, 0);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                if (current_cmd.red){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, i, 0);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }

            for (int i = 255; i > 0; i--)
            {

                if (current_cmd.blue){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, i, 0);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                if (current_cmd.green){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, i, 0);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                if (current_cmd.red){
                    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, i, 0);
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
            break;

        case LED_CMD_STOP:
            ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, RED_CHANNEL, 0, 0);
            ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, GREEN_CHANNEL, 0, 0);
            ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, BLUE_CHANNEL, 0, 0);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
            break;
        }
    }
}

void init_led_control()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO) | (1ULL << LED_BLUE_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    ledc_channel_config_t blue_channel = {
        .gpio_num = LED_BLUE_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BLUE_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&blue_channel);

    ledc_channel_config_t green_channel = {
        .gpio_num = LED_GREEN_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = GREEN_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&green_channel);

    ledc_channel_config_t red_channel = {
        .gpio_num = LED_RED_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = RED_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&red_channel);

    ledc_fade_func_install(0);

    led_queue = xQueueCreate(5, sizeof(led_command_t));
    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 5, &led_task_handle);

    ESP_LOGI(TAG, "Advanced LED control system initialized");
}

void set_led_fade(bool red, bool green, bool blue)
{
    led_command_t cmd = {LED_CMD_FADE, red, green, blue};
    xQueueSend(led_queue, &cmd, 0);
}

void set_led_solid(bool red, bool green, bool blue)
{
    led_command_t cmd = {LED_CMD_SOLID, red, green, blue};
    xQueueSend(led_queue, &cmd, 0);
}

void set_led_blink(bool red, bool green, bool blue, int blink_interval_ms)
{
    led_command_t cmd = {LED_CMD_BLINK, red, green, blue, false, false, false, blink_interval_ms};
    xQueueSend(led_queue, &cmd, 0);
}

void set_led_blink_alternate(bool red1, bool green1, bool blue1,
                             bool red2, bool green2, bool blue2,
                             int blink_interval_ms)
{
    led_command_t cmd = {
        LED_CMD_BLINK_ALTERNATE,
        red1, green1, blue1,
        red2, green2, blue2,
        blink_interval_ms};
    xQueueSend(led_queue, &cmd, 0);
}

void stop_leds()
{
    led_command_t cmd = {LED_CMD_STOP};
    xQueueSend(led_queue, &cmd, 0);
}