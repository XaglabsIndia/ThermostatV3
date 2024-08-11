#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
#include "xag_ota_component.h"


TaskHandle_t MultiButtonTaskHandle = NULL;
SemaphoreHandle_t MultiButtonSemaphore;
extern TaskHandle_t AutoProvision;
extern TaskHandle_t LoraRXContiniousTaskHandle ;
extern TaskHandle_t LoraRXContiniousMessageQueueTaskHandle ;
extern TaskHandle_t HeapTaskHandle;
extern TaskHandle_t KeepAliveHandle;
extern TaskHandle_t TemperatureMonitorHandle;
extern int TaskResetDecision;

static const char* TAG = "MultiButton"; 

void IRAM_ATTR MultiButtonISR(void *arg)
{
    xTaskResumeFromISR(MultiButtonTaskHandle);
}

void MultiButton(void *pvParameter)
{
    float pressLength_milliSeconds = 0;
    vTaskSuspend(NULL);
    while(1){
      
    while (gpio_get_level(CONFIG_MULTIBUTTON_GPIO) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for button debounce
            pressLength_milliSeconds += 100;
            ESP_LOGI(TAG, "Button press duration: %.2f ms", pressLength_milliSeconds);
            if (pressLength_milliSeconds > 500 && pressLength_milliSeconds < 4999)
            {
                gpio_set_level(CONFIG_GREEN_LED_GPIO, 0);
                gpio_set_level(CONFIG_BLUE_LED_GPIO,0);
                gpio_set_level(CONFIG_RED_LED_GPIO, 1);
            }
            if (pressLength_milliSeconds > 5000 && pressLength_milliSeconds < 9999)
            {
                gpio_set_level(CONFIG_GREEN_LED_GPIO, 1);
                gpio_set_level(CONFIG_RED_LED_GPIO, 1);
                gpio_set_level(CONFIG_BLUE_LED_GPIO, 0);
            }
            if (pressLength_milliSeconds > 10000)
            {
                gpio_set_level(CONFIG_BLUE_LED_GPIO, 0);
                gpio_set_level(CONFIG_RED_LED_GPIO, 0);
                gpio_set_level(CONFIG_GREEN_LED_GPIO, 1);
            }
         }
    ESP_LOGI(TAG,"Total Value pressed is %.2f", pressLength_milliSeconds);
    int DurationPassed = (int)pressLength_milliSeconds;
    ESP_LOGI(TAG,"converted Value pressed is %d", DurationPassed);
    PerformAction(DurationPassed);
    pressLength_milliSeconds = 0;
    vTaskSuspend(NULL);
      }

    }


void PerformAction(uint32_t duration)
 {      
int retrie = 0;
const int MAX_RETRIE = 5;
char TempString = "ACK";             
  switch (duration) 
      {                 
        case 0 ... 5000:                     
        printf("Button 1 short press\n");                     
        break;                 
        case 5001 ... 10000:                     
        printf("Manual Provision Started\n");
       if (TaskResetDecision == 0){
        vTaskSuspend(LoraRXContiniousTaskHandle);
         vTaskDelay(50);
       }
        vTaskResume(AutoProvision);           
        break;                 
        case 10001 ... 15000:                     
        esp_restart();                
        break;                 
        default:                     
        break;
      }
  }

void SetupMultiButton(){
printf("CONFIG_MULTIBUTTON_GPIO %d",CONFIG_MULTIBUTTON_GPIO);
AttachInterrupt(CONFIG_MULTIBUTTON_GPIO, MultiButtonISR, GPIO_INTR_ANYEDGE);
gpio_isr_handler_add(CONFIG_MULTIBUTTON_GPIO, MultiButtonISR, (void *)CONFIG_MULTIBUTTON_GPIO);
MultiButtonSemaphore = xSemaphoreCreateBinary();
xTaskCreate(MultiButton, "MultiButton", 4000, NULL, configMAX_PRIORITIES - 1, &MultiButtonTaskHandle);
printf("Button duration detection started\n");
}


/////////////////////END////////////////////////////////////////////////////////////