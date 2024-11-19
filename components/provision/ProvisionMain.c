#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "ProvisionHeader.h"
#include "xag_nvs_component.h"
#include "esp_err.h"
#include <stdio.h>
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


char HUBKey[10] = "HUBID";
char DevKey[10] = "DevID";
char ThermoKey[10] = "ThermoID";
char LoraData[255];
int Value;
int InitValue = 0;
int CONFIG_DEV_ID = 10;
time_t CurrentTime, Timeout;
struct tm* local_time;

esp_err_t LoraDataStatus =(-1);

esp_err_t CheckStoredKeyStatus(char* Key, int* Value) {
esp_err_t RetValue = init_nvs();
if (RetValue != ESP_OK) 
{
    printf("Error initializing NVS: %d\n", RetValue);
}
esp_err_t RetValueID = read_int_from_nvs(Key, Value);
if (RetValueID != ESP_OK) {
    printf("Error reading value from NVS: %d\n", RetValueID);
    return RetValueID;
} else {
    printf("Value read from NVS: %d\n", *Value);
}
return ESP_OK;
}


// esp_err_t StartProvision(int TimeInterval, uint8_t Length){
// ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
// esp_task_wdt_reset();
// CurrentTime = time(NULL);
// Timeout = CurrentTime +20;
// esp_err_t CheckDevIDStatus = CheckStoredKeyStatus(DevKey, &Value);
// if(CheckDevIDStatus == 4354){
//    esp_err_t RetValue =save_int_to_nvs(DevKey,CONFIG_DEV_ID);
//    if (RetValue == ESP_OK){
//       esp_restart();
//     }
//   esp_restart();
//  }
// esp_err_t CheckStatus = CheckStoredKeyStatus(HUBKey, &Value);
// if(CheckStatus == 4354){
//   esp_err_t RetValue1 = save_int_to_nvs(HUBKey,InitValue);
//   if (RetValue1 == ESP_OK){
//     esp_restart();
//   }  
//  }
//  //esp_err_t LoraDataStatus = LoraSingleModeRX(1000,3,LoraData);

//  while (LoraDataStatus != 0)
//  {
//    esp_task_wdt_reset();
//   CurrentTime = time(NULL);
//   esp_err_t LoraDataStatus = LoraSingleModeRX(1000,Length,LoraData);
//   if (LoraDataStatus == 0){
//     int LoraDataInt = atoi(LoraData);
//     esp_err_t RetValue2 = save_int_to_nvs(HUBKey,LoraDataInt);
//     if (RetValue2 == ESP_OK){
//       esp_restart();
//     }  
//     printf("Converted integer from string: %d\n", LoraDataInt);
//     break;
//  }
//   esp_task_wdt_reset();
//   local_time = localtime(&CurrentTime);
//   printf("Current time: %s", asctime(local_time));
//   printf("Timeout time: %lld", Timeout);
//  if ((CurrentTime > Timeout) && (Timeout != 0)) {
//   lora_end_rx();
//   printf("Auto Provision ended due to 3 Min timout");
//   return ESP_FAIL;
//   }
//  }
//  ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
//  return ESP_OK;
// }


