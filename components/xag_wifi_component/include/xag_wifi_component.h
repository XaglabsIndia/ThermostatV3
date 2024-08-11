#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <esp_wifi.h>
#include "esp_event.h"

#include "nvs_flash.h"
#include "nvs.h"


//#include "xag_nvs_component.h"

esp_err_t wifi_init_sta(char* ssid, char* pwd);
esp_err_t connectWifi();
void StoreHardcodeWiFiData(const char *HardCodeSSID, const char *HardCodePassword);

#pragma once

typedef struct ConnectFlags
{
  bool MQTT_connect_flag;  // To make sure MQTT is connected before displaying any image.    // To check if the time count reaches the threshold set for changing display based on active beacon at that moment
  bool WiFi_connect_flag;
  bool FTP_connect_flag;
}ConnectFlags;
