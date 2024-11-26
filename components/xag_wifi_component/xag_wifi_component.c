#include "xag_wifi_component.h"
#include "xag_nvs_component.h"

/*
 * xag_wifi_component.c
 *
 *  Created on: 12-Jul-2023
 *      Author: Shruthi KR
 */


/*
 * xag_wifi_component.c
 *
 *  Created on: 19-Jul-2024
 *      Author: Mohd Aman
 * try 3 SSID
 */

ConnectFlags connectFlags; // To check connectivity of Wi-Fi, MQTT, FTP
char HardCodeSSID[30];
char HardCodePassword[30];
// Define thresholds and maximum retry attempts for Wi-Fi connection
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define MAXIMUM_RETRY 5

// Define event bits for Wi-Fi connection success and failure
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Forward declaration of the Wi-Fi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/* FreeRTOS event group to signal Wi-Fi connection events */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */

// Tag for logging purposes
static const char *TAG = "wifi";

// Counter for retry attempts
static int s_retry_num = 0;

/**
 * @brief Wi-Fi event handler.q
 * Handles various Wi-Fi events such as start, disconnect, and IP acquisition.
 *
 * @param arg User-defined argument.
 * @param event_base Event base identifier.
 * @param event_id Event ID.
 * @param event_data Event data.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Initiate connection to the Wi-Fi network.
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAXIMUM_RETRY)
        { 
            esp_wifi_connect(); // Retry connecting to the Wi-Fi network.
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Set fail bit if retries exceed maximum.
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); // Log obtained IP address.
        s_retry_num = 0; // Reset retry counter
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Set connected bit on success.
    }
}

/**
 * @brief Initialize Wi-Fi in station mode.
 *
 * Configures and starts the Wi-Fi in station mode with the provided SSID and password.
 *
 * @param local_ssid SSID of the local Wi-Fi network.
 * @param local_pwd Password of the local Wi-Fi network.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_init_sta(char *local_ssid, char *local_pwd)
{
    printf("local ssid=%s, pwd=%s\n", local_ssid, local_pwd);

    s_wifi_event_group = xEventGroupCreate(); // Create event group for Wi-Fi events.

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD, // Set authentication mode threshold.
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, // Configure WPA3 settings.
        },
    };

    // Copy SSID and password into Wi-Fi configuration structure.
    strncpy((char *)wifi_config.sta.ssid, local_ssid, 32);
    strncpy((char *)wifi_config.sta.password, local_pwd, 64);

    // Set Wi-Fi mode to station mode and apply the configuration.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Start the Wi-Fi driver.

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait for Wi-Fi connection or failure.
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdTRUE, // Clear bits on exit.
                                           pdFALSE, // Wait for any bit.
                                           portMAX_DELAY); // Wait indefinitely.

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", local_ssid, local_pwd);
        connectFlags.WiFi_connect_flag = true; // Set Wi-Fi connection flag.
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", local_ssid, local_pwd);
        connectFlags.WiFi_connect_flag = false; // Clear Wi-Fi connection flag.
        vEventGroupDelete(s_wifi_event_group); // Delete event group.
        esp_wifi_stop(); // Stop Wi-Fi driver.
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }
}

/**
 * @brief Store hardcoded Wi-Fi credentials in NVS.
 *
 * @param HardCodeSSID Hardcoded SSID.
 * @param HardCodePassword Hardcoded password.
 */
void StoreHardcodeWiFiData(const char *HardCodeSSID, const char *HardCodePassword)
{
    save_string_to_nvs("HardCodeSSID", HardCodeSSID); // Save SSID to NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS); // Delay for NVS write stability.
    save_string_to_nvs("HardPassword", HardCodePassword); // Save password to NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS); // Delay for NVS write stability.
        
    read_string_from_nvs("HardCodeSSID", HardCodeSSID, sizeof(HardCodeSSID)); // Read hardcoded SSID from NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS);
    read_string_from_nvs("HardPassword", HardCodePassword, sizeof(HardCodePassword)); // Read hardcoded password from NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS);

    printf("HardCodeSSID     : %s\n", HardCodeSSID);
    printf("HardCodePassword : %s\n", HardCodePassword);
}

/**
 * @brief Attempt to connect to Wi-Fi using stored credentials.
 *
 * Tries connecting to Wi-Fi using hardcoded, old, and new credentials in sequence.
 *
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t connectWifi()
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA BEGIN");

    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta(); // Create default Wi-Fi STA.

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Initialize Wi-Fi with default configuration.

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    // Register event handlers for Wi-Fi and IP events.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    if (read_string_from_nvs("HardCodeSSID", HardCodeSSID, sizeof(HardCodeSSID)) == 4354) {
        StoreHardcodeWiFiData(CONFIG_HC_SSID, CONFIG_HC_PASSWORD);
    }
    // Try hardcoded SSID and password first
    printf("               ************   Try HardCode SSID   ************\n");

    read_string_from_nvs("HardCodeSSID", HardCodeSSID, sizeof(HardCodeSSID)); // Read hardcoded SSID from NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS);
    read_string_from_nvs("HardPassword", HardCodePassword, sizeof(HardCodePassword)); // Read hardcoded password from NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS);

    printf("HardCodeSSID     : %s\n", HardCodeSSID);
    printf("HardCodePassword : %s\n", HardCodePassword);
    if (wifi_init_sta(HardCodeSSID, HardCodePassword) == ESP_OK)
    {
        return ESP_OK;
    }

    char SsidNew[30];
    char SsidOld[30];

    char PasswordNew[30];
    char PasswordOld[30];

    // If hardcoded credentials failed, try "OldWifiSSID" and "OldWifiPwd"
    printf("               ************   Try Old SSID   ************\n");
    read_string_from_nvs("OldWifiSSID", SsidOld, sizeof(SsidOld)); // Read old SSID from NVS.
    read_string_from_nvs("OldWifiPwd", PasswordOld, sizeof(PasswordOld)); // Read old password from NVS.
    s_retry_num = 0;
    if (wifi_init_sta(SsidOld, PasswordOld) == ESP_OK)
    {
        return ESP_OK;
    }
    printf("               ************   Try New SSID   ************\n");

    // If old credentials failed, try "NewWifiSSID" and "NewWifiPwd"
    read_string_from_nvs("NewWifiSSID", SsidNew, sizeof(SsidNew)); // Read new SSID from NVS.
    read_string_from_nvs("NewWifiPwd", PasswordNew, sizeof(PasswordNew)); // Read new password from NVS.
    s_retry_num = 0;
    if (wifi_init_sta(SsidNew, PasswordNew) == ESP_OK)
    {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Unable to connect to any known wifi.");
    esp_restart();
    return ESP_FAIL;
}

/**
 * @brief Write new Wi-Fi credentials to NVS and update old credentials.
 *
 * @param ssid New SSID.
 * @param password New password.
 */
void write_wifi_data(const char *ssid, const char *password)
{
    char SsidNew[30];
    char SsidOld[30];

    char PasswordNew[30];
    char PasswordOld[30];

    vTaskDelay(200 / portTICK_PERIOD_MS);
    read_string_from_nvs("NewWifiSSID", SsidNew, sizeof(SsidNew)); // Read current new SSID from NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS);
    read_string_from_nvs("NewWifiPwd", PasswordNew, sizeof(PasswordNew)); // Read current new password from NVS.
    vTaskDelay(200 / portTICK_PERIOD_MS);
    printf("NEW SSID     : %s\n", SsidNew);
    printf("NEW PASSWORD : %s\n", PasswordNew);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Backup current new credentials to old credentials
    strcpy(SsidOld, SsidNew);
    strcpy(PasswordOld, PasswordNew);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    save_string_to_nvs("OldWifiSSID", SsidOld); // Save old SSID to NVS.
    vTaskDelay(50 / portTICK_PERIOD_MS);
    save_string_to_nvs("OldWifiPwd", PasswordOld); // Save old password to NVS.
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Save new credentials to NVS.
    save_string_to_nvs("NewWifiSSID", ssid);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    save_string_to_nvs("NewWifiPwd", password);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Verify the stored credentials.
    read_string_from_nvs("OldWifiSSID", SsidOld, sizeof(SsidOld)); // Read back old SSID to verify.
    vTaskDelay(50 / portTICK_PERIOD_MS);
    read_string_from_nvs("OldWifiPwd", PasswordOld, sizeof(PasswordOld)); // Read back old password to verify.
    vTaskDelay(50 / portTICK_PERIOD_MS);
    read_string_from_nvs("NewWifiSSID", SsidNew, sizeof(SsidNew)); // Read back new SSID to verify.
    vTaskDelay(50 / portTICK_PERIOD_MS);
    read_string_from_nvs("NewWifiPwd", PasswordNew, sizeof(PasswordNew)); // Read back new password to verify.
    vTaskDelay(50 / portTICK_PERIOD_MS);
    printf("OLD SSID     : %s\n", SsidOld);
    printf("OLD PASSWORD : %s\n", PasswordOld);
    printf("NEW SSID     : %s\n", SsidNew);
    printf("NEW PASSWORD : %s\n", PasswordNew);
}
