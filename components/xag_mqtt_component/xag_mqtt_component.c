/*
 * Filename: xag_mqtt.c
 * 
 * Author: Mohd Aman 
 * 
 * Date: 2024-05-22 
 * 
 * Description: This MQTT component showcases the integration of an MQTT client on an ESP32 using the ESP-IDF framework. 
 * The MQTT client establishes a connection with an MQTT broker, subscribes to designated topics, and efficiently manages incoming messages. 
 * It is also equipped to handle critical MQTT events like connection establishment, disconnection, and error detection.
 * 
 * Original Author: Mohd Aman
 * 
 * Source of the Code: mqtt example code from esp-idf example code
 */


#include "xag_mqtt_component.h"
#include "xag_nvs_component.h"
#include "esp_log.h"
#include "cJSON.h"
#include <stdio.h>
#include "driver/gpio.h"
#include <stdbool.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_mac.h"
#define MAX_TOPIC_LENGTH 128
#define NVS_KEY_SUBSCRIBE "mqtt_sub_topic"
#define NVS_KEY_PUBLISH "mqtt_pub_topic"

static char CurrentTopic[100];  // To store the current topic
static const char *TAG = "XAG - MQTT";
// MQTT client handle
esp_mqtt_client_handle_t client;
extern char current_subscribe_topic[MAX_TOPIC_LENGTH];
/**
 * @brief Logs errors if the error code is non-zero.
 * 
 * This function logs an error message with the error code if the code is non-zero.
 * 
 * @param message The error message to log.
 * @param error_code The error code to check.
 */
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
/**
 * @brief MQTT event handler function.
 * 
 * This function processes different MQTT events such as connection, disconnection,
 * subscription, publication, data reception, and errors.
 * 
 * @param handler_args Arguments for the event handler.
 * @param base The event base.
 * @param event_id The event ID.
 * @param event_data The event data.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);

    // Cast the event data to MQTT event handle type
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    // Handle different types of MQTT events
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        // Restart the ESP in case of disconnection
        esp_restart();
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // Optional logging or actions when subscribed
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        
            handle_mqtt_data(event->data, event->data_len);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        // Log different types of transport errors
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/**
 * @brief Initializes the MQTT component.
 * 
 * This function configures and starts the MQTT client with the specified settings.
 */
// void mqtt_component_init(void)
// {
//     // Generate a unique client ID
//     char client_id[23]; // Increase size to accommodate larger ID
//     uint8_t mac[6];
//     esp_read_mac(mac, ESP_MAC_WIFI_STA);
//     snprintf(client_id, sizeof(client_id), "nems-%02X%02X%02X%02X%02X%02X", 
//              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

//     ESP_LOGI(TAG, "Generated MQTT Client ID: %s", client_id);

//     // MQTT client configuration structure
//     esp_mqtt_client_config_t mqtt_cfg = {
//         .broker.address.uri = CONFIG_BROKER_URL,  // Broker URL
//         .credentials.username = CONFIG_USER_NAME,
//         .credentials.authentication.password = CONFIG_PASSWORD,
//         .credentials.client_id = client_id,     // Client ID
//     };

//     // Initialize the MQTT client with the configuration
//     client = esp_mqtt_client_init(&mqtt_cfg);

//     // Register the event handler for all MQTT events
//     esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);

//     // Start the MQTT client
//     esp_mqtt_client_start(client);
// }
esp_err_t mqtt_component_init(void)
{
    esp_err_t ret = ESP_OK;

    // Generate a unique client ID
    char client_id[23]; // Increase size to accommodate larger ID
    uint8_t mac[6];
    ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC address: %s", esp_err_to_name(ret));
        return ret;
    }

    snprintf(client_id, sizeof(client_id), "nems-%02X%02X%02X%02X%02X%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "CONFIG_BROKER_URL: %s", CONFIG_BROKER_URL);
    ESP_LOGI(TAG, "CONFIG_USER_NAME: %s", CONFIG_USER_NAME);
    ESP_LOGI(TAG, "CONFIG_PASSWORD: %s", CONFIG_PASSWORD);
    ESP_LOGI(TAG, "Generated MQTT Client ID: %s", client_id);

    // MQTT client configuration structure
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,  // Broker URL
        .credentials.username = CONFIG_USER_NAME,
        .credentials.authentication.password = CONFIG_PASSWORD,
        .credentials.client_id = client_id,     // Client ID
    };

    // Initialize the MQTT client with the configuration
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    // Register the event handler for all MQTT events
    ret = esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(client);
        return ret;
    }

    // Start the MQTT client
    ret = esp_mqtt_client_start(client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(client);
        return ret;
    }

    ESP_LOGI(TAG, "MQTT client initialized and started successfully");
    return ESP_OK;
}

/**
 * @brief Subscribes to a specific MQTT topic.
 * 
 * This function subscribes to a given MQTT topic with the specified Quality of Service (QoS) level.
 * 
 * @param topic The MQTT topic to subscribe to.
 * @param qos The Quality of Service level for the subscription.
 */
esp_err_t mqtt_subscribe(const char *topic, int qos)
{
    esp_err_t ret = ESP_OK;
    // Check that the topic is not null
    if (topic == NULL) {
        ESP_LOGE(TAG, "Topic is null");
        return ESP_FAIL;
    }

    // Call the MQTT subscribe function
    int msg_id = esp_mqtt_client_subscribe_single(client, topic, qos);
    if (msg_id >= 0)
    {
        ESP_LOGI(TAG, "Subscribed to topic %s, msg_id=%d", topic, msg_id);
        return ret;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to subscribe to topic %s", topic);
        return ESP_FAIL;
    }
}

/**
 * @brief Publishes a message to a specific MQTT topic.
 * 
 * This function publishes a message to a given MQTT topic with the specified Quality of Service (QoS) level and retain flag.
 * 
 * @param topic The MQTT topic to publish the message to.
 * @param message The message to be published.
 * @param qos The Quality of Service level for the message.
 * @param retain Whether to retain the message on the broker.
 */
void mqtt_publish(const char *topic, const char *message, int qos, int retain)
{
    // Check that the topic and message are not null
    if (topic == NULL || message == NULL) {
        ESP_LOGE(TAG, "Topic or message is null");
        return;
    }

    // Call the MQTT publish function
    int msg_id = esp_mqtt_client_publish(client, topic, message, 0, qos, retain);
    if (msg_id >= 0)
    {
        ESP_LOGI(TAG, "Published message to topic %s, msg_id=%d", topic, msg_id);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to publish message to topic %s", topic);
    }
}
