/*
 * Filename: mqtt_component.cpp 
 * 
 * Author: Mohd Aman 
 * 
 * Date: 2024-05-22 
 * 
 * Description: This MQTT component showcases the integration of an MQTT client on an ESP32 using the ESP-IDF framework. 
 * The MQTT client establishes a connection with an MQTT broker, subscribes to designated topics, and efficiently manages incoming messages. 
 * It is also equipped to handle critical MQTT events like connection establishment, disconnection, and error detection.
 * 
 * Original Author: ESP-IDF
 * 
 * Source of the Code: mqtt example code from esp-idf example code
 */

#include "esp_system.h"       // Includes system-wide ESP-IDF functions and macros
#include "esp_event.h"        // Includes event loop library functions
#include "mqtt_client.h"      // Includes the MQTT client library
#include "esp_err.h"
// Extern declaration of the MQTT client handle, making it accessible across different files
extern esp_mqtt_client_handle_t client;

// Function prototype for initializing the MQTT component
esp_err_t mqtt_component_init(void);

/**
 * @brief Subscribes to an MQTT topic.
 * 
 * This function allows the client to subscribe to a specified topic with a given Quality of Service (QoS) level.
 * 
 * @param topic The MQTT topic to subscribe to.
 * @param qos The Quality of Service level for the subscription.
 */
esp_err_t mqtt_subscribe(const char *topic, int qos);

/**
 * @brief Publishes a message to an MQTT topic.
 * 
 * This function allows the client to publish a message to a specified topic with given QoS and retain settings.
 * 
 * @param topic The MQTT topic to publish the message to.
 * @param message The message to be published.
 * @param qos The Quality of Service level for the message.
 * @param retain Whether to retain the message on the broker.
 */
void mqtt_publish(const char *topic, const char *message, int qos, int retain);

/**
 * @brief Handles incoming MQTT data.
 * 
 * This function processes data received from subscribed MQTT topics. It can be used anywhere in the project directory.
 * 
 * @param data The incoming data from the MQTT topic.
 * @param len The length of the incoming data.
 */
void handle_mqtt_data(const char *data, size_t len);

char GetMQTTTopic(char* Topic);
