MQTT Component for ESP32


Overview
This MQTT component showcases the integration of an MQTT client on an ESP32 microcontroller using the ESP-IDF framework. The MQTT client establishes a connection with an MQTT broker, subscribes to designated topics, and efficiently manages incoming messages. Additionally, it is equipped to handle critical MQTT events such as connection establishment, disconnection, and error detection.

Features
Integration of MQTT client on ESP32 microcontroller.
Connection establishment with an MQTT broker.
Subscription to specified MQTT topics.
Handling of MQTT events including connection, disconnection, subscription, publication, data reception, and errors.
Robust error handling and logging.





Usage
Clone or download the repository.
Integrate the mqtt_component.c and mqtt_component.h files into your ESP-IDF project.
Configure the MQTT broker URL, username, password, and other settings in mqtt_component.h.
Include mqtt_component.h in your main application file.
Initialize the MQTT component using mqtt_component_init() function.
Subscribe to MQTT topics using mqtt_subscribe() function.
Publish messages to MQTT topics using mqtt_publish() function.


Requirements
ESP32 microcontroller
ESP-IDF framework
Access to an MQTT broker


Configuration
The following configurations can be adjusted in mqtt_component.h:

CONFIG_BROKER_URL: URL of the MQTT broker.
CONFIG_USER_NAME: Username for MQTT authentication.
CONFIG_PASSWORD: Password for MQTT authentication.
Other MQTT client configurations such as client ID, QoS level, etc., can be modified directly in the mqtt_component_init() function.


Example
// Include the MQTT component header file
#include "mqtt_component.h"

void app_main() {
    // Initialize the MQTT component
    mqtt_component_init();

    // Subscribe to an MQTT topic
    mqtt_subscribe("example/topic", 0);

    // Publish a message to an MQTT topic
    mqtt_publish("example/topic", "Hello, MQTT!", 1, 0);
}


Original Author: Mohd Aman
Source of the Code: ESP-IDF example code