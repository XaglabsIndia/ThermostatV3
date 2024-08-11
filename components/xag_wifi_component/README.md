
# Wi-Fi Component for ESP32

This repository contains code for managing Wi-Fi connections on ESP32 devices. It includes functions for connecting to Wi-Fi networks using hardcoded, old, and new credentials stored in Non-Volatile Storage (NVS). Additionally, it handles event-driven Wi-Fi connection retries and status monitoring.

## Features

- **Wi-Fi Initialization**: Initialize ESP32 in station mode and connect to a Wi-Fi network using provided credentials.
- **Event Handling**: Monitor Wi-Fi events such as connection status changes and IP address acquisition.
- **Credential Management**: Store and manage multiple sets of Wi-Fi credentials using NVS.
- **Retry Mechanism**: Automatically retry connecting to Wi-Fi up to a specified maximum number of attempts.

## Dependencies

- **ESP-IDF**: This project is built using the Espressif IoT Development Framework (ESP-IDF).

1. **Configure Wi-Fi Credentials**:
   - Update `xag_nvs_component.h` to define NVS keys (`HardCodeSSID`, `HardPassword`, `OldWifiSSID`, `OldWifiPwd`, `NewWifiSSID`, `NewWifiPwd`) as per your application requirements.


## Usage

### Initializing Wi-Fi

To initialize and connect to a Wi-Fi network, use the `connectWifi` function. It attempts to connect using hardcoded, old, and new credentials sequentially until a successful connection is established or all options fail.

```c
esp_err_t status = connectWifi();
if (status == ESP_OK) {
    // Wi-Fi connected successfully
} else {
    // Failed to connect
}
```

### Managing Wi-Fi Credentials

To store hardcoded Wi-Fi credentials or update them, use the following functions:

- `StoreHardcodeWiFiData`: Store hardcoded SSID and password in NVS.
- `write_wifi_data`: Update and store new SSID and password while preserving old credentials.

```c
StoreHardcodeWiFiData("mySSID", "myPassword");
write_wifi_data("newSSID", "newPassword");
```
