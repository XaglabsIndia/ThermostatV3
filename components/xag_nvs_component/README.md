### README: Non-Volatile Storage (NVS) Component

#### Overview

This repository contains an implementation of Non-Volatile Storage (NVS) operations for ESP32 microcontrollers using the ESP-IDF framework. NVS allows storing various types of data persistently on the device even when it's powered off.

#### Features

- **Struct Storage**: Supports storing and retrieving structured data (`MyStruct`) from NVS.
- **String Storage**: Enables saving and loading strings to/from NVS.
- **Integer and Float Storage**: Provides functionality to store and retrieve integer and float values in NVS.

#### Usage

1. **Struct Storage Example**:
   - The provided `MyStruct` in `xag_nvs_component.h` serves as an example.
   - Modify `MyStruct` to suit your data needs, ensuring it fits within the size limits of NVS operations.
   - Use functions like `save_struct_to_nvs`, `read_struct_from_nvs`, and `delete_struct_from_nvs` to manage `MyStruct` data in NVS.

2. **String, Integer, and Float Storage**:
   - Functions `save_string_to_nvs`, `read_string_from_nvs`, `save_int_to_nvs`, `read_int_from_nvs`, `save_float_to_nvs`, and `read_float_from_nvs` handle other data types.
   - Replace placeholders (`STRING_KEY`, `INT_KEY`, `FLOAT_KEY`) with your specific keys.

#### Getting Started

1. **Include the Files**:
   - Ensure `nvs_flash.h`, `nvs.h`, and `xag_nvs_component.h` are accessible in your project.
   - Modify `STORAGE_NAMESPACE` as needed for your application.

2. **Initialize NVS**:
   - Call `init_nvs()` in your application startup to initialize NVS. Handle errors using `esp_err_to_name`.

3. **Data Operations**:
   - Save data using appropriate functions (`save_struct_to_nvs`, `save_string_to_nvs`, etc.).
   - Read data using corresponding functions (`read_struct_from_nvs`, `read_string_from_nvs`, etc.).

4. **Error Handling**:
   - Functions return `esp_err_t`, allowing robust error handling (`ESP_OK` on success, specific error codes on failure).

#### Notes

- **Modify `MyStruct`**: Update `MyStruct` in `xag_nvs_component.h` according to your application's data structure needs.
- **Multiple Structures**: Define additional structures as needed, modifying function calls to accommodate multiple data types.
- **Documentation**: Refer to ESP-IDF documentation for detailed usage of NVS and error codes.

