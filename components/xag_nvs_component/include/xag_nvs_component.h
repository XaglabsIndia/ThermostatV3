#ifndef XAG_NVS_COMPONENT_H
#define XAG_NVS_COMPONENT_H

#include "esp_err.h"

/**
 * @brief Structure to hold data for NVS operations.
 * **Modify `MyStruct`**: Update `MyStruct` according to your application's data structure needs.
 */
typedef struct {
    int intValue;           // Integer value
    float floatValue;       // Float value
    char stringValue[64];   // String value
} MyStruct;


esp_err_t init_nvs();

esp_err_t save_struct_to_nvs(const char *namespace, void *data, size_t size);
MyStruct read_struct_from_nvs(const char *namespace, esp_err_t *err);
esp_err_t delete_struct_from_nvs(const char *namespace);
esp_err_t save_string_to_nvs(const char *key, const char *value);
esp_err_t read_string_from_nvs(const char *key, char *value, size_t max_len);
esp_err_t save_int_to_nvs(const char *key, int value);
esp_err_t read_int_from_nvs(const char *key, int *value);
esp_err_t save_float_to_nvs(const char *key, float value);
esp_err_t read_float_from_nvs(const char *key, float *value);

#endif // XAG_NVS_COMPONENT_H
