#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "xag_nvs_component.h"

const char *STORAGE_NAMESPACE = "table_s_i_f";

/**
 * @brief Initializes the NVS flash.
 *
 * This function initializes the NVS flash. If the initialization fails due to no free pages or a new version being found,
 * it erases the NVS flash and retries the initialization.
 *
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Saves a struct to NVS.
 *
 * @param namespace The namespace to use for NVS.
 * @param data Pointer to the struct to be saved.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t save_struct_to_nvs(const char *namespace, void *data, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err;

    // Open NVS namespace
    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return err;
    }

    // Store the data blob in NVS using the key "nvs_table"
    err = nvs_set_blob(handle, "nvs_table", data, size);
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    // Close NVS handle
    nvs_close(handle);
    return err;
}

/**
 * @brief Reads a struct from NVS.
 *
 * @param namespace The namespace to use for NVS.
 * @param err Pointer to an esp_err_t variable to store the result.
 * @return MyStruct Returns the struct read from NVS.
 */
MyStruct read_struct_from_nvs(const char *namespace, esp_err_t *err)
{
    nvs_handle_t handle;
    MyStruct data = {0};
    size_t required_size;

    *err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (*err != ESP_OK)
        return data;

    *err = nvs_get_blob(handle, "nvs_table", NULL, &required_size);
    if (*err != ESP_OK && *err != ESP_ERR_NVS_NOT_FOUND)
        return data;

    if (required_size != sizeof(MyStruct))
    {
        *err = ESP_ERR_NVS_INVALID_LENGTH;
        return data;
    }

    *err = nvs_get_blob(handle, "nvs_table", &data, &required_size);

    nvs_close(handle);
    return data;
}

/**
 * @brief Deletes a struct from NVS.
 *
 * @param namespace The namespace to use for NVS.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t delete_struct_from_nvs(const char *namespace)
{
    nvs_handle_t handle;
    esp_err_t err;

    // Open NVS handle in read-write mode
    err = nvs_open(namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // Erase the key "data"
    err = nvs_erase_key(handle, "nvs_table");
    if (err == ESP_OK)
    {
        // Commit changes
        err = nvs_commit(handle);
    }

    // Close NVS handle
    nvs_close(handle);
    return err;
}

/**
 * @brief Saves a string to NVS.
 *
 * @param key The key to use for NVS.
 * @param value The string to be saved.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t save_string_to_nvs(const char *key, const char *value)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_str(handle, key, value);
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

/**
 * @brief Reads a string from NVS.
 *
 * @param key The key to use for NVS.
 * @param value Pointer to the buffer to store the string.
 * @param max_len The maximum length of the buffer.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t read_string_from_nvs(const char *key, char *value, size_t max_len)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_get_str(handle, key, value, &max_len);

    nvs_close(handle);
    return err;
}

/**
 * @brief Saves an integer to NVS.
 *
 * @param key The key to use for NVS.
 * @param value The integer to be saved.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t save_int_to_nvs(const char *key, int value)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_i32(handle, key, value);
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

/**
 * @brief Reads an integer from NVS.
 *
 * @param key The key to use for NVS.
 * @param value Pointer to an integer to store the read value.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t read_int_from_nvs(const char *key, int *value)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_get_i32(handle, key, value);

    nvs_close(handle);
    return err;
}

/**
 * @brief Saves a float to NVS.
 *
 * @param key The key to use for NVS.
 * @param value The float to be saved.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t save_float_to_nvs(const char *key, float value)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(handle, key, &value, sizeof(float));
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

/**
 * @brief Reads a float from NVS.
 *
 * @param key The key to use for NVS.
 * @param value Pointer to a float to store the read value.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t read_float_from_nvs(const char *key, float *value)
{
    nvs_handle_t handle;
    esp_err_t err;
    size_t required_size = sizeof(float);

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_get_blob(handle, key, value, &required_size);

    nvs_close(handle);
    return err;
}
