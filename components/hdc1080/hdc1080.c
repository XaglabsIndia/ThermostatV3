/* 
This is HDC1080 temperature and humidity sensor application driver in esp-idf based on the 
usermanual of HDC1080 sensor 
https://www.ti.com/lit/gpn/hdc1080

and esp idf I2C documentation (ESP-IDF v5.0-dirty) 
https://docs.espressif.com/projects/esp-idf/en/release-v5.0/esp32/api-reference/peripherals/i2c.html

 Responsible person : KRISHNA.TL 
 Date written       : 06-02-2024

*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "hdc1080.h"
static const char *HDC_1080 = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           33      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           32      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define HDC_1080_SENSOR_ADDR                 0x40        /*!< Slave address of the HDC1080 sensor */
#define HDC_1080_SENSOR_CONFIG_REG           0x02        /*!< Register addresses of the configuration register */
#define HDC_1080_SENSOR_CONFIG_VALUE           0x15       /* configuration value of hdc1080 */
#define HDC_1080_SENSOR_TEMP_POINTER           0x00     /* temperature pointer of hdc1080 */
#define HDC_1080_SENSOR_HUMID_POINTER           0x01    /* humidity pointer of hdc1080 */
#define CONVERSION_TIME                         500     /* (milli seconds) conversion time for sensor */

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(int sda,int scl)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/// @brief this function is used to initialise the sensor and configuring with required parameters
/// @param sda Synchrounous data pin for sensor communication
/// @param scl Synchrounous clock pin for sensor communication
/// @return 
//      ESP_OK if all success
//      ESP_FAIL if pins are not connected properly
esp_err_t HDC1080_sensor_init_and_config(int sda, int scl)
{
  uint8_t data[2];
  ESP_ERROR_CHECK(i2c_master_init(sda, scl));
  // ESP_LOGI(HDC_1080, "I2C initialized successfully");
  uint8_t regaddr[2] = {HDC_1080_SENSOR_CONFIG_REG, HDC_1080_SENSOR_CONFIG_VALUE};
  ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, &regaddr, 2, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
  if (((data[0] << 8) | data[1]) == (HDC_1080_SENSOR_CONFIG_VALUE<<8))
  {
    //  ESP_LOGI(HDC_1080, "I2C configured successfully");
    return ESP_OK;
  }
  return ESP_FAIL;
}
/// @brief this function gives directly the temperature value of the HDC1080 sensor
/// @return returns the temperature value in degree celsius (C)
uint8_t HDC1080_temp_data()
{
    ESP_ERROR_CHECK(HDC1080_sensor_init_and_config(I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO));
    uint8_t regaddr = HDC_1080_SENSOR_TEMP_POINTER,data[2];
    uint32_t temperature=0,temperature1=0;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, &regaddr, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    vTaskDelay(CONVERSION_TIME / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    temperature=((data[0] << 8) | data[1]);
    temperature1=temperature*165; // 2^7 =128
    temperature=(temperature1/65536);
    temperature1=(temperature-40);
    return temperature1;
}
/// @brief this function gives directly the Humidity value of the HDC1080 sensor
/// @return returns the Humidity value in Relative Humidity (% RH)
uint8_t HDC1080_humid_data()
{
    ESP_ERROR_CHECK(HDC1080_sensor_init_and_config(I2C_MASTER_SDA_IO,I2C_MASTER_SCL_IO));
    uint8_t regaddr = HDC_1080_SENSOR_HUMID_POINTER,data[2];
    uint32_t humidity=0,humidity1=0;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, &regaddr, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    vTaskDelay((2*CONVERSION_TIME) / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    humidity=((data[0] << 8) | data[1]);
    humidity1=humidity*100;
    humidity=(humidity1/65536);
  return humidity;
}

/* example for testing hdc1080 component

void app_main(void)
{
  while (1)
  {
    ESP_LOGI(HDC_1080, "temperature data %d c ", HDC1080_temp_data());
    ESP_LOGI(HDC_1080, "humidity data %d ", HDC1080_humid_data());
  }
  
}
*/