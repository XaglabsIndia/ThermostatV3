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

#define I2C_MASTER_SCL_IO           41      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           42      /*!< GPIO number used for I2C master data  */
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
static esp_err_t i2c_master_init(int sda, int scl)
{
    int i2c_master_port = I2C_MASTER_NUM;
    
    // Delete any existing driver first
    i2c_driver_delete(i2c_master_port);  // It's okay if this fails

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

esp_err_t HDC1080_sensor_init_and_config(int sda, int scl)
{
    uint8_t data[2];
    esp_err_t ret;

    // Initialize I2C with retry
    ret = i2c_master_init(sda, scl);
    if(ret != ESP_OK) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ret = i2c_master_init(sda, scl);
        if(ret != ESP_OK) {
            return ESP_FAIL;
        }
    }

    // Configure sensor
    uint8_t regaddr[2] = {HDC_1080_SENSOR_CONFIG_REG, HDC_1080_SENSOR_CONFIG_VALUE};
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, 
                                     &regaddr, 2, data, 2, 
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    if (((data[0] << 8) | data[1]) == (HDC_1080_SENSOR_CONFIG_VALUE<<8))
    {
        return ESP_OK;
    }
    return ESP_FAIL;
}

int HDC1080_temp_data()
{
    esp_err_t ret;
    
    // Initialize with retry
    ret = HDC1080_sensor_init_and_config(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if(ret != ESP_OK) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ret = HDC1080_sensor_init_and_config(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        if(ret != ESP_OK) {
            return -127; // Error value
        }
    }

    uint8_t regaddr = HDC_1080_SENSOR_TEMP_POINTER, data[2];
    uint32_t temperature = 0, temperature1 = 0;

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, 
                                   &regaddr, 1, 
                                   I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(ret != ESP_OK) {
        i2c_driver_delete(I2C_MASTER_NUM);
        return -127;
    }

    vTaskDelay(CONVERSION_TIME / portTICK_PERIOD_MS);

    ret = i2c_master_read_from_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, 
                                    data, 2, 
                                    I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(ret != ESP_OK) {
        i2c_driver_delete(I2C_MASTER_NUM);
        return -127;
    }

    // Clean up I2C driver
    i2c_driver_delete(I2C_MASTER_NUM);

    // Calculate temperature (unchanged)
    temperature = ((data[0] << 8) | data[1]);
    temperature1 = temperature * 165;
    temperature = ((temperature1 * 2) / 65536);
    temperature1 = (temperature - 80);

    return (temperature1/2);
}
/// @brief this function gives directly the Humidity value of the HDC1080 sensor
/// @return returns the Humidity value in Relative Humidity (% RH)
uint8_t HDC1080_humid_data()
{
   esp_err_t ret;
   
   // Initialize with retry
   ret = HDC1080_sensor_init_and_config(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
   if(ret != ESP_OK) {
       vTaskDelay(1000 / portTICK_PERIOD_MS);
       ret = HDC1080_sensor_init_and_config(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
       if(ret != ESP_OK) {
           return 0; // Error value for humidity
       }
   }

   uint8_t regaddr = HDC_1080_SENSOR_HUMID_POINTER, data[2];
   uint32_t humidity = 0, humidity1 = 0;
   
   // Write humid register address
   ret = i2c_master_write_to_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, 
                                  &regaddr, 1, 
                                  I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
   if(ret != ESP_OK) {
       i2c_driver_delete(I2C_MASTER_NUM);
       return 0;
   }

   vTaskDelay((2*CONVERSION_TIME) / portTICK_PERIOD_MS);

   // Read humidity data
   ret = i2c_master_read_from_device(I2C_MASTER_NUM, HDC_1080_SENSOR_ADDR, 
                                   data, 2, 
                                   I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
   if(ret != ESP_OK) {
       i2c_driver_delete(I2C_MASTER_NUM);
       return 0;
   }

   // Clean up I2C driver
   i2c_driver_delete(I2C_MASTER_NUM);

   // Calculate humidity (unchanged)
   humidity = ((data[0] << 8) | data[1]);
   humidity1 = humidity * 100;
   humidity = (humidity1/65536);
   
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
void func1(void)
{
printf("\n msg from hdc component");

}
