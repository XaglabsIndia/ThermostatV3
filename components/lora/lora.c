/**
 * @file lora.c
 * @author Laštůvka Lukáš
 *
 * @version 0.1
 * @date 2023-02-28
 *
 * @copyright
 * MIT License
 *
 * Copyright (c) 2023 Laštůvka Lukáš
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @ref https://github.com/LastuvkaLukas/espidf-lora
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/gpio.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <time.h>//time
#include "lora.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "esp_system.h"
#include "esp_err.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include <freertos/semphr.h>
#include <ctype.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#ifdef CONFIG_LORA_HSPI_ON
#define LORA_SPI 1
#elif CONFIG_LORA_VSPI_ON
#define LORA_SPI 1
#else
#error "No selected LORA_SPI"
#define LORA_SPI
#endif

#define LORA_SYNC_WORD 0x54
static const char TAG[] = "lora";


static volatile int VarCADDone = 0;
static volatile int VarCADDetected = 0;
static volatile int VarRXDone = 0;
static volatile int VarRXDoneSingle = 0;
static SemaphoreHandle_t spi_mutex = NULL;

uint8_t len;
uint8_t new_buf[256];
// Intrupt Variable
// static volatile VarCADDone = 0;
// static volatile VarCADDetected = 0;
// static volatile VarRXDone = 0;
// static volatile VarRXDoneSingle = 0;


int DIO0Pin = 0;
int DIO1Pin = 0;
/**
 * Initializes the GPIO device.
 */

void AttachInterrupt(int gpio,void* isr_handler,int interrupt_type)
{
   //initialize the config structure.
   gpio_config_t io_conf;
   //enable or disable interrupt
   io_conf.intr_type = interrupt_type;
   ESP_LOGI("AttachInterrupt", "GPIO %d",gpio);
   //bit mask of the pins that you want to set
   io_conf.pin_bit_mask = (1ULL << gpio) ;
   //set as output mode
   io_conf.mode = GPIO_MODE_INPUT;
   io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
   io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
   //configure GPIO with the given settings
   gpio_config(&io_conf);
}
/**
 * Remove ISR handller.
 */

void DeattachInterrupt(int gpio)
{
   //remove isr handler for gpio number
   gpio_isr_handler_remove(gpio);
}

/**
 * Initializes the SPI device.
 */
static spi_device_handle_t __spi;

/**
 * Clears the IRQ flags.
 */
static void lora_clear_irq(void)
{
   lora_write_reg(LORA_REG_IRQ_FLAGS, 0xff);
}

/**
 * Sets the LoRa mode.
 *
 * @param mode The LoRa mode.
 */
void lora_set_mode(lora_reg_op_mode_t mode)
{
    ESP_LOGV(TAG, "Set mode num: %i", mode);
    
    // Read current mode
    uint8_t current_mode = lora_read_reg(LORA_REG_OP_MODE) & ~LORA_OP_MODE_LONG_RANGE_MODE;
    
    // If changing modes, go through STANDBY
    if (current_mode != mode) {
        // First go to standby if not already there
        if (mode != LORA_OP_MODE_STDBY) {
            lora_write_reg(LORA_REG_OP_MODE, LORA_OP_MODE_LONG_RANGE_MODE | LORA_OP_MODE_STDBY);
            vTaskDelay(pdMS_TO_TICKS(10));  // Allow time to settle
        }
        
        // Then switch to desired mode
        lora_write_reg(LORA_REG_OP_MODE, LORA_OP_MODE_LONG_RANGE_MODE | mode);
        vTaskDelay(pdMS_TO_TICKS(10));  // Allow time to settle
    }

    lora_clear_irq();  // Clear any pending interrupts
}

/**
 * Resets the LoRa module.
 */
static void lora_reset(void)
{
   ESP_LOGI(TAG, "Reset module %d", CONFIG_LORA_RST_GPIO);
   gpio_set_level(CONFIG_LORA_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_LORA_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Sets the output power of the LoRa transceiver.
 *
 * @param level The output power level.
 */
static void lora_set_tx_power(uint8_t level)
{
   ESP_LOGD(TAG, "Set TX power level: %i", level);
   if (level < 2)
      level = 2;
   else if (level > 17)
      level = 17;
   lora_write_reg(LORA_REG_PA_CONFIG, LORA_PA_CONFIG_PA_SELECT_PA_BOOST | (level - 2));
}

/**
 * Sets the frequency of the LoRa transceiver.
 *
 * @param frequency The frequency in Hz.
 */
static void lora_set_frequency(uint64_t frequency)
{
   ESP_LOGI(TAG, "Set frequency to: %lli", frequency);
   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(LORA_REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(LORA_REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(LORA_REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Sets the preamble length for LoRa.
 *
 * @param len The length of the preamble in symbols.
 */
static void lora_set_preamble_length(uint16_t len)
{
   ESP_LOGI(TAG, "Set preamble length: %i", len);
   lora_write_reg(LORA_REG_PREAMBLE_MSB, (uint8_t)(len >> 8));
   lora_write_reg(LORA_REG_PREAMBLE_LSB, (uint8_t)(len >> 0));
}

/**
 * Sets the LoRa sync word.
 *
 * @param word The sync word to set.
 */
static void lora_set_sync_word(uint8_t word)
{
   ESP_LOGI(TAG, "Set Network ID: %#02x", word);
   lora_write_reg(LORA_REG_SYNC_WORD, word);
}

/**
 * Enables the CRC check on received packets.
 */
static void lora_enable_crc(void)
{
   ESP_LOGD(TAG, "Enable CRC in payload");
   lora_write_reg(LORA_REG_MODEM_CONFIG_2, lora_read_reg(LORA_REG_MODEM_CONFIG_2) | LORA_MODEM_CONFIG2_RX_PAYLOAD_CRC_ON);
}

/**
 * Writes a value to a LoRa register.
 *
 * @param reg The register to write.
 * @param val The value to write to the register.
 */


void lora_write_reg(lora_reg_t reg, uint8_t val)
{
    if (__spi == NULL) {
        ESP_LOGE(TAG, "SPI device not initialized");
        return;
    }

    if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for SPI write");
        return;
    }

    esp_err_t ret = ESP_OK;
    
    // Prepare transaction
    uint8_t tx[2] = {0x80 | reg, val};
    uint8_t rx[2] = {0};
    
    spi_transaction_t t = {
        .flags = 0,
        .length = 16,        // Total length is 16 bits
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    // Assert CS before transaction
    #ifdef CONFIG_LORA_CS_ON_GPIO
    gpio_set_level(CONFIG_LORA_CS_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));  // Brief delay after CS assertion
    #endif

    // Perform SPI transaction
    ret = spi_device_transmit(__spi, &t);

    // Deassert CS after transaction
    #ifdef CONFIG_LORA_CS_ON_GPIO
    vTaskDelay(pdMS_TO_TICKS(1));  // Brief delay before CS deassert
    gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
    #endif

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
    }

    xSemaphoreGive(spi_mutex);
}

// Mutex-protected SPI read with proper CS handling
uint8_t lora_read_reg(lora_reg_t reg)
{
    if (__spi == NULL) {
        ESP_LOGE(TAG, "SPI device not initialized");
        return 0xFF;
    }

    uint8_t val = 0xFF;  // Default error value

    if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for SPI read");
        return val;
    }

    // Prepare transaction
    uint8_t tx[2] = {reg & 0x7F, 0xFF};  // Clear MSB for read operation
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .flags = 0,
        .length = 16,        // Total length is 16 bits
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    // Assert CS before transaction
    #ifdef CONFIG_LORA_CS_ON_GPIO
    gpio_set_level(CONFIG_LORA_CS_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));  // Brief delay after CS assertion
    #endif

    // Perform SPI transaction
    esp_err_t ret = spi_device_transmit(__spi, &t);

    // Deassert CS after transaction
    #ifdef CONFIG_LORA_CS_ON_GPIO
    vTaskDelay(pdMS_TO_TICKS(1));  // Brief delay before CS deassert
    gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
    #endif

    if (ret == ESP_OK) {
        val = rx[1];  // Second byte contains the read data
    } else {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
    }

    xSemaphoreGive(spi_mutex);
    return val;
}
/**
 * Initializes the LoRa module.
 *
 * @returns ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t lora_init(void)
{
   rtc_gpio_hold_dis(CONFIG_LORA_CS_GPIO);
   rtc_gpio_hold_dis(CONFIG_LORA_RST_GPIO);
   rtc_gpio_hold_dis(CONFIG_LORA_MISO_GPIO); 
   rtc_gpio_hold_dis(CONFIG_LORA_MOSI_GPIO);
   rtc_gpio_hold_dis(CONFIG_LORA_SCK_GPIO);

   // Set as regular GPIO pins for SPI
   gpio_reset_pin(CONFIG_LORA_CS_GPIO);
   gpio_reset_pin(CONFIG_LORA_RST_GPIO);
   gpio_reset_pin(CONFIG_LORA_MISO_GPIO);
   gpio_reset_pin(CONFIG_LORA_MOSI_GPIO);
   gpio_reset_pin(CONFIG_LORA_SCK_GPIO);
   ESP_LOGI(TAG, "Initialize LoRa module");
   esp_err_t err = ESP_OK;
   gpio_pad_select_gpio(CONFIG_LORA_RST_GPIO);
   gpio_set_direction(CONFIG_LORA_RST_GPIO, GPIO_MODE_OUTPUT);
#ifdef CONFIG_LORA_CS_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_CS_GPIO);
   gpio_set_direction(CONFIG_LORA_CS_GPIO, GPIO_MODE_OUTPUT);
#endif
spi_mutex = xSemaphoreCreateMutex();
   if (spi_mutex == NULL) {
       return ESP_ERR_NO_MEM;
   }
   ESP_LOGI("CONFIG_LORA_MISO_GPIO", "CONFIG_LORA_MISO_GPIO %d",CONFIG_LORA_MISO_GPIO);
   ESP_LOGI("CONFIG_LORA_MOSI_GPIO", "CONFIG_LORA_MOSI_GPIO %d",CONFIG_LORA_MOSI_GPIO);
   ESP_LOGI("CONFIG_LORA_SCK_GPIO", "CONFIG_LORA_SCK_GPIO %d",CONFIG_LORA_SCK_GPIO);
   ESP_LOGI("CONFIG_LORA_SCK_GPIO", "CONFIG_LORA_CS_GPIO %d",CONFIG_LORA_CS_GPIO);

   spi_bus_config_t bus = {
       .miso_io_num = CONFIG_LORA_MISO_GPIO,
       .mosi_io_num = CONFIG_LORA_MOSI_GPIO,
       .sclk_io_num = CONFIG_LORA_SCK_GPIO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = SPICOMMON_BUSFLAG_MASTER};
   ESP_LOGI("LORA_SPI", "LORA_SPI %d",LORA_SPI);
   err = spi_bus_initialize(LORA_SPI, &bus, 0);
   if (err != ESP_OK)
      return err;
   spi_device_interface_config_t dev = {
        .clock_speed_hz = 5000000,    // 5MHz - stable speed for LoRa
        .mode = 0,                    // SPI mode 0
        .spics_io_num = -1,          // CS pin (managed manually)
        .queue_size = 1,             // We want to handle transactions ourselves
        .flags = SPI_DEVICE_NO_DUMMY,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 3,        // CS setup time
        .cs_ena_posttrans = 3,       // CS hold time
        .pre_cb = NULL
    };
   err = spi_bus_add_device(LORA_SPI, &dev, &__spi);
   if (err != ESP_OK)
      return err;
   ESP_LOGI("gpio_pad_select_gpio", "287");
   lora_reset();
   ESP_LOGI("gpio_pad_select_gpio", "289");
   lora_set_mode(LORA_OP_MODE_SLEEP);

   uint8_t i = 0;
   while (i++ < CONFIG_LORA_INIT_TIMEOUT)
   {  //lora_write_reg(LORA_REG_VERSION,0x12);
      uint8_t version = lora_read_reg(LORA_REG_VERSION);
      ESP_LOGI("version", "version %d",version);
      if (version == 0x12)
         break;
      vTaskDelay(2);
   }

   if (i >= CONFIG_LORA_INIT_TIMEOUT + 1)
      return ESP_ERR_TIMEOUT;
   ESP_LOGI("gpio_pad_select_gpio", "303");
   lora_write_reg(LORA_REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(LORA_REG_FIFO_TX_BASE_ADDR, 0);
   lora_write_reg(LORA_REG_LNA, lora_read_reg(LORA_REG_LNA) | LORA_LNA_BOOST_HF);
   lora_write_reg(LORA_REG_MODEM_CONFIG_3, LORA_MODEM_CONFIG3_AGC_AUTO_ON);
   lora_set_tx_power(CONFIG_LORA_TX_POWER);
   lora_set_frequency(CONFIG_LORA_FREQ);
   lora_set_preamble_length(CONFIG_LORA_PREAMBLE_LEN);

   lora_set_sync_word(LORA_SYNC_WORD);
   lora_enable_crc();
   lora_set_mode(LORA_OP_MODE_STDBY);
   gpio_install_isr_service(0); 
   return err;
}


/**
 * Sets the LoRa module to sleep mode.
 */
void lora_sleep(void)
{
   ESP_LOGI(TAG, "Go sleep LoRa module");
   lora_set_mode(LORA_OP_MODE_SLEEP);
}

/**
 * Returns the RSSI of the last received packet.
 *
 * @returns The RSSI of the last received packet.
 */
int16_t lora_packet_rssi(void)
{
   return (lora_read_reg(LORA_REG_PKT_RSSI_VALUE) - (CONFIG_LORA_FREQ < 868E6 ? 164 : 157));
}

/**
 * Returns the SNR of the last received packet.
 *
 * @returns The SNR of the last received packet.
 */
float lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(LORA_REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Sets the LoRa module to transmit mode.
 */
void lora_begin_tx(void)
{
   lora_set_mode(LORA_OP_MODE_STDBY);
   lora_write_reg(LORA_REG_FIFO_ADDR_PTR, 0);
}

/**
 * Writes data to the LoRa TX FIFO.
 *
 * @param buf The buffer containing the data to be written.
 * @param size The size of the buffer.
 *
 * @returns ESP_OK if successful ESP_ERR_INVALID_SIZE if the size is invalid.
 */
esp_err_t lora_write_tx(uint8_t *buf, uint8_t size)
{
   if (size == 0 || (uint16_t)((lora_read_reg(LORA_REG_FIFO_ADDR_PTR) + size)) > 0xff)
      return ESP_ERR_INVALID_SIZE;

   for (int i = 0; i < size; i++)
      lora_write_reg(LORA_REG_FIFO, *buf++);

   return ESP_OK;
}

/**
 * Ends a transmission.
 */
esp_err_t lora_end_tx(void)
{
   uint8_t len = lora_read_reg(LORA_REG_FIFO_ADDR_PTR);
   if (len == 0)
      return ESP_ERR_INVALID_SIZE;

   lora_write_reg(LORA_REG_PAYLOAD_LENGTH, len);

   lora_set_mode(LORA_OP_MODE_TX);
   uint8_t read;
   do
   {
      read = lora_read_reg(LORA_REG_IRQ_FLAGS);
      vTaskDelay(2);
   } while ((read & LORA_IRQ_FLAGS_TX_DONE) == 0);

   ESP_LOGI(TAG, "Transmit payload with length: %i", len);
   //lora_write_reg(LORA_REG_IRQ_FLAGS, LORA_IRQ_FLAGS_TX_DONE);
   //vtaskdelay(10);
   lora_write_reg(LORA_REG_IRQ_FLAGS, 0x00);
   return ESP_OK;
}


/**
 * Test a CAD to be detected.
 */
bool lora_is_cad_detected(void)
{  
   
   lora_set_mode(LORA_OP_MODE_CAD);

   uint8_t read;

   for (;;)
   {
      read = lora_read_reg(LORA_REG_IRQ_FLAGS);
      ESP_LOGI(TAG, "read %d ",read);
      if (read & LORA_IRQ_FLAGS_CAD_DONE)
      {
         if (read & LORA_IRQ_FLAGS_CAD_DETECTED)
         {
            
            ESP_LOGD(TAG, "CAD detected");
            
            ESP_LOGI(TAG, "LORA_IRQ_FLAGS_CAD_DETECTED %d ",LORA_IRQ_FLAGS_CAD_DETECTED);
            ESP_LOGI(TAG, "LORA_IRQ_FLAGS_CAD_DONE %d ",LORA_IRQ_FLAGS_CAD_DONE);
            return true;
         }
         return false;
      }
   }
}

/**
 * Starts the LoRa module in receive mode after CAD detection.
 *
 * @returns ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t lora_waiting_cad_rx(uint8_t *len)
{
   lora_waiting_cad();
   return 0;
}


#ifdef CONFIG_CONFIG_LORA_DIO_ON_GPIO

/**
 * Sets the DIO mode for a DIO pin.
 *
 * @param num The DIO pin number.
 * @param mode The DIO mode.
 * @param val The value to set the DIO pin to.
 */
void lora_dio_write(lora_reg_dio_num_t num, lora_reg_dio_mode_t mode, uint8_t val)
{
   lora_reg_t dio_reg = ((num == LORA_DIO_NUM_4) || (num == LORA_DIO_NUM_5)) ? LORA_REG_DIO_MAPPING_2 : LORA_REG_DIO_MAPPING_1;
   uint8_t mask = 0x00;

   switch (num)
   {
   case LORA_DIO_NUM_0:
   case LORA_DIO_NUM_4:
      mask = (val << 0x06);
      break;

   case LORA_DIO_NUM_1:
   case LORA_DIO_NUM_5:
      mask = (val << 0x04);
      break;

   case LORA_DIO_NUM_2:
      mask = (val << 0x02);
      break;

   case LORA_DIO_NUM_3:
      mask = val;
      break;
   }

   if (mode == LORA_DIO_SET)
   {
      lora_write_reg(dio_reg, lora_read_reg(dio_reg) | mask);
      return;
   }
   lora_write_reg(dio_reg, lora_read_reg(dio_reg) & (~mask));
}

#endif

/**
@berief This function is usend to send data with cad detection The Function set the operation LORA_REG_OP_MODE registed by performing bitwise opereation with LORA_OP_MODE_LONG_RANGE_MODE and The Lora CAD mode the Once CAD mode is activated
The function check for cad activity by continous polling the IRQ registers. If CAD mode is detetced the function wait for ceratin time and retry same process for 5 time. Once the channel is free the function transmit the data
**/
void ActivateCADMode(void)
{
    lora_set_mode(LORA_OP_MODE_CAD);
}

/**
 * Starts the LoRa module in receive mode.
 *
 * @returns ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t lora_begin_rx(uint8_t *len, const uint8_t *valid_lengths, size_t num_lengths)
{
   *len = 0;
   uint8_t read;
   if (lora_read_reg(LORA_REG_IRQ_FLAGS) & LORA_IRQ_FLAGS_PAYLOAD_CRC_ERROR)
      return ESP_ERR_INVALID_CRC;
   *len = lora_read_reg(LORA_REG_RX_NB_BYTES);
   int RSSIValue = lora_packet_rssi();              
   float SNRValue = lora_packet_snr();
  for (size_t i = 0; i < num_lengths; ++i) {
        if (*len == valid_lengths[i]) {
            ESP_LOGI(TAG, "Received payload with length: %i and RSSI of %ddBm and SNR of %fdB", *len, RSSIValue, SNRValue);
            lora_write_reg(LORA_REG_FIFO_ADDR_PTR, lora_read_reg(LORA_REG_FIFO_RX_CURRENT_ADDR));
            lora_write_reg(LORA_REG_IRQ_FLAGS, 0x00);
            return ESP_OK;
        }
    }

    // If no valid length matched
    lora_set_mode(LORA_OP_MODE_SLEEP);
    return ESP_FAIL;
}


/**
 * Reads data from the LoRa module's RX FIFO.
 *
 * @param buf A pointer to the buffer to store the data.
 * @param size The number of bytes to read.
 *
 * @returns ESP_OK if the read was successful, ESP_ERR_INVALID_SIZE if the
 *          size is invalid, or ESP_FAIL if the read failed.
 */
esp_err_t lora_read_rx(uint8_t *buf, uint8_t size)
{
   if (size == 0 || (uint16_t)((lora_read_reg(LORA_REG_FIFO_ADDR_PTR) + size)) > 0xff)
      return ESP_ERR_INVALID_SIZE;
   for (int i = 0; i < size; i++)
      *buf++ = lora_read_reg(LORA_REG_FIFO);
   lora_write_reg(LORA_REG_IRQ_FLAGS, 0x00);
   lora_set_mode(LORA_OP_MODE_SLEEP);
   return ESP_OK;
}

/**
 * End of income.
 */
void lora_end_rx(void)
{
   lora_set_mode(LORA_OP_MODE_STDBY);
}
/**
 * Change Lora mode  to continious mode.
 */
void LoraContiniousMode(){
   lora_set_mode(LORA_OP_MODE_RX_CONTINUOUS);
}
       

/**
 * Lora Continious mode API Takes input as Time
 * **/

// void IRAM_ATTR RXDone(void* arg) {
//   VarRXDone = 1;
//   gpio_isr_handler_remove(CONFIG_LORA_DIO0_GPIO);
// }

void IRAM_ATTR RXDoneSingle(void* arg) {
  VarRXDoneSingle = 1;
  gpio_isr_handler_remove(CONFIG_LORA_DIO0_GPIO);
}

/**
 * @brief Receive LoRa messages in continuous mode for a specified time interval.
 *
 * This function configures the LoRa module to receive messages in continuous mode
 * for a given time interval. It calculates the timeout time based on the provided
 * time interval and enters a loop to receive messages and push it into the queue until the timeout occurs.
 *
 * @param TimeIntervalRX The time interval in minutes for which the LoRa module
 *                       should remain in continuous receive mode.
 * @param Length The expected length of the received payload. If the received
 *               payload length differs from this value, it will be discarded.
 *                
 *
 * @note This function assumes the availability of the following functions:
 *       - lora_write_reg
 *       - AttachInterrupt
 *       - gpio_isr_handler_add
 *       - LoraContiniousMode
 *       - lora_begin_rx
 *       - lora_read_rx
 *       - lora_end_rx
 * IF Timeout is given as value 0 for TimeIntervalRX then it is an infinite loop. 
 */
// void RecivedLoraContiniousMode(int TimeIntervalRX, uint8_t Length, QueueHandle_t lora_queue) {
//     time_t CurrentTime, Timeout;
//     struct tm* local_time;
//     uint8_t buf[Length+1];
//     uint8_t len;
//     uint8_t new_buf[256];
//     CurrentTime = time(NULL);
//     local_time = localtime(&CurrentTime);
//    //  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
//    //  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
 
//     // Set timeout
//     Timeout = CurrentTime + (TimeIntervalRX);
//     printf("Current time: %s", asctime(local_time));
 
//     // Configure the DIO0 pin for RX done interrupt
//     lora_write_reg(LORA_REG_DIO_MAPPING_1, 0x00);
//     ESP_LOGI("CONFIG_LORA_CS_GPIO", "CONFIG_LORA_DIO0_GPIO %d", CONFIG_LORA_DIO0_GPIO);
//     ESP_LOGI("LORA_REG_DIO_MAPPING", "LORA_REG_DIO_MAPPING %d", LORA_REG_DIO_MAPPING_1);
//     AttachInterrupt(CONFIG_LORA_DIO0_GPIO, RXDone, GPIO_INTR_POSEDGE);
//     gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, RXDone, NULL);
 
//     ESP_LOGI("CONFIG_LORA_CS_GPIO", "CONFIG_LORA_DIO0_GPIO %d", CONFIG_LORA_DIO0_GPIO);
 
//     // Set LoRa mode to continuous mode
//     LoraContiniousMode();
 
//     while (1) {
//        esp_task_wdt_reset();
//         CurrentTime = time(NULL);
//         if (VarRXDone == 1) {
//             esp_err_t err = lora_begin_rx(&len, Length);
//           //  esp_task_wdt_reset();
//             if (err == ESP_OK) {
//                 lora_read_rx(buf, len);
//             //     esp_task_wdt_reset_user(*WTRXContiniousHdl);
//                // vTaskDelay(10);
//                 memset(new_buf, 0, 256); // Clear the buffer
//                 memcpy(new_buf, buf, len);
//                 // Add the received message to the queue
//                 if (xQueueSend(lora_queue, (void *)new_buf, (TickType_t)10) != pdPASS) {
//                     printf("Failed to send to queue\n");
//                 }
//             }
//             VarRXDone = 0;
//             gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, RXDone, NULL);
//             LoraContiniousMode();
//         }
//         if ((CurrentTime > Timeout) && (Timeout != 0)) {
//             lora_end_rx();
//             break;
//         }
//     }
// }

esp_err_t RecivedLoraContiniousModeInit(){
   lora_set_mode(LORA_OP_MODE_STDBY);
   vTaskDelay(10);
  lora_write_reg(LORA_REG_DIO_MAPPING_1, 0x00);
  ESP_LOGI("CONFIG_LORA_CS_GPIO", "CONFIG_LORA_DIO0_GPIO %d", CONFIG_LORA_DIO0_GPIO);
  ESP_LOGI("LORA_REG_DIO_MAPPING", "LORA_REG_DIO_MAPPING %d", LORA_REG_DIO_MAPPING_1);
  AttachInterrupt(CONFIG_LORA_DIO0_GPIO, RXDone, GPIO_INTR_POSEDGE);
  gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, RXDone, NULL);
 
  ESP_LOGI("CONFIG_LORA_CS_GPIO", "CONFIG_LORA_DIO0_GPIO %d", CONFIG_LORA_DIO0_GPIO);
  LoraContiniousMode();
  return ESP_OK;
}

esp_err_t RecivedLoraContiniousMode(int TimeIntervalRX, const uint8_t *LoraContiRXLength, size_t num_lengths, QueueHandle_t lora_queue) {
   uint8_t buf[256];
   esp_err_t err = lora_begin_rx(&len, LoraContiRXLength, num_lengths);
   if (err == ESP_OK) 
   {
      lora_read_rx(buf, len);
      memset(new_buf, 0, 256); // Clear the buffer
      memcpy(new_buf, buf, len);
      // Add the received message to the queue
      if (xQueueSend(lora_queue, (void *)new_buf, (TickType_t)10) != pdPASS) {
         printf("Failed to send to queue\n");
      }
}
      gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, RXDone, NULL);
      LoraContiniousMode();
      return err;
}

/**
 * @brief Calculate the symbol count based on the provided time interval.
 *
 * This function calculates the symbol count required for the LoRa module to stay
 * in RX mode for a given time interval. The calculation is based on the LoRa
 * bandwidth and spreading factor settings.
 *
 * @param TimeIntervalms The time interval in milliseconds for which the symbol
 *                       count needs to be calculated.
 *
 * @return The calculated symbol count as an unsigned integer.
 *
 * @note This function assumes the LoRa bandwidth is set to 125 kHz and the
 *       spreading factor is set to 7. If your LoRa settings differ, you'll
 *       need to modify the corresponding constants in the function.
 */

unsigned int CalculateSymbolCount(int TimeIntervalms) {
    // LoRa settings
    const int Bandwidth = 125000; 
    const int SpreadingFactor = 7; 
    // Calculate symbol rate
    double SymbolRate = (double)Bandwidth / pow(2.0, SpreadingFactor);
    // Calculate symbol count based on time interval
    double TimeInterval = (double)TimeIntervalms / 1000.0; // Convert ms to seconds
    unsigned int SymbolCount = (unsigned int)(TimeInterval * SymbolRate);
    return SymbolCount;
}

/**
 * @brief Set the RX timer for the LoRa module based on the symbol count.
 *
 * This function sets the RX timer for the LoRa module by writing the symbol count
 * to the LORA_REG_MODEM_CONFIG_2 and LORA_REG_MODEM_CONFIG_2 + 1 registers.
 * The symbol count is split into two parts, with the most significant 2 bits
 * stored in LORA_REG_MODEM_CONFIG_2 and the least significant 8 bits stored
 * in LORA_REG_MODEM_CONFIG_2 + 1.
 *
 * @param SymbolCount The symbol count value to be set for the RX timer.
 *                    If the value is greater than 1020, it is capped at 1020.
 */
 void SetRXTimmer(uint32_t SymbolCount)
{
   uint8_t RegistedLSB, RegisterMSB;
   if (SymbolCount > 1020){
      SymbolCount = 1020;
   }
   RegistedLSB = lora_read_reg(LORA_REG_MODEM_CONFIG_2); // Read the current value
   RegistedLSB &= 0xFC; // Clear the bits where symbol count will be set
   RegistedLSB |= (SymbolCount >> 8); // Set the new symbol count
   RegisterMSB = (SymbolCount & 0xFF); // Get the lower 8 bits of the symbol count
   lora_write_reg(LORA_REG_MODEM_CONFIG_2, RegistedLSB); // Write the first part of the symbol count
   lora_write_reg(LORA_REG_MODEM_CONFIG_2 + 1, RegisterMSB); // Write the second part of the symbol count
}

/**
 * @brief Receive a LoRa message in single RX mode with a specified time interval.
 *
 * This function configures the LoRa module to receive a message in single RX mode. It calculates
 * the symbol count based on the provided time interval and sets the RX timer accordingly. The
 * function then enters the RX mode and waits for the RX done interrupt or a timeout.
 *
 * @param TimeInterval The time interval in milliseconds for which the LoRa module should remain
 *                     in RX mode .
 *                      Note: Max time to be set between 100 ms to 1.2S as Timeout
 * @param Length The expected length of the received payload. If the received payload length
 *               differs from this value, a warning is logged.
 *
 * @return
 *    - ESP_OK: Successful reception of the LoRa message.
 *    - ESP_ERR_TIMEOUT: Timeout occurred while waiting for the RX done interrupt.
 *    - ESP_ERR_INVALID_CRC: The received payload has an invalid CRC.
 */
esp_err_t LoraSingleModeRX(int TimeInterval, uint8_t Length, char* Data)
{
    esp_err_t result = ESP_FAIL;
    uint8_t len = 0;
    uint8_t read;
    uint8_t buf[255];
    TickType_t start_time;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(TimeInterval + 1000); // Add 1 second margin
    
    memset(buf, 0, 255);               
    
    // Reset state
    VarRXDoneSingle = 0;
    lora_write_reg(LORA_REG_IRQ_FLAGS, 0xFF);  // Clear all IRQ flags
    
    // Setup for reception
    lora_set_mode(LORA_OP_MODE_SLEEP);
    int SymbolCount = CalculateSymbolCount(TimeInterval);
    SetRXTimmer(SymbolCount);
    
    // Setup DIO0 interrupt for RX Done
    lora_write_reg(LORA_REG_DIO_MAPPING_1, 0x00);
    gpio_intr_disable(CONFIG_LORA_DIO0_GPIO);
    gpio_isr_handler_remove(CONFIG_LORA_DIO0_GPIO);
    gpio_set_intr_type(CONFIG_LORA_DIO0_GPIO, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, RXDoneSingle, NULL);
    gpio_intr_enable(CONFIG_LORA_DIO0_GPIO);
    
    // Enter RX Single mode
    lora_set_mode(LORA_OP_MODE_RX_SINGLE);
    start_time = xTaskGetTickCount();
    
    // Wait for reception with timeout
    while(VarRXDoneSingle == 0)
    {
        // Check for timeout
        if((xTaskGetTickCount() - start_time) > timeout_ticks) {
            ESP_LOGW(TAG, "Reception timeout after %d ms", TimeInterval);
            result = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
        
        // Check IRQ flags
        read = lora_read_reg(LORA_REG_IRQ_FLAGS);
        if (read & LORA_IRQ_FLAGS_RX_TIMEOUT)
        {
            ESP_LOGW(TAG, "LoRa RX timeout flag set");
            result = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
        
        // Prevent watchdog trigger
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Check for CRC error
    if (lora_read_reg(LORA_REG_IRQ_FLAGS) & LORA_IRQ_FLAGS_PAYLOAD_CRC_ERROR) {
        ESP_LOGW(TAG, "CRC error detected");
        result = ESP_ERR_INVALID_CRC;
        goto cleanup;
    }
    
    // Read received data
    len = lora_read_reg(LORA_REG_RX_NB_BYTES);
    if (Length != len) {
        ESP_LOGW(TAG, "Invalid message length. Expected: %d, Got: %d", Length, len);
        result = ESP_ERR_INVALID_SIZE;
        goto cleanup;
    }
    
    // Read the payload
    lora_write_reg(LORA_REG_FIFO_ADDR_PTR, lora_read_reg(LORA_REG_FIFO_RX_CURRENT_ADDR));
    lora_read_rx(buf, len);
    memset(Data, 0, 256);
    memcpy(Data, buf, len);
    
    ESP_LOGI(TAG, "Successfully received %d bytes", len);
    result = ESP_OK;

cleanup:
    // Common cleanup code
    gpio_intr_disable(CONFIG_LORA_DIO0_GPIO);
    gpio_isr_handler_remove(CONFIG_LORA_DIO0_GPIO);
    VarRXDoneSingle = 0;
    lora_write_reg(LORA_REG_IRQ_FLAGS, 0xFF);
    lora_end_rx();
    return result;
}
// Intrupt Handler for Send Data function
void IRAM_ATTR CADDone(void* arg) {
  VarCADDone = 1;
  gpio_isr_handler_remove(CONFIG_LORA_DIO0_GPIO);
}
void IRAM_ATTR CADDetected(void* arg) {
  VarCADDetected = 1;
 // gpio_isr_handler_remove(CONFIG_LORA_DIO1_GPIO);
}

/**
 * @brief Send a LoRa message using the CAD (Channel Activity Detection) mechanism.
 *
 * This function sends a LoRa message using the Channel Activity Detection (CAD) mechanism to
 * ensure that the channel is free before transmitting. It first configures the DIO0 and DIO1
 * pins for CAD done and CAD detected interrupts, respectively. Then, it prepares the data
 * packet, initiates the LoRa transmission, and waits for the CAD done or CAD detected interrupts.
 * If the channel is free, it completes the transmission; otherwise, it retries up to 5 times
 * before giving up.
 *
 * @param Data The null-terminated string containing the data to be transmitted.
 *
 * @return
 *    - ESP_OK: Transmission successful.
 *    - ESP_ERR_INVALID_SIZE: The input data is too long for the buffer.
 */
esp_err_t  SendMessageWithCAD(char* Data)
{
   lora_set_mode(LORA_OP_MODE_STDBY);
   vTaskDelay(10);
   //Local VAriables
  uint8_t buf[255];
  uint8_t len;
  int Retries = 0; 
  ESP_LOGI("Lora TX", "CONFIG_LORA_DIO0_GPIO pin is %d",CONFIG_LORA_DIO0_GPIO);
  //ESP_LOGI("Lora TX", "CONFIG_LORA_DIO1_GPIO pin is %d",CONFIG_LORA_DIO1_GPIO);
  // Mapping of DIO0 and DIO1 as CAD done and detetcted as per SEMtect document
  lora_write_reg(LORA_REG_DIO_MAPPING_1, 0xA0);
  // Attch the intrupt to DIO0 and DIO1 pin 
  AttachInterrupt(CONFIG_LORA_DIO0_GPIO, CADDone, GPIO_INTR_POSEDGE);
  gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, CADDone, NULL);
  //AttachInterrupt(CONFIG_LORA_DIO1_GPIO, CADDetected, GPIO_INTR_POSEDGE);
  //gpio_isr_handler_add(CONFIG_LORA_DIO1_GPIO, CADDetected, NULL);
 // Lora Begain will se the operation mode for transmission
  lora_begin_tx();
  //Preparation of data packet
  size_t DataLen = strlen(Data);
  if (DataLen >= sizeof(buf))
    {
        ESP_LOGE("Lora TX", "Data too long");
        esp_err_t  err1 = ESP_ERR_INVALID_SIZE;
        return err1;
    }

  strncpy((char *)buf, Data, sizeof(buf)); 
  size_t size = strlen((char *)buf);
  

 esp_err_t err = lora_write_tx(buf,  size);
  if (err != ESP_OK)
    {
        ESP_LOGE("Lora TX", "Failed to send LoRa message: %s", esp_err_to_name(err));
        return err;
    }
  ActivateCADMode();
  while(1)
  {
    if(VarCADDone == 1)
    {
      ESP_LOGI("Lora TX","CADDone detected %d",VarCADDone);
      VarCADDone = 0;
      if (VarCADDetected == 1)
      {
        ESP_LOGI("Lora TX","CAD Detected intrupt raised");
        VarCADDetected = 0;
        Retries++;
        if (Retries > 5){
        return ESP_FAIL;
        }
      }
            
      else
      {
        ESP_LOGI("Lora TX","Channel Is Free");
        lora_end_tx();
        ESP_LOGI("Lora TX","transmission completed");
        return ESP_OK;
      }
      gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, CADDone, NULL);
 //     gpio_isr_handler_add(CONFIG_LORA_DIO1_GPIO, CADDetected, NULL);
    }
  ActivateCADMode();
  }
  return ESP_OK;
}


void LoraDeepSleepInit(void){
   lora_set_mode(LORA_OP_MODE_STDBY);
   vTaskDelay(100);
   lora_set_mode(LORA_OP_MODE_SLEEP);
   vTaskDelay(100);
   rtc_gpio_init(CONFIG_LORA_CS_GPIO);
   rtc_gpio_set_direction(CONFIG_LORA_CS_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_pulldown_en(CONFIG_LORA_CS_GPIO);
   rtc_gpio_init(CONFIG_LORA_RST_GPIO);
   rtc_gpio_set_direction(CONFIG_LORA_RST_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_pulldown_en(CONFIG_LORA_RST_GPIO);
   rtc_gpio_init(CONFIG_LORA_MISO_GPIO);
   rtc_gpio_set_direction(CONFIG_LORA_MISO_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_pulldown_en(CONFIG_LORA_MISO_GPIO);
   rtc_gpio_init(CONFIG_LORA_MOSI_GPIO);
   rtc_gpio_set_direction(CONFIG_LORA_MOSI_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_pulldown_en(CONFIG_LORA_MOSI_GPIO);
   rtc_gpio_init(CONFIG_LORA_SCK_GPIO);
   rtc_gpio_set_direction(CONFIG_LORA_SCK_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_pulldown_en(CONFIG_LORA_SCK_GPIO);
   rtc_gpio_hold_en(CONFIG_LORA_CS_GPIO);  
   rtc_gpio_hold_en(CONFIG_LORA_RST_GPIO);  
   rtc_gpio_hold_en(CONFIG_LORA_MISO_GPIO);  
   rtc_gpio_hold_en(CONFIG_LORA_MOSI_GPIO);  
   rtc_gpio_hold_en(CONFIG_LORA_SCK_GPIO);  
}