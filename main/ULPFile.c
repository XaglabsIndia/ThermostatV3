/* ULP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "sdkconfig.h"
#include "ulp_main.h"
#include "math.h"
#include "esp_log.h"
#include "lora.h"
#include "Config.h"


char* lora_message ="NEMS";
#define ulp_halt_time 1 //(seconds) 1 means 1 second
#define ulp_deep_sleep_wakeup_time 1 //(minutes) ex:2 means 2 minutes
#define MY_THERMO_ID 203
#define MY_HUB_ID 100
#define DELIMITER ','

// This is the ponter which stores the address of the incoming LoRa message
uint32_t *address_tx;
// This is the ponter which stores the address of the incoming LoRa message
uint32_t *address;
// This is the ponter which stores the address of the incoming hdc1080 sensor temperature buffer
uint32_t *address_t = &ulp_HDC1080_buffer;
// This is the ponter which stores the address of the incoming hdc1080 sensor humidity buffer
uint32_t *address_h = &ulp_HDC1080_humidbuffer;

// This is ulp thing 
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
uint32_t ulp_one_time=1;

// change the ulp pins as needed
#if CONFIG_IDF_TARGET_ESP32
const gpio_num_t GPIO_CS = GPIO_NUM_15;
const gpio_num_t GPIO_MOSI = GPIO_NUM_13;
const gpio_num_t GPIO_SCLK = GPIO_NUM_14;
const gpio_num_t GPIO_MISO = GPIO_NUM_12;
const gpio_num_t GPIO_RESET = GPIO_NUM_26;
const gpio_num_t GPIO_SCL = GPIO_NUM_32;
const gpio_num_t GPIO_SDA = GPIO_NUM_33;
#endif
// change the ulp pins as needed
#if CONFIG_IDF_TARGET_ESP32S3
const gpio_num_t GPIO_CS = GPIO_NUM_4;
const gpio_num_t GPIO_MOSI = GPIO_NUM_5;
const gpio_num_t GPIO_SCLK = GPIO_NUM_6;
const gpio_num_t GPIO_MISO = GPIO_NUM_7;
const gpio_num_t GPIO_RESET = GPIO_NUM_8;
const gpio_num_t GPIO_SCL = GPIO_NUM_9;
const gpio_num_t GPIO_SDA = GPIO_NUM_10;
#endif

// This is used to send the lora message into rtc memory
//message format "NEMS,"
void writingtoulp(const char*message,uint8_t devid,uint8_t hubid,char delimiter)
{
    char devid_str[8];
    snprintf(devid_str, sizeof(devid_str), "%d", devid);
    char hubid_str[8];
    snprintf(hubid_str, sizeof(hubid_str), "%d", hubid);
    // Concatenate strings
    char result[64];

    snprintf(result, sizeof(result), "%s%c%c%s%c%c%s%c", message,delimiter, '0',devid_str,delimiter,'0',hubid_str,delimiter);
    uint8_t i = 0;
    address_tx = &ulp_tx_fifo;
    for (; (*(result + i) != '\0'); i++)
    {
        address_tx[i] = (*(result + i)*256);
    }

}

// This is used to initilaise the ulp program
static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    rtc_gpio_init(GPIO_CS);
    rtc_gpio_set_direction(GPIO_CS, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_CS, 1);

    rtc_gpio_init(GPIO_MOSI);
    rtc_gpio_set_direction(GPIO_MOSI, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_MOSI, 0);

    rtc_gpio_init(GPIO_SCLK);
    rtc_gpio_set_direction(GPIO_SCLK, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_SCLK, 0);

    rtc_gpio_init(GPIO_MISO);
    rtc_gpio_set_direction(GPIO_MISO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_level(GPIO_MISO, 0);

    rtc_gpio_init(GPIO_RESET);
    rtc_gpio_set_direction(GPIO_RESET, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pullup_en(GPIO_RESET);
   
    rtc_gpio_init(GPIO_SCL);//THESE RTC GPIOS ARE USED TO INITILASIS GPIO(SCL,SDA) AS INPUT PINS.
    rtc_gpio_set_direction(GPIO_SCL, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(GPIO_SDA);
    rtc_gpio_set_direction(GPIO_SDA, RTC_GPIO_MODE_INPUT_ONLY);
    
    /* Set ULP wake up period to 2s */
    ulp_tx_retry_cond_1_counter=0;
    ulp_tx_counter=0;
    ulp_rx_count=0;
    ulp_sensor_count_value=0;
    ulp_set_wakeup_period(0,(ulp_halt_time*1000000));
    writingtoulp("NEMS",MY_THERMO_ID,MY_HUB_ID,DELIMITER);
}

// printing the LoRa message from RTC_memory 
void printing_lora_message_from_ulp(int len)
{
    address = &ulp_tx_counter;
    printf("ulp lora txd data :  \n");
     printf("\n");
    for (uint8_t i = 1; i < len; i++)
    {
        printf("%c", (uint16_t) (* (address + i)/256));
    }
    printf("\n");
   address = &ulp_tx_fifo;
    printf("ulp lora txd data :  \n");
     printf("\n");
    for (uint8_t i = 0; i < len; i++)
    {
        printf("%c", (uint16_t) (* (address + i)));
    }
    printf("\n");
        address = &ulp_tx_fifo;
    printf("ulp lora received data :  \n");
 }



void deep_sleep_status()
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
   
   
    uint32_t *address1 = &ulp_HDC1080_dummy1;
    if ((cause != ESP_SLEEP_WAKEUP_ULP)&&(cause != ESP_SLEEP_WAKEUP_TIMER))
    {
        printf("Not ULP wakeup, initializing ULP\n"); 
        init_ulp_program();  
    }
    else if(cause == ESP_SLEEP_WAKEUP_TIMER)
    {
        printf("Deep sleep TIMER wakeup\n");
        printf("received payload value is:     %d\n", (uint8_t)ulp_pay_lend);
        printf("\ntemperature value of HDC1080 sensor : %d C humidity value of HDC1080 sensor : %d RH ", (((uint8_t)ulp_HDC1080_dummy)/2), (uint8_t) * (address1));
        uint8_t value = ulp_pay_lend;
        printing_lora_message_from_ulp(value);
        init_ulp_program();
    }
    else
    {
        printf("Deep sleep ULP  wakeup\n");
        printf("received payload value is:     %d\n", (uint8_t)ulp_pay_lend);
        printf("\ntemperature value of HDC1080 sensor : %d C humidity value of HDC1080 sensor : %d RH ", (((uint8_t)ulp_HDC1080_dummy)/2), (uint8_t) * (address1));
        uint8_t value = ulp_pay_lend;
        printing_lora_message_from_ulp(value);
        init_ulp_program();
    }
}

void ulp_component(int ulp_timer_wakeup_time_in_min)
{
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_CS));
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_MOSI));
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_SCLK));
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_MISO));
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_NUM_26));
    deep_sleep_status();

    ESP_LOGE("ulp_rx", "\nEntering deep sleep\n");
    ESP_ERROR_CHECK(ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)));
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    if (ulp_timer_wakeup_time_in_min > 0)
    {
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(ulp_timer_wakeup_time_in_min * 60000000));
    }
    esp_deep_sleep_start();
}
