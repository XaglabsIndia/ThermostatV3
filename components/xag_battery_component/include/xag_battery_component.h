#ifndef XAG_BATTERY_COMPONENT_H
#define XAG_BATTERY_COMPONENT_H
#include "esp_err.h"  

char* int_to_string_for_lora(int value, int max_length);
int calculate_battery_percentage(float voltage);
float read_battery_voltage(void);
esp_err_t init_adc(void);
#endif