#include "esp_err.h"

void IRAM_ATTR MultiButtonISR(void *arg);
void MultiButton(void *pvParameter);
void PerformAction(uint32_t duration);
void SetupMultiButton();
// Function prototypes without extern
extern void led_control_task(void *pvParameters);
extern void init_led_control();
extern void set_led_fade(bool red, bool green, bool blue);
extern void set_led_solid(bool red, bool green, bool blue);
extern void set_led_blink(bool red, bool green, bool blue, int blink_interval_ms);
extern void set_led_blink_alternate(bool red1, bool green1, bool blue1, 
                             bool red2, bool green2, bool blue2, 
                             int blink_interval_ms);
extern void stop_leds();
extern void HandleOTAMessage(int DevID, int HubID);
extern int StoredDevID, StoredHubID;