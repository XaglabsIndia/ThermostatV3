#pragma once

#include <stdbool.h>  // For bool type

void init_led_control(void);
void set_led_fade(bool red, bool green, bool blue);
void set_led_solid(bool red, bool green, bool blue);
void set_led_blink(bool red, bool green, bool blue, int blink_interval_ms);
void set_led_blink_alternate(bool red1, bool green1, bool blue1,
                            bool red2, bool green2, bool blue2,
                            int blink_interval_ms);
void stop_leds(void);