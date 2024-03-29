#pragma once

typedef enum led_mode_t{
    OFF,
    BLINK,
    ALWAYS_BRIGHT
} led_mode_t;

void ledc_init();
void ledc_set_blink_mode(led_mode_t led_mode);
