#pragma once
#include <stdint.h>

typedef struct
{
    char ssid[32];
    char password[64];
    uint8_t max_retry;
    uint8_t channel;
    uint8_t max_connection;
} wifi_cfg_t;

void wifi_init(wifi_cfg_t wifi_config);