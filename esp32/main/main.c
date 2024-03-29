#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "unistd.h"
#include "ads1293.h"
#include "ad5941.h"
#include "wifi.h"
#include "websocketClient.h"
#include "filter.h"
#include "led.h"

struct
{
    ADS1293_cfg_t ads1293_config;
    AD5941_cfg_t ad5941_config;
    filter_cfg_t filter_config;
    wifi_cfg_t wifi_config;
} system_config;

void app_main(void)
{
    printf("Hey you! Idf ver: " IDF_VER " free heap size %u.\n", esp_get_free_heap_size());

    memset(&system_config, 0, sizeof(system_config));

    // Setting ads1293_config and filter_config parameters
    system_config.ads1293_config.leads = 1;
    system_config.ads1293_config.ads1293_queue_size = 200;
    system_config.ads1293_config.ADCmax = 0xC35000;

    system_config.filter_config.filter_queue_size = 200;
    strcpy(system_config.filter_config.wname, "db6");
    system_config.filter_config.level = 7;
    system_config.filter_config.sig_length = 50;
    system_config.filter_config.heart_rate_queue = xQueueCreate(1, sizeof(int));

    if (system_config.ads1293_config.leads == 5)
    {
        system_config.ads1293_config.ads1293_queue =
            xQueueCreate(system_config.ads1293_config.ads1293_queue_size, sizeof(ecg_5leads_data_t));
        system_config.filter_config.filter_queue =
            xQueueCreate(system_config.filter_config.filter_queue_size, sizeof(double) * 3);
    }
    else if (system_config.ads1293_config.leads == 3)
    {
        system_config.ads1293_config.ads1293_queue =
            xQueueCreate(system_config.ads1293_config.ads1293_queue_size, sizeof(ecg_3leads_data_t));
        system_config.filter_config.filter_queue =
            xQueueCreate(system_config.filter_config.filter_queue_size, sizeof(double) * 2);
    }
    else if (system_config.ads1293_config.leads == 1)
    {
        system_config.ads1293_config.ads1293_queue =
            xQueueCreate(system_config.ads1293_config.ads1293_queue_size, sizeof(ecg_1leads_data_t));
        system_config.filter_config.filter_queue =
            xQueueCreate(system_config.filter_config.filter_queue_size, sizeof(double) * 1);
    }

    // Setting ad5941_config parameters
    system_config.ad5941_config.ad5941_queue_size = 20;
    system_config.ad5941_config.ad5941_queue =
        xQueueCreate(system_config.ad5941_config.ad5941_queue_size, sizeof(float));

    // Setting wifi_config parameters
    strcpy(system_config.wifi_config.ssid, "TP-LINK_083E");
    strcpy(system_config.wifi_config.password, "11031103");
    system_config.wifi_config.max_retry = 50;

    // Create queue set
    QueueSetHandle_t send_data_queue_set = xQueueCreateSet(system_config.filter_config.filter_queue_size + system_config.ad5941_config.ad5941_queue_size + 1);
    xQueueAddToSet(system_config.filter_config.filter_queue, send_data_queue_set);
    xQueueAddToSet(system_config.ad5941_config.ad5941_queue, send_data_queue_set);
    xQueueAddToSet(system_config.filter_config.heart_rate_queue, send_data_queue_set);

    // Check everything was created.
    configASSERT(send_data_queue_set);
    configASSERT(system_config.filter_config.filter_queue);
    configASSERT(system_config.ad5941_config.ad5941_queue);
    configASSERT(system_config.filter_config.heart_rate_queue);

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3));

    // Initialize each module
    ad5941_init(system_config.ad5941_config);
    ledc_init();
    ledc_set_blink_mode(ALWAYS_BRIGHT);
    ESP_LOGI("ads1293", "free heap size %u", esp_get_free_heap_size());
    wifi_init(system_config.wifi_config);
    ESP_LOGI("wifi", "free heap size %u", esp_get_free_heap_size());
    web_server_init(system_config.filter_config.filter_queue,
                    system_config.ad5941_config.ad5941_queue,
                    system_config.filter_config.heart_rate_queue,
                    send_data_queue_set,
                    system_config.ads1293_config.leads);
    ESP_LOGI("webserver", "free heap size %u", esp_get_free_heap_size());

    ads1293_init(system_config.ads1293_config); // Changing the initialization order will cause an error
    filter_init(&system_config.filter_config, system_config.ads1293_config);
}
