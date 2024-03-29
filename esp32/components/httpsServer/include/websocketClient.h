#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void web_server_init(QueueHandle_t ecg_queue, QueueHandle_t ad5941_queue, QueueHandle_t heart_rate_queue,
                     QueueSetHandle_t send_data_queue_set, int ads1293_leads);