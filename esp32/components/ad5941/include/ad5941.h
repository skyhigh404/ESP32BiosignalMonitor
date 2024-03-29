#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

typedef struct AD5941_cfg_t
{
  QueueHandle_t ad5941_queue;     // EDA data storage queue
  unsigned int ad5941_queue_size; // Queue size
} AD5941_cfg_t;

typedef struct ad5941_info_t
{
  QueueHandle_t ad5941_queue;
  SemaphoreHandle_t gpio0_semaphore_handle;
} ad5941_info_t;

void ad5941_init(AD5941_cfg_t ad5941_config);