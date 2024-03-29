#include "ad5940.h"
#include "ad5941.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void ad5941_init(AD5941_cfg_t ad5941_config)
{
    void AD5940_Main(void *pvParameters);

    
    QueueHandle_t ad5941_queue = ad5941_config.ad5941_queue;
    SemaphoreHandle_t gpio0_semaphore_handle = xSemaphoreCreateBinary();
    ad5941_info_t ad5941_info = {
        .ad5941_queue = ad5941_queue,
        .gpio0_semaphore_handle = gpio0_semaphore_handle,
    };
    
    AD5940_MCUResourceInit((void *)&gpio0_semaphore_handle);
    xTaskCreate(AD5940_Main, "AD5940_Main", 3000, (void *)&ad5941_info,
                5, NULL);
}
