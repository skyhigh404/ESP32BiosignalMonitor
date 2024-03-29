#include <stdio.h>
#include "led.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (32) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

TaskHandle_t led_blink_task_handle;

void ledc_init()
{
    led_blink_task_handle = NULL;
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void set_led_off()
{
    // Set duty to 100%. ((2 ** 13) - 1) = 8191
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 8191));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void set_led_on()
{
    // Set duty to 50%. ((2 ** 13) - 1) * 70% = 5734
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 5734));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void set_led_blink(void *pvParameters)
{
    while (1)
    {
        set_led_on();
        vTaskDelay(pdMS_TO_TICKS(1000));
        set_led_off();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ledc_set_blink_mode(led_mode_t led_mode)
{
    if(led_blink_task_handle){
        vTaskDelete(led_blink_task_handle);
        led_blink_task_handle = NULL;
    }

    if(led_mode==OFF){
        set_led_off();
    }
    else if(led_mode==ALWAYS_BRIGHT){
        set_led_on();
    }
    else if(led_mode==BLINK){
        xTaskCreate(set_led_blink, "led_blink_task", 1000, NULL, 3, &led_blink_task_handle);
    }
}

