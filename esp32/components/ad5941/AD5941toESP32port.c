#include "ad5940.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include "esp_log.h"


spi_device_handle_t ad5941_spi_handle;
static SemaphoreHandle_t gpio0_semaphore_handle;
static const char* TAG = "ad5941 port";
/**
    @brief Using SPI to transmit N bytes and return the received bytes. This function targets to
         provide a more efficient way to transmit/receive data.
    @param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
    @param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
    @param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
    @return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.tx_buffer = pSendBuffer;
    t.rx_buffer = pRecvBuff;
    ret = spi_device_polling_transmit(ad5941_spi_handle, &t);
    assert(ret == ESP_OK);
}

// Pull down the cs pin
void AD5940_CsClr(void)
{
    gpio_set_level(GPIO_NUM_5, 0);
}
// Pull up the cs pin
void AD5940_CsSet(void)
{
    gpio_set_level(GPIO_NUM_5, 1);
}
// Pull up the reset pin
void AD5940_RstSet(void)
{
    gpio_set_level(GPIO_NUM_4, 1);
}
// Pull down the reset pin
void AD5940_RstClr(void)
{
    gpio_set_level(GPIO_NUM_4, 0);
}

void AD5940_Delay10us(uint32_t time)
{
    time /= 100;
    if (time == 0)
        time = 1;
    vTaskDelay(time / portTICK_PERIOD_MS);
}

uint32_t AD5940_GetMCUIntFlag(void)
{
    return 0;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
    ESP_LOGI(TAG, "run AD5940_ClrMCUIntFlag");
    // Uncommenting the line below will cause the error which occur in 
    // xSemaphoreTake(ad5941_info.gpio0_semaphore_handle, portMAX_DELAY); which at AD5940_Main()
    xSemaphoreTake(gpio0_semaphore_handle, 0);
    return 1;
}

void IRAM_ATTR gpio17_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(gpio0_semaphore_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Functions that used to initialize MCU platform
 *
 * @param pCfg :spi_device_handle_t*
 *
 * @return uint32_t
 */
uint32_t AD5940_MCUResourceInit(void *pCfg)
{
    // Initial semaphore to synchronize
    gpio0_semaphore_handle = *(SemaphoreHandle_t *)pCfg;

    esp_err_t ret;
    // ad5941 uses VSPI for spi and gpio ? for reset
    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .max_transfer_sz = 64,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};
    // ads1293 uses 8-bit command field (R/WB + 7-bit address)
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,          // SPI mode 0
        .spics_io_num = -1, // CS pin
        .queue_size = 7,
        .clock_speed_hz = SPI_MASTER_FREQ_16M};
    // Initialize the SPI
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &ad5941_spi_handle);
    ESP_ERROR_CHECK(ret);

    // Setting cs pin
    gpio_config_t cs = {
        .pin_bit_mask = 1ull << 5,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    ret = gpio_config(&cs);
    ESP_ERROR_CHECK(ret);

    // Configure external interrupot pin
    gpio_config_t ad5941_intr_gpio = {
        .pin_bit_mask = 1ull << 17,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE // trigger at falling edge
    };
    ESP_ERROR_CHECK(gpio_config(&ad5941_intr_gpio));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_NUM_17, gpio17_isr_handler, NULL));

    // Configure reset pin
    gpio_config_t ad5941_reset_gpio = {
        .pin_bit_mask = 1ull << 4,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0};
    ESP_ERROR_CHECK(gpio_config(&ad5941_reset_gpio));

    AD5940_CsSet();
    AD5940_RstClr();
    vTaskDelay(pdMS_TO_TICKS(10));
    AD5940_RstSet();

    return 0;
}
