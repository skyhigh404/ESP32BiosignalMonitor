#include "ads1293.h"
#include "TI_ADS1293.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include "esp_log.h"

spi_device_handle_t ads1293_spi_handle;
static int lead_num;
static SemaphoreHandle_t DRDYB_semaphore_handle;
static QueueHandle_t ads1293_queue = NULL;
static const char *TAG = "ads1293";


/**
 *          Pin connect
 *
 *      ESP32       ADS1293
 *      IO12          MISO
 *      IO13          MOSI
 *      IO14          SCLK
 *      IO15          CS
 *      IO26          DRDYB
 */
void ads1293_init(ADS1293_cfg_t ads1293_config)
{
  lead_num = ads1293_config.leads;
  esp_err_t ret;
  /* Step1, initialize SPI peripheral*/
  spi_bus_config_t buscfg = {
      .mosi_io_num = 13,
      .miso_io_num = 12,
      .sclk_io_num = 14,
      .max_transfer_sz = 32,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1};
  // ads1293 uses 8-bit command field (R/WB + 7-bit address)
  spi_device_interface_config_t devcfg = {
      .command_bits = 1,
      .address_bits = 7,
      .mode = 0,          // SPI mode 0
      .spics_io_num = 15, // CS pin
      .queue_size = 7,
      .clock_speed_hz = SPI_MASTER_FREQ_20M // Clock out at 8 MHz
  };
  // Initialize the SPI
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &ads1293_spi_handle);
  ESP_ERROR_CHECK(ret);

  /* Step2, initialize ads1293 */
  // Works in 5 leads mode
  if (lead_num == 5)
  {
    //    DESIGN PARAMETER      EXAMPLE VALUE
    //  Number of electrodes         5
    //  Lead I definition         LA – RA
    //  Lead II definition        LL –RA
    //  Lead V definition         V1 – WCT
    //  Bandwidth                 32 Hz
    //  Output data rate          160 sps

    ads1293_write_reg(TI_ADS1293_CONFIG_REG, 0x00);      // Stops data conversion.
    ads1293_write_reg(TI_ADS1293_FLEX_CH1_CN_REG, 0x11); // Connects channel 1's INP to IN2 and INN to IN1.
    ads1293_write_reg(TI_ADS1293_FLEX_CH2_CN_REG, 0x19); // Connect channel 2's INP to IN3 and INN to IN1.
    ads1293_write_reg(TI_ADS1293_FLEX_CH3_CN_REG, 0x2E); // Connects channel 3's INP to IN5 and INN to IN6.
    ads1293_write_reg(TI_ADS1293_CMDET_EN_REG, 0x07);    // Enables the common-mode detector on input pins IN1, IN2 and IN3.
    ads1293_write_reg(TI_ADS1293_RLD_CN_REG, 0x04);      // Connects the output of the RLD amplifier internally to pin IN4.
    ads1293_write_reg(TI_ADS1293_WILSON_EN1_REG, 0x01);  // Connects the first buffer of the Wilson reference to the IN1 pin.
    ads1293_write_reg(TI_ADS1293_WILSON_EN2_REG, 0x02);  // Connects the second buffer to the IN2 pin.
    ads1293_write_reg(TI_ADS1293_WILSON_EN3_REG, 0x03);  // Connects the third buffer to the IN3 pin.
    ads1293_write_reg(TI_ADS1293_WILSON_CN_REG, 0x01);   // Connects the output of the Wilson reference internally to IN6.
    ads1293_write_reg(TI_ADS1293_OSC_CN_REG, 0x04);      // Uses external crystal and feeds the output of the internal oscillator module to the digital.
    ads1293_write_reg(TI_ADS1293_R2_RATE_REG, 0x02);     // Configures the R2 decimation rate as 5 for all channels.
    ads1293_write_reg(TI_ADS1293_R3_RATE1_REG, 0x20);    // Configures the R3 decimation rate as 32 for channel 1.
    ads1293_write_reg(TI_ADS1293_R3_RATE2_REG, 0x20);    // Configures the R3 decimation rate as 32 for channel 2.
    ads1293_write_reg(TI_ADS1293_R3_RATE3_REG, 0x20);    // Configures the R3 decimation rate as 32 for channel 3.
    ads1293_write_reg(TI_ADS1293_DRDYB_SRC_REG, 0x08);   // Configures the DRDYB source to ECG channel 1 (or fastest channel).
    ads1293_write_reg(TI_ADS1293_CH_CNFG_REG, 0x70);     // Enables ECG channel 1, ECG channel 2, and ECG channel 3 for loop read-back mode.
    ads1293_write_reg(TI_ADS1293_AFE_RES_REG, 0x03);     // High-resolution mode for Channel 1, 2 and 3 instrumentation amplifier
  }
  // Works in 3 leads mode
  else if (lead_num == 3)
  {
    //    DESIGN PARAMETER      EXAMPLE VALUE
    //  Number of electrodes         4
    //  Lead I definition         LA – RA
    //  Lead II definition        LL –RA
    //  Bandwidth                 32 Hz
    //  Output data rate          160 sps
    ads1293_write_reg(TI_ADS1293_CONFIG_REG, 0x00);      // Stops data conversion.
    ads1293_write_reg(TI_ADS1293_FLEX_CH1_CN_REG, 0x11); // Connects channel 1's INP to IN2 and INN to IN1.
    ads1293_write_reg(TI_ADS1293_FLEX_CH2_CN_REG, 0x19); // Connect channel 2's INP to IN3 and INN to IN1.
    ads1293_write_reg(TI_ADS1293_CMDET_EN_REG, 0x07);    // Enables the common-mode detector on input pins IN1, IN2 and IN3.
    ads1293_write_reg(TI_ADS1293_RLD_CN_REG, 0x04);      // Connects the output of the RLD amplifier internally to pin IN4.
    ads1293_write_reg(TI_ADS1293_OSC_CN_REG, 0x04);      // Uses external crystal and feeds the output of the internal oscillator module to the digital.
    ads1293_write_reg(TI_ADS1293_AFE_SHDN_CN_REG, 0x24); // Shuts down unused channel 3's signal path.
    ads1293_write_reg(TI_ADS1293_R2_RATE_REG, 0x02);     // Configures the R2 decimation rate as 5 for all channels.
    ads1293_write_reg(TI_ADS1293_R3_RATE1_REG, 0x20);    // Configures the R3 decimation rate as 32 for channel 1.
    ads1293_write_reg(TI_ADS1293_R3_RATE2_REG, 0x20);    // Configures the R3 decimation rate as 32 for channel 2.
    ads1293_write_reg(TI_ADS1293_DRDYB_SRC_REG, 0x08);   // Configures the DRDYB source to ECG channel 1 (or fastest channel).
    ads1293_write_reg(TI_ADS1293_CH_CNFG_REG, 0x30);     // Enables channel 1 ECG and channel 2 ECG for loop read-back mode.
    ads1293_write_reg(TI_ADS1293_AFE_RES_REG, 0x03);     // High-resolution mode for Channel 1 and 2 instrumentation amplifier
  }
  // Works in 1 leads mode
  else if (lead_num == 1)
  {
    ads1293_write_reg(TI_ADS1293_CONFIG_REG, 0x00);      // Stops data conversion.
    ads1293_write_reg(TI_ADS1293_FLEX_CH1_CN_REG, 0x11); // Connects channel 1's INP to IN2 and INN to IN1.
    ads1293_write_reg(TI_ADS1293_CMDET_EN_REG, 0x03);    // Enables the common-mode detector on input pins IN1 and IN2.
    ads1293_write_reg(TI_ADS1293_RLD_CN_REG, 0x03);      // Connects the output of the RLD amplifier internally to pin IN3.
    ads1293_write_reg(TI_ADS1293_OSC_CN_REG, 0x04);      // Uses external crystal and feeds the output of the internal oscillator module to the digital.
    ads1293_write_reg(TI_ADS1293_AFE_SHDN_CN_REG, 0x36); // Shuts down unused channel 2's and channel3's signal path.
    ads1293_write_reg(TI_ADS1293_R2_RATE_REG, 0x02);     // Configures the R2 decimation rate as 5 for all channels.
    ads1293_write_reg(TI_ADS1293_R3_RATE1_REG, 0x20);    // Configures the R3 decimation rate as 32 for channel 1.
    ads1293_write_reg(TI_ADS1293_DRDYB_SRC_REG, 0x08);   // Configures the DRDYB source to ECG channel 1 (or fastest channel).
    ads1293_write_reg(TI_ADS1293_CH_CNFG_REG, 0x10);     // Enables channel 1 ECG for loop read-back mode.
    ads1293_write_reg(TI_ADS1293_AFE_RES_REG, 0x03);     // High-resolution mode for Channel 1 instrumentation amplifier
  }

  /* Step3, initialize GPIO26 interrupt */
  gpio_config_t ads1293_intr_gpio = {
      .pin_bit_mask = 1ull << 26,
      .mode = GPIO_MODE_INPUT,
      .pull_down_en = 0,
      .pull_up_en = 1,
      .intr_type = GPIO_INTR_NEGEDGE // trigger at falling edge
  };

  ret = gpio_config(&ads1293_intr_gpio);
  ESP_ERROR_CHECK(ret);
  ret = gpio_isr_handler_add(GPIO_NUM_26, gpio26_isr_handler, NULL);
  ESP_ERROR_CHECK(ret);

  /* Step4, creat task to read and send data*/
  DRDYB_semaphore_handle = xSemaphoreCreateBinary();
  assert(DRDYB_semaphore_handle != NULL);
  ads1293_queue = ads1293_config.ads1293_queue;

  xTaskCreate(ads1293_task, "ads1293_task", 2048, NULL, 10, NULL);

  // Start working
  ads1293_write_reg(TI_ADS1293_CONFIG_REG, 0x01); // Starts data conversion.
  ESP_LOGI(TAG, "Init compelete.");
}

void ads1293_write_reg(const uint8_t addr, const uint8_t val)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.cmd = 0x00; // Write command
  t.addr = addr;
  t.length = 8;
  t.tx_buffer = &val;
  ret = spi_device_polling_transmit(ads1293_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
}

uint8_t ads1293_read_reg(const uint8_t addr)
{
  uint8_t val;
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.cmd = 0x01; // Read command
  t.addr = addr;
  t.length = 8;
  t.rx_buffer = &val;
  ret = spi_device_polling_transmit(ads1293_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
  return val;
}

void ads1293_stream_read_reg(const uint8_t count, uint8_t val[])
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.cmd = 0x01; // Read command
  t.addr = TI_ADS1293_DATA_LOOP_REG;
  t.length = count * 8;
  t.rx_buffer = val;
  ret = spi_device_polling_transmit(ads1293_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
}

void IRAM_ATTR gpio26_isr_handler(void *arg)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(DRDYB_semaphore_handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ads1293_task(void *pvParameters)
{
  // ESP_LOGI(TAG, "remain stack size when task start : %u", uxTaskGetStackHighWaterMark(NULL));

  ecg_5leads_data_t out_5leads;
  ecg_3leads_data_t out_3leads;
  ecg_1leads_data_t out_1leads;
  uint8_t data[9];
  for (;;)
  {
    // TickType_t start_time = xTaskGetTickCount();
    xSemaphoreTake(DRDYB_semaphore_handle, portMAX_DELAY);

    // Works in 5 leads mode
    if (lead_num == 5)
    {
      memset(&out_5leads, 0, sizeof(ecg_5leads_data_t));
      ads1293_stream_read_reg(9, data);
      out_5leads.ecg_data[0] = (data[0] << 16) + (data[1] << 8) + data[2];
      out_5leads.ecg_data[1] = (data[3] << 16) + (data[4] << 8) + data[5];
      out_5leads.ecg_data[2] = (data[6] << 16) + (data[7] << 8) + data[8];
      xQueueSendToBack(ads1293_queue, &out_5leads, portMAX_DELAY);
    }
    // Works in 3 leads mode
    else if (lead_num == 3)
    {
      memset(&out_3leads, 0, sizeof(ecg_3leads_data_t));
      ads1293_stream_read_reg(6, data);
      out_3leads.ecg_data[0] = (data[0] << 16) + (data[1] << 8) + data[2];
      out_3leads.ecg_data[1] = (data[3] << 16) + (data[4] << 8) + data[5];
      xQueueSendToBack(ads1293_queue, &out_3leads, portMAX_DELAY);
    }
    // Works in 1 leads mode
    else if (lead_num == 1)
    {
      memset(&out_1leads, 0, sizeof(ecg_1leads_data_t));
      ads1293_stream_read_reg(3, data);
      out_1leads.ecg_data[0] = (data[0] << 16) + (data[1] << 8) + data[2];
      xQueueSendToBack(ads1293_queue, &out_1leads, portMAX_DELAY);
    }
    // TickType_t end_time = xTaskGetTickCount();
    // ESP_LOGI(TAG, "excute time: %u remain stack size: %u.", end_time - start_time, uxTaskGetStackHighWaterMark(NULL));
  }
}
