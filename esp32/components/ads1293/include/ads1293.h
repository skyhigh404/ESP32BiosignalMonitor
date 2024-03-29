#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct
{
  int leads;                       // ECG lead num
  QueueHandle_t ads1293_queue;     // ECG data storage queue
  unsigned int ads1293_queue_size; // Queue size
  unsigned int ADCmax;             // ADCMAX represents the maximum output code of the ADC. Can be used to calculate differential input voltage
} ADS1293_cfg_t;

typedef struct
{
  uint32_t ecg_data[3];
} ecg_5leads_data_t;

typedef struct
{
  uint32_t ecg_data[2];
} ecg_3leads_data_t;

typedef struct
{
  uint32_t ecg_data[1];
} ecg_1leads_data_t;

/**
 * @brief Initialize ads1293
 */
void ads1293_init(ADS1293_cfg_t ads1293_config);

/**
 * @brief Write a value in the register of ads1293. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * @param addr Register address.
 * @param val Rgister value.
 */
void ads1293_write_reg(const uint8_t addr, const uint8_t val);

/**
 * @brief Read a value in the register of ads1293. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * @param addr Register address.
 * @return uint8_t Rgister value.
 */
uint8_t ads1293_read_reg(const uint8_t addr);

/**
 * @brief Loop read back data from TI_ADS1293_DATA_LOOP_REG
 * 
 * @param count The number source bytes enabled in TI_ADS1293_CH_CNFG_REG
 * @param val Array to store data
 */
void ads1293_stream_read_reg(const uint8_t count, uint8_t val[]);

/**
 * @brief Interrupt service routine, ECG data is ready.
 */
void IRAM_ATTR gpio26_isr_handler(void *arg);
void ads1293_task(void *pvParameters);
