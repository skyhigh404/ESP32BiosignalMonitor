#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ads1293.h"
#include "wavelib.h"

typedef struct
{
  QueueHandle_t filter_queue;     // Filter data storage queue
  unsigned int filter_queue_size; // Queue size
  QueueHandle_t heart_rate_queue; // Heart Rate data storage queue
  char wname[50];                 // Wavelet name
  int level;                      // Decomposition Levels
  int inp_length;                 // Length of Signal/Time Series
  int sig_length;                 // New signal length to process in inp_array
  double *inp_array[3];           // Maxium 3 channel ECG data
} filter_cfg_t;

/**
 * @brief Init filter for ECG filtering.
 * 
 * @param filter_config_p: pointer of filter config.
 * @param ads1293_config: ads1293 config.
 */
void filter_init(filter_cfg_t *filter_config_p, ADS1293_cfg_t ads1293_config);

/**
 * @brief Excute filter task.
 * 
 * @param pvParameters: filter_task_cfg_t
 */
void filter_task(void *pvParameters);

/**
 * @brief Denoise ECG data in wavelet decomposition level 7 sampling rate 160Hz condition.
 * 
 * @param wt_object: wt_object
 */
void filter(wt_object wt);

double calculate_standard_deviation(double *temp, int n);

/**
 * @brief A realtime notch filter.
 * 
 * @param data: double; input data, the filter result will modifies it.
 * @param w: double array[2]; delay line w0,w1. Length of 2.
 */
void notch_filter_50Hz(double *data, double *w);

/**
 * @brief Calculate heart rate
 * 
 * @param signal: input signal 
 * @param siglen: signal length 
 * @return int: heart rate
 */
int cal_heart_rate(const double *signal, int siglen);

/**
 * @brief Detect R peaks of ECG signal. ECG sample rate is 160Hz.
 * 
 * @param signal: intput signal.
 * @param siglen: int put signal length.
 * @param posout: the output array of R peaks positon.
 * @param outlen: valid posout length.
 * @param r_duration_max: hyperparameters. R peak duration should less than this number.
 */
void detect_R_peaks(const double *signal, int siglen, int *posout, int *outlen, int r_duration_max);

/**
 * @brief According to the calculation method of matlab.
 * 
 * Prominence
 * 
 * The prominence of a peak measures how much the peak stands out due to its intrinsic height and its 
 * location relative to other peaks. A low isolated peak can be more prominent than one that is higher
 * but is an otherwise unremarkable member of a tall range.
 * 
 * To measure the prominence of a peak:
 *  1. Place a marker on the peak.
 *  2. Extend a horizontal line from the peak to the left and right until the line does one of the following:
 *     Crosses the signal because there is a higher peak
 *     Reaches the left or right end of the signal
 *  3. Find the minimum of the signal in each of the two intervals defined in Step 2. This point is either 
 *     a valley or one of the signal endpoints.
 *  4. The higher of the two interval minima specifies the reference level. The height of the peak above 
 *     this level is its prominence.
 * 
 * findpeaks makes no assumption about the behavior of the signal beyond its endpoints, whatever their height. 
 * This is reflected in Steps 2 and 4 and often affects the value of the reference level. Consider for example 
 * the peaks of this signal:
 * 
 * @param input: input signal.
 * @param len: signal length.
 * @param peak_pos: the peak position to calculate prominence.
 * @return double: the result of prominence.
 */
double cal_prominence(const double *input, int len, int peak_pos);
