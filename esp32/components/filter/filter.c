#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "filter.h"
#include "esp_log.h"
#include "freertos/task.h"

// #define TEST_TIME
const int SAMPLE_RATE = 160; // ECG sample rate
const static char *TAG = "filter_task";

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

typedef struct filter_task_cfg_t
{
  filter_cfg_t filter_config;
  ADS1293_cfg_t ads1293_config;
  wave_object obj;
  wt_object wt;
  double w[6]; // delay line of channel 1, 2, 3
} filter_task_cfg_t;

static double min_a(const double *input, int len)
{
  assert(len > 0);
  double min = input[0];
  for (int i = 1; i < len; i++)
  {
    if (min > input[i])
      min = input[i];
  }
  return min;
}

static double max_a(const double *input, int len)
{
  assert(len > 0);
  double max = input[0];
  for (int i = 1; i < len; i++)
  {
    if (max < input[i])
      max = input[i];
  }
  return max;
}

static double mean_a(const double *input, int len)
{
  double m = 0.0;
  for (int i = 0; i < len; ++i)
  {
    m += input[i];
  }
  m = m / len;
  return m;
}

void filter_init(filter_cfg_t *filter_config_p, ADS1293_cfg_t ads1293_config)
{
  wave_object obj = wave_init(filter_config_p->wname); // Initialize the wavelet
  filter_config_p->inp_length = pow(2, filter_config_p->level) * (obj->filtlength - 1);

  wt_object wt = wt_init(obj, "dwt", filter_config_p->inp_length, filter_config_p->level);
  setDWTExtension(wt, "sym");
  setWTConv(wt, "direct");

  for (int i = 0; i < 3; ++i)
  {
    filter_config_p->inp_array[i] = (double *)calloc(filter_config_p->inp_length, sizeof(double));
    if (filter_config_p->inp_array[i] == NULL)
      ESP_LOGE(TAG, "Failed to allocate memory to filter_config_p->inp_array[%d].", i);
  }

  filter_task_cfg_t filter_task_cfg = {
      .filter_config = *filter_config_p,
      .ads1293_config = ads1293_config,
      .obj = obj,
      .wt = wt,
      .w = {0, 0, 0, 0, 0, 0},
  };

  xTaskCreate(filter_task, "filter_task", 3000, (void *)&filter_task_cfg, 7, NULL);
  ESP_LOGI(TAG, "init compelete, filtlength: %d, inp_length: %d", obj->filtlength, filter_config_p->inp_length);
}

void filter_task(void *pvParameters)
{
  filter_task_cfg_t filter_task_cfg = *(filter_task_cfg_t *)pvParameters;
  double ADCmax = (double)filter_task_cfg.ads1293_config.ADCmax;

  int data_num, ecg_channel = -1;
  uint32_t ecg_data_5leads[3];
  uint32_t ecg_data_3leads[2];
  uint32_t ecg_data_1leads[1];
  double filter_data_5leads[3];
  double filter_data_3leads[2];
  double filter_data_1leads[1];
  int heart_rate;

  if (filter_task_cfg.ads1293_config.leads == 5)
    ecg_channel = 3;
  else if (filter_task_cfg.ads1293_config.leads == 3)
    ecg_channel = 2;
  else if (filter_task_cfg.ads1293_config.leads == 1)
    ecg_channel = 1;

  double *out_array[3];
  for (int i = 0; i < 3; ++i)
  {
    out_array[i] = (double *)calloc(filter_task_cfg.filter_config.inp_length, sizeof(double));
    if (out_array[i] == NULL)
      ESP_LOGE(TAG, "Failed to allocate memory to filter_config_p->inp_array[%d].", i);
  }

  ESP_LOGI(TAG, "free heap size %u", esp_get_free_heap_size());

  while (1)
  {

    // To implement real-time wavelet filtering, pad the sequence with old data
    data_num = 0;
    while (data_num < filter_task_cfg.filter_config.inp_length - filter_task_cfg.filter_config.sig_length)
    {
      if (ecg_channel == 3)
      {
        filter_task_cfg.filter_config.inp_array[0][data_num] = filter_task_cfg.filter_config.inp_array[0][data_num + filter_task_cfg.filter_config.sig_length];
        filter_task_cfg.filter_config.inp_array[1][data_num] = filter_task_cfg.filter_config.inp_array[1][data_num + filter_task_cfg.filter_config.sig_length];
        filter_task_cfg.filter_config.inp_array[2][data_num] = filter_task_cfg.filter_config.inp_array[2][data_num + filter_task_cfg.filter_config.sig_length];
      }
      else if (ecg_channel == 2)
      {
        filter_task_cfg.filter_config.inp_array[0][data_num] = filter_task_cfg.filter_config.inp_array[0][data_num + filter_task_cfg.filter_config.sig_length];
        filter_task_cfg.filter_config.inp_array[1][data_num] = filter_task_cfg.filter_config.inp_array[1][data_num + filter_task_cfg.filter_config.sig_length];
      }
      else if (ecg_channel == 1)
      {
        filter_task_cfg.filter_config.inp_array[0][data_num] = filter_task_cfg.filter_config.inp_array[0][data_num + filter_task_cfg.filter_config.sig_length];
      }
      ++data_num;
    }

    // Load data
    // Convert voltage code to voltage
    // Apply 50Hz notch filter
    // \mathrm{V}_{\mathrm{INP}}-\mathrm{V}_{\mathrm{INM}}=\left(\frac{\mathrm{ADC}_{\mathrm{OUT}}}{\mathrm{ADC}_{\mathrm{MAX}}}-\frac{1}{2}\right) \cdot \frac{2 \mathrm{~V}_{\mathrm{REF}}}{3.5}
    data_num = filter_task_cfg.filter_config.inp_length - filter_task_cfg.filter_config.sig_length;
    while (data_num < filter_task_cfg.filter_config.inp_length)
    {
      if (ecg_channel == 3)
      {
        xQueueReceive(filter_task_cfg.ads1293_config.ads1293_queue, ecg_data_5leads, portMAX_DELAY);
        filter_task_cfg.filter_config.inp_array[0][data_num] = ((double)ecg_data_5leads[0] / (double)ADCmax - 0.5) * 4.8 / 3.5;
        filter_task_cfg.filter_config.inp_array[1][data_num] = ((double)ecg_data_5leads[1] / (double)ADCmax - 0.5) * 4.8 / 3.5;
        filter_task_cfg.filter_config.inp_array[2][data_num] = ((double)ecg_data_5leads[2] / (double)ADCmax - 0.5) * 4.8 / 3.5;
        notch_filter_50Hz(&filter_task_cfg.filter_config.inp_array[0][data_num], filter_task_cfg.w + 0);
        notch_filter_50Hz(&filter_task_cfg.filter_config.inp_array[1][data_num], filter_task_cfg.w + 2);
        notch_filter_50Hz(&filter_task_cfg.filter_config.inp_array[2][data_num], filter_task_cfg.w + 4);
      }
      else if (ecg_channel == 2)
      {
        xQueueReceive(filter_task_cfg.ads1293_config.ads1293_queue, ecg_data_3leads, portMAX_DELAY);
        filter_task_cfg.filter_config.inp_array[0][data_num] = ((double)ecg_data_3leads[0] / (double)ADCmax - 0.5) * 4.8 / 3.5;
        filter_task_cfg.filter_config.inp_array[1][data_num] = ((double)ecg_data_3leads[1] / (double)ADCmax - 0.5) * 4.8 / 3.5;
        notch_filter_50Hz(&filter_task_cfg.filter_config.inp_array[0][data_num], filter_task_cfg.w + 0);
        notch_filter_50Hz(&filter_task_cfg.filter_config.inp_array[1][data_num], filter_task_cfg.w + 2);
      }
      else if (ecg_channel == 1)
      {
        xQueueReceive(filter_task_cfg.ads1293_config.ads1293_queue, ecg_data_1leads, portMAX_DELAY);
        filter_task_cfg.filter_config.inp_array[0][data_num] = ((double)ecg_data_1leads[0] / (double)ADCmax - 0.5) * 4.8 / 3.5;
        notch_filter_50Hz(&filter_task_cfg.filter_config.inp_array[0][data_num], filter_task_cfg.w + 0);
      }
      ++data_num;
    }

#ifdef TEST_TIME
    TickType_t start_time = xTaskGetTickCount();
    TickType_t end_time;
#endif
    // Perform data filtering. Only in channel 1 !!
    
    for (int i = 0; i < ecg_channel; ++i)
    {
      dwt(filter_task_cfg.wt, filter_task_cfg.filter_config.inp_array[i]); // Perform DWT
      // ESP_LOGI(TAG, "dwt task remain stack size: %u; free heap size %u.",
      //        uxTaskGetStackHighWaterMark(NULL), esp_get_free_heap_size());
      // wt_summary(filter_task_cfg.wt);

      filter(filter_task_cfg.wt);             // Filtering

      // ESP_LOGI(TAG, "idwt task remain stack size: %u; free heap size %u.",
      //        uxTaskGetStackHighWaterMark(NULL), esp_get_free_heap_size());
      idwt(filter_task_cfg.wt, out_array[i]); // Perform IDWT
    }
    


#ifdef TEST_TIME
    end_time = xTaskGetTickCount();
    ESP_LOGI(TAG, "wavelet filter excute time: %u.", end_time - start_time);
    start_time = xTaskGetTickCount();
#endif

    // calculate heart rate
    heart_rate = cal_heart_rate(out_array[0], filter_task_cfg.filter_config.inp_length);

#ifdef TEST_TIME
    end_time = xTaskGetTickCount();
    ESP_LOGI(TAG, "calculate heart rate excute time: %u.", end_time - start_time);
#endif

    // Push the processed data into the queue
    xQueueSendToBack(filter_task_cfg.filter_config.heart_rate_queue, &heart_rate, 0);
    data_num = filter_task_cfg.filter_config.inp_length - filter_task_cfg.filter_config.sig_length;
    while (data_num < filter_task_cfg.filter_config.inp_length)
    {
      if (ecg_channel == 3)
      {
        filter_data_5leads[0] = out_array[0][data_num];
        filter_data_5leads[1] = out_array[1][data_num];
        filter_data_5leads[2] = out_array[2][data_num];
        xQueueSendToBack(filter_task_cfg.filter_config.filter_queue, filter_data_5leads, portMAX_DELAY);
      }
      else if (ecg_channel == 2)
      {
        filter_data_3leads[0] = out_array[0][data_num];
        filter_data_3leads[1] = out_array[1][data_num];
        xQueueSendToBack(filter_task_cfg.filter_config.filter_queue, filter_data_3leads, portMAX_DELAY);
      }
      else if (ecg_channel == 1)
      {
        filter_data_1leads[0] = out_array[0][data_num];
        xQueueSendToBack(filter_task_cfg.filter_config.filter_queue, filter_data_1leads, portMAX_DELAY);
      }

      ++data_num;
    }
  }
}

void filter(wt_object wt)
{
  int interval = 320;
  int i, j;
  // Get wavelet transform coefficients start position.
  //
  //  coefficients CA7  CD7  CD6  CD5  CD4  CD3  CD2  CD1
  //     index      0    1    2    3    4    5    6    7
  int *wt_coefficients_start_position = (int *)malloc(sizeof(int) * (wt->J + 1));
  wt_coefficients_start_position[0] = 0;
  for (i = 0; i < wt->J; ++i)
  {
    wt_coefficients_start_position[i + 1] = wt_coefficients_start_position[i] + wt->length[i];
  }

  //  Baseline wander correction. Set CA7 to 0.
  for (i = wt_coefficients_start_position[0]; i < wt->length[0]; ++i)
  {
    wt->output[i] = 0;
  }

  // Using CD1 to calculate the standard deviation of the noise.
  double standard_deviation = calculate_standard_deviation(
      wt->output + wt_coefficients_start_position[7], wt->length[7]);

  // Using universal threshold selection method to select threshold.
  double threshold = standard_deviation;

  //  Apply soft-thresholding method
  for (int cdi = 5; cdi < wt->J; ++cdi)
  {
    for (i = wt_coefficients_start_position[cdi + 1];
         i < wt_coefficients_start_position[cdi + 1] + wt->length[cdi + 1];
         ++i)
    {
      if (wt->output[i] >= 0 && wt->output[i] >= threshold)
        wt->output[i] -= threshold;
      else if (wt->output[i] < 0 && wt->output[i] <= -threshold)
        wt->output[i] += threshold;
      else
        wt->output[i] = 0;
    }
  }

  // A new method
  // for (int cdi = 0; cdi < MIN(wt->J, 2); ++cdi, interval /= 2)
  // {
  //   int cdi_start = wt_coefficients_start_position[cdi + 1];
  //   int cdi_end = wt_coefficients_start_position[cdi + 1] + wt->length[cdi + 1];
  //   for (i = cdi_start; i < cdi_end; i += interval)
  //   {
  //     int interval_end = MIN(i + interval, cdi_end);
  //     double threshold = calculate_standard_deviation(wt->output + i, interval_end);
  //     for (j = i; j < interval_end; ++j)
  //     {
  //       if (wt->output[j] >= 0 && wt->output[j] >= threshold)
  //         wt->output[j] -= threshold;
  //       else if (wt->output[j] < 0 && wt->output[j] <= -threshold)
  //         wt->output[j] += threshold;
  //       else
  //         wt->output[j] = 0;
  //     }
  //   }
  // }
  free(wt_coefficients_start_position);
}

static int cmpfunc(const void *p, const void *q)
{
  double x = *(const double *)p;
  double y = *(const double *)q;
  return (x > y) - (x < y);
}

double calculate_standard_deviation(double *array, int n)
{
  double standard_deviation;
  double *temp = (double *)malloc(sizeof(double) * n);

  for (int i = 0; i < n; ++i)
    temp[i] = fabs(array[i]);

  qsort(temp, n, sizeof(double), cmpfunc);                                          // ascending sort
  standard_deviation = (n % 2) ? temp[n / 2] : (temp[n / 2] + temp[n / 2 - 1]) / 2; // find the median
  standard_deviation = standard_deviation / 0.6745;                                 // estimated the standard deviation of noise

  free(temp);
  return standard_deviation;
}

void notch_filter_50Hz(double *data, double *w)
{
  // Parameters at 160Hz sampling rate.
  // a0 = 1
  // coef[5] = { b0, b1, b2, a1, a2}
  static double coef[5] = {0.972708434191025, 0.744478804573387, 0.972708434191025, 0.744478804573387, 0.945416868382050};

  double d0 = *data - coef[3] * w[0] - coef[4] * w[1];
  *data = coef[0] * d0 + coef[1] * w[0] + coef[2] * w[1];
  w[1] = w[0];
  w[0] = d0;
}

int cal_heart_rate(const double *signal, int siglen)
{
  static int r_peaks_pos[50]; // R peak position in channel 1 (inp_array[0])
  static int r_peaks_len;     // valid r_peaks_pos length
  float heart_rate;

  // Calculate the heart rate
  detect_R_peaks(signal, siglen, r_peaks_pos, &r_peaks_len, 9);
  if (r_peaks_len > 1)
    heart_rate = 60.0 * (r_peaks_len - 1) * SAMPLE_RATE / (r_peaks_pos[r_peaks_len - 1] - r_peaks_pos[0]);
  else
    heart_rate = 0;
  // ESP_LOGI(TAG, "r_peaks_len: %d, heart rate: %f, peaks pos: %d , %d", r_peaks_len, heart_rate, r_peaks_pos[r_peaks_len - 1], r_peaks_pos[0]);
  return (int)(heart_rate + 0.5);
}

void detect_R_peaks(const double *signal, int siglen, int *posout, int *outlen, int r_duration_max)
{
  static int window_size = 0.5 * SAMPLE_RATE;                            // search window size is 0.5s
  static int step_size = 0.1 * SAMPLE_RATE;                              // step size is 0.1s
  static int valid_window[2] = {0.15 * SAMPLE_RATE, 0.35 * SAMPLE_RATE}; // R peak positon should be within valid_window, 0.15s-0.35s
  static double scale = 0.5;                                             // Peak detection threshold shrinkage coefficient

  double threshold;
  double min_peak_prominence;
  *outlen = 0;
  posout[0] = -1;

  int i, j, k;
  double max;
  // move searching window [i, i + window_size)
  for (i = 0; i + window_size < siglen; i += step_size)
  {
    max = max_a(signal + i, window_size);
    threshold = max * scale;   // Use thresholds to reduce computation
    min_peak_prominence = max; // minimum peak prominence.
    // searching
    for (j = i + 1; j < i + window_size - 1; ++j)
    {
      // is peak, within valid_window, significant and does not exist before
      if ((signal[j] > threshold && signal[j] > signal[j - 1] && signal[j] > signal[j + 1] &&
           j >= i + valid_window[0] && j < i + valid_window[1]) &&
          *outlen < 50 && (*outlen == 0 || posout[*outlen - 1] != j) &&
          cal_prominence(signal + i, window_size, j - i) > min_peak_prominence)
      {
        // calculate the width of the peak above the threshold to determine whether it is R peak
        // searching left
        for (k = j - 1; k >= i; --k)
        {
          if (signal[k] < threshold)
            break;
        }

        int tbegin = k;
        // searching right
        for (k = j + 1; k < i + window_size; ++k)
        {
          if (signal[k] < threshold)
            break;
        }
        int tend = k;

        if (tend - tbegin - 1 < r_duration_max)
        {
          posout[*outlen] = j;
          *outlen += 1;
        }
      }
    }
  }
}

double cal_prominence(const double *input, int len, int peak_pos)
{
  int i;
  // searching left
  for (i = peak_pos - 1; i >= 0; --i)
  {
    if (input[i] > input[peak_pos])
      break;
  }
  double left_prominence = input[peak_pos] - min_a(input + i, peak_pos - i + 1);

  // searching right
  for (i = peak_pos + 1; i < len; ++i)
  {
    if (input[i] > input[peak_pos])
      break;
  }
  double right_prominence = input[peak_pos] - min_a(input + i, i - peak_pos + 1);
  // return min value
  return left_prominence > right_prominence ? right_prominence : left_prominence;
}
