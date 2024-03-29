#include "websocketClient.h"
#include "string.h"
#include "esp_event.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_websocket_client.h"
#include "led.h"

#define NO_DATA_TIMEOUT_SEC 5
const char *WEBSOCKET_URI = "ws://192.168.2.106:5000/ws/lowerComputer";

static const char *TAG = "WEBSOCKET";

static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;
esp_websocket_client_handle_t client;

typedef struct send_data_cfg_t
{
  QueueHandle_t ecg_queue;
  QueueHandle_t ad5941_queue;
  QueueHandle_t heart_rate_queue;
  QueueSetHandle_t send_data_queue_set;
  int ads1293_leads;
  unsigned int ecg_id;
  unsigned int eda_id;
  char sECG_id[11];
  char sEDA_id[11];
} send_data_cfg_t;

static void shutdown_signaler(TimerHandle_t xTimer)
{
  ESP_LOGI(TAG, "No data received for %d seconds, signaling shutdown", NO_DATA_TIMEOUT_SEC);
  xSemaphoreGive(shutdown_sema);
}

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
  switch (event_id)
  {
  case WEBSOCKET_EVENT_CONNECTED:
    ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
    ledc_set_blink_mode(BLINK);
    break;
  case WEBSOCKET_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
    ledc_set_blink_mode(OFF);
    break;
  case WEBSOCKET_EVENT_DATA:
    ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
    ESP_LOGI(TAG, "Received opcode=%d", data->op_code);
    if (data->op_code == 0x08 && data->data_len == 2)
    {
      ESP_LOGW(TAG, "Received closed message with code=%d", 256 * data->data_ptr[0] + data->data_ptr[1]);
      ledc_set_blink_mode(OFF);
    }
    else
    {
      ESP_LOGW(TAG, "Received=%.*s", data->data_len, (char *)data->data_ptr);
    }
    ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

    xTimerReset(shutdown_signal_timer, portMAX_DELAY);
    break;
  case WEBSOCKET_EVENT_ERROR:
    ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
    ledc_set_blink_mode(OFF);
    break;
  }
}

static void websocket_app_start(void)
{
  esp_websocket_client_config_t websocket_cfg = {};

  shutdown_signal_timer = xTimerCreate("Websocket shutdown timer", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS,
                                       pdFALSE, NULL, shutdown_signaler);
  shutdown_sema = xSemaphoreCreateBinary();

  websocket_cfg.uri = WEBSOCKET_URI;

  ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

  client = esp_websocket_client_init(&websocket_cfg);
  esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

  esp_websocket_client_start(client);
  xTimerStart(shutdown_signal_timer, portMAX_DELAY);

  while (xSemaphoreTake(shutdown_sema, portMAX_DELAY))
  {
    esp_websocket_client_close(client, portMAX_DELAY);
    ledc_set_blink_mode(OFF);
    ESP_LOGI(TAG, "Websocket Stopped");
    esp_websocket_client_start(client);
    xTimerStart(shutdown_signal_timer, portMAX_DELAY);
  }
  esp_websocket_client_destroy(client);
}

static void send_data_task(void *pvParameters)
{
  const static char *TAGp = "send_data_task";
  send_data_cfg_t send_data_cfg = *(send_data_cfg_t *)pvParameters;
  QueueSetHandle_t send_data_queue_set = send_data_cfg.send_data_queue_set;

  char *buffer = (char *)malloc(sizeof(char) * 8000);
  QueueSetMemberHandle_t activate_member;
  BaseType_t status;
  double ecg_data_5leads[3];
  double ecg_data_3leads[2];
  double ecg_data_1leads[1];
  float eda_data;
  int heart_rate;
  int i = 0;
  // JSON format
  // {                // pRoot
  //   "ecg" : [      // pECG_array
  //     {            // pECG_data
  //       "id" : id,
  //       "data" : data, // double
  //     },
  //     {
  //       "id" : id,
  //       "data" : data,
  //     }
  //   ],
  //   "eda" : [      // pEDA_array
  //     {            // pEDAdata
  //       "id" : id,
  //       "data" : data, //float
  //     }
  //   ]
  //   "hr": heart_rate //int
  // }
  // Recommend not using more than 5000 bytes per message.
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(42));

    status = pdFALSE;
    cJSON *pRoot = cJSON_CreateObject(); // creat JSON root node
    cJSON *pECG_array = cJSON_CreateArray();
    cJSON_AddItemToObject(pRoot, "ecg", pECG_array);
    cJSON *pEDA_array = cJSON_CreateArray();
    cJSON_AddItemToObject(pRoot, "eda", pEDA_array);

    for (i = 0; i < 20; ++i)
    {
      activate_member = xQueueSelectFromSet(send_data_queue_set, 0);
      if (activate_member == NULL)
      {
        break;
      }
      // Sending ECG data
      else if (activate_member == send_data_cfg.ecg_queue)
      {
        // count++;
        cJSON *pECG_data = cJSON_CreateObject();
        cJSON_AddItemToArray(pECG_array, pECG_data);
        // receive 5 leads data
        if (send_data_cfg.ads1293_leads == 5)
        {
          status = xQueueReceive(activate_member, ecg_data_5leads, 0);
          if (status == pdPASS)
          {
            sprintf(send_data_cfg.sECG_id, "%u", send_data_cfg.ecg_id++);
            cJSON_AddStringToObject(pECG_data, "id", send_data_cfg.sECG_id);
            cJSON *pData = cJSON_CreateDoubleArray((const double *)ecg_data_5leads, 3);
            cJSON_AddItemToObject(pECG_data, "data", pData);
          }
        }
        // receive 3 leads data
        else if (send_data_cfg.ads1293_leads == 3)
        {
          status = xQueueReceive(activate_member, ecg_data_3leads, 0);
          if (status == pdPASS)
          {
            sprintf(send_data_cfg.sECG_id, "%u", send_data_cfg.ecg_id++);
            cJSON_AddStringToObject(pECG_data, "id", send_data_cfg.sECG_id);
            cJSON *pData = cJSON_CreateDoubleArray((const double *)ecg_data_3leads, 2);
            cJSON_AddItemToObject(pECG_data, "data", pData);
          }
        }
        // receive 1 leads data
        else if (send_data_cfg.ads1293_leads == 1)
        {
          status = xQueueReceive(activate_member, ecg_data_1leads, 0);
          if (status == pdPASS)
          {
            sprintf(send_data_cfg.sECG_id, "%u", send_data_cfg.ecg_id++);
            cJSON_AddStringToObject(pECG_data, "id", send_data_cfg.sECG_id);
            cJSON *pData = cJSON_CreateDoubleArray((const double *)ecg_data_1leads, 1);
            cJSON_AddItemToObject(pECG_data, "data", pData);
          }
        }
      }

      // Sending EDA data
      else if (activate_member == send_data_cfg.ad5941_queue)
      {
        cJSON *pEDA_data = cJSON_CreateObject();
        cJSON_AddItemToArray(pEDA_array, pEDA_data);

        status = xQueueReceive(activate_member, &eda_data, 0);
        if (status == pdPASS)
        {
          sprintf(send_data_cfg.sEDA_id, "%u", send_data_cfg.eda_id++);
          cJSON_AddStringToObject(pEDA_data, "id", send_data_cfg.sEDA_id);
          cJSON *pData = cJSON_CreateFloatArray(&eda_data, 1);
          cJSON_AddItemToObject(pEDA_data, "data", pData);
        }
      }

      // Sending heart rate data
      else if (activate_member == send_data_cfg.heart_rate_queue)
      {
        status = xQueueReceive(activate_member, &heart_rate, 0);
        if (status == pdPASS)
          cJSON_AddNumberToObject(pRoot, "hr", heart_rate);
      }
    }

    if (!cJSON_PrintPreallocated(pRoot, buffer, 8000, 0))
      ESP_LOGE(TAGp, "sendData is NULL.");

    if (esp_websocket_client_is_connected(client))
    {
      esp_websocket_client_send_text(client, buffer, strlen(buffer), portMAX_DELAY);
    }

    cJSON_Delete(pRoot);

  }
}

void web_server_init(QueueHandle_t ecg_queue, QueueHandle_t ad5941_queue, QueueHandle_t heart_rate_queue,
                     QueueSetHandle_t send_data_queue_set, int ads1293_leads)
{
  send_data_cfg_t send_data_cfg = {
      .ecg_queue = ecg_queue,
      .ad5941_queue = ad5941_queue,
      .heart_rate_queue = heart_rate_queue,
      .send_data_queue_set = send_data_queue_set,
      .ads1293_leads = ads1293_leads,
      .ecg_id = 0,
      .eda_id = 0};

  client = NULL;
  xTaskCreate(send_data_task, "send_data_task", 4000, (void *)&send_data_cfg, 8, NULL);
  xTaskCreate(websocket_app_start, "websocket_app_task", 2000, NULL, 9, NULL);
  // websocket_app_start();
}