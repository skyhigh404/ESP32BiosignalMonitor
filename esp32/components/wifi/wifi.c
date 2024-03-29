#include "wifi.h"
#include <string.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;
static uint8_t s_max_retry = 5;
static const char *TAG = "wifi station";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  // if (event_id == WIFI_EVENT_AP_STACONNECTED) {
  //     wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
  //     ESP_LOGI("wifi softAP", "station "MACSTR" join, AID=%d",
  //              MAC2STR(event->mac), event->aid);
  // } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
  //     wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
  //     ESP_LOGI("wifi softAP", "station "MACSTR" leave, AID=%d",
  //              MAC2STR(event->mac), event->aid);
  // }
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    esp_wifi_connect();
    // ESP_LOGI(TAG, "try to connect to the AP");

    // if (s_retry_num < s_max_retry)
    // {
    //   esp_wifi_connect();
    //   s_retry_num++;
    //   ESP_LOGI(TAG, "try to connect to the AP");
    // }
    // else
    // {
    //   ESP_LOGI(TAG, "exceeded maximum number of retries");
    //   xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    // }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
  }
}

void wifi_init(wifi_cfg_t wifi_config)
{
  // ESP_ERROR_CHECK(esp_netif_init());
  // ESP_ERROR_CHECK(esp_event_loop_create_default());
  // esp_netif_create_default_wifi_ap();

  // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  // ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
  //                                                     ESP_EVENT_ANY_ID,
  //                                                     &wifi_event_handler,
  //                                                     NULL,
  //                                                     NULL));

  // wifi_config_t wifi_setup = {
  //     .ap = {
  //         .ssid_len = strlen(wifi_config.ssid),
  //         .channel = wifi_config.channel,
  //         .max_connection = wifi_config.max_connection,
  //         .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  // };
  // memcpy(wifi_setup.ap.ssid, wifi_config.ssid, strlen(wifi_config.ssid));
  // memcpy(wifi_setup.ap.password, wifi_config.password, strlen(wifi_config.password));

  // if (strlen(wifi_config.password) == 0)
  // {
  //   wifi_setup.ap.authmode = WIFI_AUTH_OPEN;
  // }

  // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_setup));
  // ESP_ERROR_CHECK(esp_wifi_start());

  // ESP_LOGI("WIFI softAP", "init finished. SSID:%s password:%s channel:%d",
  //          wifi_config.ssid, wifi_config.password, wifi_config.channel);

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_got_ip));

  wifi_config_t wifi_setup = {
      .sta = {
          .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          .pmf_cfg = {
              .capable = true,
              .required = false},
      },
  };
  memcpy(wifi_setup.sta.ssid, wifi_config.ssid, strlen(wifi_config.ssid));
  memcpy(wifi_setup.sta.password, wifi_config.password, strlen(wifi_config.password));
  s_max_retry = wifi_config.max_retry;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_setup));
  ESP_ERROR_CHECK(esp_wifi_start());

  // ESP_LOGI(TAG, "wifi_init_sta finished.");

  // /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
  //  * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  // EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
  //                                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
  //                                        pdFALSE,
  //                                        pdFALSE,
  //                                        portMAX_DELAY);

  // /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
  //  * happened. */
  // if (bits & WIFI_CONNECTED_BIT)
  // {
  //   ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
  //            wifi_setup.sta.ssid, wifi_setup.sta.password);
  // }
  // else if (bits & WIFI_FAIL_BIT)
  // {
  //   ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
  //            wifi_setup.sta.ssid, wifi_setup.sta.password);
  // }
  // else
  // {
  //   ESP_LOGE(TAG, "UNEXPECTED EVENT");
  // }

  // /* The event will not be processed after unregister */
  // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
  // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
  // vEventGroupDelete(s_wifi_event_group);
}
