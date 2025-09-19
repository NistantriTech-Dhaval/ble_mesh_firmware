#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "mqtt_manager.h"
#include "config.h"
#include "nvs_manager.h"
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#define TAG "WIFI"
static bool reconnect = true;
static bool wifi_started = false;
static int reconnect_attempt = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        ESP_LOGI(TAG, "sta scan done");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (reconnect)
        {
            reconnect_attempt++;
            int delay_ms = 1000 * reconnect_attempt; // 1s, 2s, 3s...
            if (delay_ms > 30000)
                delay_ms = 30000; // cap at 30s

            ESP_LOGI(TAG, "sta disconnect, retry in %d ms...", delay_ms);

            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            esp_wifi_connect();
        }
        else
        {
            ESP_LOGI(TAG, "sta disconnect (no reconnect)");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        mqtt_app_start();
    }
}
void wifi_init_sta(void)
{
    if (wifi_started)
    {
        ESP_LOGI(TAG, "Wi-Fi already started, skipping init.");
        return;
    }

    char ssid[32];
    char password[64];
    esp_err_t err = nvs_get_wifi_credentials(ssid, password);
    if (err != ESP_OK || strlen(ssid) == 0 || strlen(password) == 0)
    {
        ESP_LOGW(TAG, "No Wi-Fi credentials found. Restarting for provisioning...");
        return;
    }

    // Initialize netif
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(err);
    }

    // Create default event loop
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(err);
    }

    // Create default Wi-Fi STA netif
    esp_netif_create_default_wifi_sta();

    // ✅ INIT WIFI (required!)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // ← THIS was missing

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Log and set Wi-Fi config
    ESP_LOGI(TAG, "Using stored Wi-Fi credentials: SSID: %s", ssid);
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_started = true; // ✅ Mark as started
    ESP_LOGI(TAG, "wifi_init_sta finished. Connecting to SSID: %s", ssid);
}
