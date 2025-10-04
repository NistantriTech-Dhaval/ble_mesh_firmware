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
#include "wifi_manager.h"
#define TAG "WIFI"
static bool reconnect = true;
static bool wifi_started = false;
static int reconnect_attempt = 0;
wifi_ap_record_t g_ap_list[MAX_AP_NUM];
uint16_t g_ap_count = 0;
uint16_t wifi_read_index = 0;
uint16_t wifi_total_len = 0;
char wifi_buffer[1024]; // buffer containing all SSIDs

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
void build_wifi_list()
{
    int offset = 1;  // start from 1 to leave space for length byte

    for (int i = 0; i < g_ap_count; i++)
    {
        char ssid[33]; // max SSID length + null
        memcpy(ssid, g_ap_list[i].ssid, 32);
        ssid[32] = '\0'; // ensure null termination

        // remove trailing spaces and nulls
        for(int j = 31; j >= 0; j--){
            if(ssid[j] == ' ' || ssid[j] == '\0') ssid[j] = '\0';
            else break;
        }

        // Skip empty SSIDs
        if(strlen(ssid) == 0) continue;

        int n = snprintf((char *)(wifi_buffer + offset), sizeof(wifi_buffer) - offset,
                         "%s,", ssid);
        if (n < 0 || offset + n >= sizeof(wifi_buffer))
            break;
        offset += n;
    }

    // Remove last comma if any
    if (offset > 1) {
        wifi_buffer[offset - 1] = '\0';
    } else {
        wifi_buffer[1] = '\0'; // empty buffer after length byte
    }

    // First byte = remaining length after first byte
    wifi_buffer[0] = offset - 1;

    wifi_total_len = offset;
    wifi_read_index = 0; // reset index

    ESP_LOGI(TAG, "Wi-Fi list built, total len=%d, remaining=%d", wifi_total_len, wifi_buffer[0]);
}

void wifi_scan_task(void *pvParameter)
{
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,      // scan all channels
        .show_hidden = true
    };

    while (1) {
        esp_err_t err = esp_wifi_scan_start(&scan_config, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Scan start failed: %d", err);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        // Wait until scan is done
        uint16_t ap_count = 0;
        while (esp_wifi_scan_get_ap_num(&ap_count) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (ap_count > MAX_AP_NUM) ap_count = MAX_AP_NUM; // limit

        wifi_ap_record_t temp_list[MAX_AP_NUM];
        esp_wifi_scan_get_ap_records(&ap_count, temp_list);

        // Atomically copy to global variable
        memcpy(g_ap_list, temp_list, sizeof(wifi_ap_record_t) * ap_count);
        g_ap_count = ap_count; // update count last

        ESP_LOGI(TAG, "Found %d APs", ap_count);
        build_wifi_list();

        vTaskDelay(pdMS_TO_TICKS(10000)); // 5 sec delay
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
        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        return;
    }
    else
    {

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
}
