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

// Global flags and variables
static bool reconnect = true;          // Should Wi-Fi reconnect automatically on disconnect
static bool wifi_started = false;      // To prevent multiple Wi-Fi initialization
static int reconnect_attempt = 0;      // Counter for exponential backoff on reconnect

wifi_ap_record_t g_ap_list[MAX_AP_NUM]; // Stores scanned APs
uint16_t g_ap_count = 0;                // Number of APs found
uint16_t wifi_read_index = 0;           // Index for reading Wi-Fi buffer in chunks
uint16_t wifi_total_len = 0;            // Total length of Wi-Fi buffer
char wifi_buffer[1024];                 // Buffer containing all SSIDs

// Event handler for Wi-Fi and IP events
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    // Wi-Fi station started
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    // Wi-Fi scan completed
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        ESP_LOGI(TAG, "sta scan done");
    }
    // Wi-Fi disconnected
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (reconnect)
        {
            reconnect_attempt++;
            int delay_ms = 1000 * reconnect_attempt; // Exponential backoff: 1s, 2s, 3s...
            if (delay_ms > 30000)
                delay_ms = 30000; // Cap at 30s

            ESP_LOGI(TAG, "sta disconnect, retry in %d ms...", delay_ms);

            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            esp_wifi_connect();
        }
        else
        {
            ESP_LOGI(TAG, "sta disconnect (no reconnect)");
        }
    }
    // IP assigned
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        mqtt_app_start(); // Start MQTT after getting IP
    }
}

/** Return true if STA is connected to an AP. */
bool wifi_sta_is_connected(void)
{
    wifi_ap_record_t ap_info;
    return (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
}

// Build a comma-separated Wi-Fi SSID list in wifi_buffer
void build_wifi_list(void)
{
    int offset = 1;  // Start from 1 to leave space for length byte

    for (int i = 0; i < g_ap_count; i++)
    {
        char ssid[33]; // max SSID length + null terminator
        memcpy(ssid, g_ap_list[i].ssid, 32);
        ssid[32] = '\0'; // ensure null termination

        // Remove trailing spaces and null characters
        for(int j = 31; j >= 0; j--){
            if(ssid[j] == ' ' || ssid[j] == '\0') ssid[j] = '\0';
            else break;
        }

        // Skip empty SSIDs
        if(strlen(ssid) == 0) continue;

        // Append SSID + comma to buffer
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

    // First byte indicates remaining length after first byte
    wifi_buffer[0] = offset - 1;

    wifi_total_len = offset;
    wifi_read_index = 0; // reset index for GATT read

    ESP_LOGI(TAG, "Wi-Fi list built, total len=%d, remaining=%d", wifi_total_len, wifi_buffer[0]);
}

// FreeRTOS task to scan Wi-Fi periodically
void wifi_scan_task(void *pvParameter)
{
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,      // Scan all channels
        .show_hidden = true
    };

    while (1) {
        // Do not scan or update WiFi list when already connected
        if (wifi_sta_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        // Start Wi-Fi scan (blocking)
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

        // Limit AP count to MAX_AP_NUM
        if (ap_count > MAX_AP_NUM) ap_count = MAX_AP_NUM;

        // Temporary list to fetch scan results
        wifi_ap_record_t temp_list[MAX_AP_NUM];
        esp_wifi_scan_get_ap_records(&ap_count, temp_list);

        // Copy scan results atomically to global variable
        memcpy(g_ap_list, temp_list, sizeof(wifi_ap_record_t) * ap_count);
        g_ap_count = ap_count; // update count last

        ESP_LOGI(TAG, "Found %d APs", ap_count);

        // Build buffer for BLE GATT read
        build_wifi_list();

        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay 10s before next scan
    }
}

// Initialize Wi-Fi in station mode
void wifi_init_sta(void)
{
    if (wifi_started)
    {
        ESP_LOGI(TAG, "Wi-Fi already started, skipping init.");
        return;
    }

    char ssid[32];
    char password[64];
    // Read saved Wi-Fi credentials from NVS
    esp_err_t err = nvs_get_wifi_credentials(ssid, password);
    if (err != ESP_OK || strlen(ssid) == 0 || strlen(password) == 0)
    {
        // No credentials found, just init Wi-Fi stack
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
        // Initialize network interface
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

        // Create default Wi-Fi STA network interface
        esp_netif_create_default_wifi_sta();

        // Initialize Wi-Fi driver
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // Register event handlers for Wi-Fi and IP events
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

        // Set Wi-Fi configuration with stored credentials
        ESP_LOGI(TAG, "Using stored Wi-Fi credentials: SSID: %s", ssid);
        wifi_config_t wifi_config = {};
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        wifi_started = true; // Mark Wi-Fi as started
        ESP_LOGI(TAG, "wifi_init_sta finished. Connecting to SSID: %s", ssid);
    }
}
