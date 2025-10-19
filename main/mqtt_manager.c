#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_event.h"
#include "config.h"
#include "esp_wifi.h"
#include <stdlib.h> // for rand()
#include <time.h>   // for seeding
#include "nvs_manager.h"
#include "mqtt_manager.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#define TAG "MQTT"

// MQTT client handle
static esp_mqtt_client_handle_t client = NULL;

// Handle to the sensor update task
static TaskHandle_t mqtt_task_handle = NULL;

/* ---- MQTT Event Handler ---- */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");

        // Read mesh type from NVS
        char mesh_type[20];
        nvs_get_string_value("mesh_type", mesh_type);

        // Start sensor data update task only for standalone or gateway mesh types
        if (strcmp(mesh_type, "mesh_standalone") == 0 || strcmp(mesh_type, "mesh_gateway") == 0)
        {
            xTaskCreate(sensor_data_update, "sensor_data_update", 8192, NULL, 5, NULL);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");

        // Delete the sensor update task if running
        if (mqtt_task_handle)
        {
            vTaskDelete(mqtt_task_handle);
            mqtt_task_handle = NULL;
        }

        // Attempt reconnection (optional: auto-reconnect can be enabled)
        esp_mqtt_client_reconnect(client);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error occurred");
        break;

    default:
        break;
    }
}

/* ---- Deinitialize MQTT and Wi-Fi ---- */
void deinit_mqtt_wifi(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
}

/* ---- Sensor Data Update Task ---- */
static void sensor_data_update(void *arg)
{
    // Seed random number generator
    srand(time(NULL));

    while (1)
    {
        // Get Bluetooth MAC address
        uint8_t *mac = esp_bt_dev_get_address(); // Must call after esp_bluedroid_enable()
        if (mac)
        {
            ESP_LOGI(TAG, "Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            // Generate random temperature (10°C-49°C) and humidity (10%-89%)
            int temp_val = (rand() % 40) + 10;
            int humidity_val = (rand() % 80) + 10;

            // Prepare JSON payload
            char json_params[512];
            snprintf(json_params, sizeof(json_params),
                     "{\"%02X:%02X:%02X:%02X:%02X:%02X\":[{\"temp\":%d,\"humidity\":%d}]}",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     temp_val, humidity_val);

            // Publish sensor data
            publish_sensor_data("v1/gateway/telemetry", json_params);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get Bluetooth MAC");
        }

        // Delay 10 seconds between updates
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ---- Start MQTT Client ---- */
void mqtt_app_start(void)
{
    if (client)
        return; // MQTT already started

    char uri[200];
    char mesh_type[20];

    // Get mesh type from NVS
    nvs_get_string_value("mesh_type", mesh_type);
    ESP_LOGI(TAG, "Current mesh_type: %s", mesh_type);

    // Set MQTT URI based on mesh type
    if (strcmp(mesh_type, "mesh_gateway") == 0)
    {
        sprintf(uri, "mqtt://%s@%s:%s", "VqI5DD9ZXZ3g24fTO8np", MQTT_HOST, MQTT_PORT);
    }
    else if (strcmp(mesh_type, "mesh_standalone") == 0)
    {
        sprintf(uri, "mqtt://%s@%s:%s", "ssDSjLfucBarZeztbonm", MQTT_HOST, MQTT_PORT);
    }
    else
    {
        sprintf(uri, "mqtt://%s@%s:%s", "VqI5DD9ZXZ3g24fTO8np", MQTT_HOST, MQTT_PORT);
    }

    // Configure MQTT client
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .network.disable_auto_reconnect = true, // Disable automatic reconnect
        .session.keepalive = 10,                // TCP keepalive interval
    };

    // Initialize MQTT client
    client = esp_mqtt_client_init(&cfg);

    // Register MQTT event handler
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // Start MQTT client
    esp_mqtt_client_start(client);
}

/* ---- Publish Sensor Data ---- */
void publish_sensor_data(const char *topic, const char *data)
{
    int msg_id = esp_mqtt_client_publish(client, topic, data, 0, 1, 0);

    if (msg_id != -1)
    {
        printf("Data successfully published to topic: %s, message ID: %d\n", topic, msg_id);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to publish data to topic: %s\n", topic);
    }
}
