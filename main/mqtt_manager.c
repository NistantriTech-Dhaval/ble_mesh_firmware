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

static esp_mqtt_client_handle_t client = NULL;
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
            char mesh_type[20]; // Array to hold the custom color
    nvs_get_string_value("mesh_type", mesh_type);
    if (strcmp(mesh_type, "mesh_standalone") == 0 ||strcmp(mesh_type, "mesh_gateway") == 0 )
    {
            xTaskCreate(sensor_data_update, "sensor_data_update", 8192, NULL, 5, NULL);
    }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");

        // Optional: restart client or reconnect manually
        if (mqtt_task_handle)
        {
            vTaskDelete(mqtt_task_handle);
            mqtt_task_handle = NULL;
        }
        esp_mqtt_client_reconnect(client); // Optional: remove if auto-reconnect enabled
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error occurred");
        break;

    default:
        break;
    }
}
void deinit_mqtt_wifi(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
}


static void sensor_data_update(void *arg)
{
    // Seed random once
    srand(time(NULL));

    while (1)
    {
        uint8_t *mac = esp_bt_dev_get_address(); // Must be after esp_bluedroid_enable()
        if (mac)
        {
            ESP_LOGI(TAG, "Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            // Generate random temperature and humidity
            int temp_val = (rand() % 40) + 10;      // 10°C to 49°C
            int humidity_val = (rand() % 80) + 10;  // 10% to 89%

           char json_params[512];
            snprintf(json_params, sizeof(json_params),
                     "{\"%02X:%02X:%02X:%02X:%02X:%02X\":[{\"temp\":%d,\"humidity\":%d}]}",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     temp_val, humidity_val);

            publish_sensor_data("v1/gateway/telemetry", json_params);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get Bluetooth MAC");
        }

        // Delay 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ---- MQTT Start ---- */
void mqtt_app_start(void)
{
    if (client)
        return; // already started
    char uri[200];
    sprintf(uri, "mqtt://%s@%s:%s", "VqI5DD9ZXZ3g24fTO8np", MQTT_HOST, MQTT_PORT);
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .network.disable_auto_reconnect = true, // ✅ ENABLE auto-reconnect
        .session.keepalive = 10,                // ✅ Ensure keepalive for TCP
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

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