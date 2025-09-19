#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_event.h"
#include "config.h"
#include "esp_wifi.h"
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