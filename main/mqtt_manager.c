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
#include "cJSON.h"
#include <stdbool.h>
#include "led_strip.h"
#include "sensor_server.h"

#define TAG "MQTT"

/* ThingsBoard topics: device own data vs gateway (other devices) */
#define TOPIC_DEVICE_TELEMETRY "v1/devices/me/telemetry"
#define TOPIC_GATEWAY_TELEMETRY "v1/gateway/telemetry"
#define TOPIC_RPC_REQUEST      "v1/devices/me/rpc/request/+"

// MQTT client handle
static esp_mqtt_client_handle_t client = NULL;

// Handle to the sensor update task
static TaskHandle_t mqtt_task_handle = NULL;

// MQTT connection state: publish only when true
static volatile bool s_mqtt_connected = false;

/* ---- MQTT Event Handler ---- */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_mqtt_connected = true;

        /* Subscribe to ThingsBoard RPC request topic */
        int sub_id = esp_mqtt_client_subscribe(client, TOPIC_RPC_REQUEST, 0);
        if (sub_id >= 0)
        {
            ESP_LOGI(TAG, "Subscribed to RPC topic: %s", TOPIC_RPC_REQUEST);
        }
        else
        {
            ESP_LOGW(TAG, "RPC subscribe failed");
        }

        // Read mesh type from NVS
        char mesh_type[20];
        nvs_get_string_value("mesh_type", mesh_type);

        // Start sensor data update task only for standalone or gateway mesh types
        if (strcmp(mesh_type, "mesh_standalone") == 0 || strcmp(mesh_type, "mesh_gateway") == 0)
        {
            xTaskCreate(sensor_data_update, "sensor_data_update", 8192, NULL, 5, &mqtt_task_handle);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_mqtt_connected = false;

        // Delete the sensor update task if running
        if (mqtt_task_handle)
        {
            vTaskDelete(mqtt_task_handle);
            mqtt_task_handle = NULL;
        }
        // Reconnect is handled by client auto-reconnect with backoff
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error occurred");
        s_mqtt_connected = false;
        break;

case MQTT_EVENT_DATA: {
    int data_len = event->data_len;
    if (data_len <= 0) break;

    char *payload = malloc((size_t)data_len + 1);
    if (!payload) break;

    memcpy(payload, event->data, (size_t)data_len);
    payload[data_len] = '\0';

    ESP_LOGI(TAG, "RPC received payload: %s", payload);

    cJSON *root = cJSON_Parse(payload);
    if (!root) {
        ESP_LOGW(TAG, "RPC payload not valid JSON");
        free(payload);
        break;
    }

    // Accept both:
    // 1) { "method": "...", "params": {...} }
    // 2) { "device": "...", "data": { "id":..., "method":"...", "params": {...} } }
    cJSON *data_obj = cJSON_GetObjectItem(root, "data");
    cJSON *method   = NULL;
    cJSON *params   = NULL;
    cJSON *req_id   = NULL;

    if (cJSON_IsObject(data_obj)) {
        method = cJSON_GetObjectItem(data_obj, "method");
        params = cJSON_GetObjectItem(data_obj, "params");
        req_id = cJSON_GetObjectItem(data_obj, "id");
    } else {
        method = cJSON_GetObjectItem(root, "method");
        params = cJSON_GetObjectItem(root, "params");
        req_id = cJSON_GetObjectItem(root, "id");
    }

    if (cJSON_IsNumber(req_id)) {
        ESP_LOGI(TAG, "RPC id: %d", req_id->valueint);
    }

    if (cJSON_IsString(method)) {
        ESP_LOGI(TAG, "RPC method: %s", method->valuestring);
    } else {
        ESP_LOGW(TAG, "RPC method missing/invalid");
    }

    if (params && cJSON_IsObject(params)) {
        char *params_str = cJSON_PrintUnformatted(params);
        if (params_str) {
            ESP_LOGI(TAG, "RPC params: %s", params_str);
            cJSON_free(params_str);
        }
    }

    // Handle set_bulb
    if (cJSON_IsString(method) && strcmp(method->valuestring, "set_bulb") == 0 && cJSON_IsObject(params)) {
        cJSON *state     = cJSON_GetObjectItem(params, "state");
        cJSON *addr_item = cJSON_GetObjectItem(params, "address");

        int on = 0;

        // Accept "on"/"off", true/false, 1/0
        if (cJSON_IsString(state)) {
            on = (strcmp(state->valuestring, "on") == 0) ? 1 : 0;
        } else if (cJSON_IsBool(state)) {
            on = cJSON_IsTrue(state) ? 1 : 0;
        } else if (cJSON_IsNumber(state)) {
            on = (state->valueint != 0) ? 1 : 0;
        }

        uint16_t own_addr = get_primary_unicast();

        // address in your example is "8" (decimal). Often mesh unicast is like 0x0008.
        uint16_t target = own_addr;
        if (cJSON_IsNumber(addr_item)) {
            target = (uint16_t)addr_item->valueint;
        }

        if (own_addr == 0) {
            ESP_LOGW(TAG, "RPC set_bulb: device not provisioned (own_addr=0)");
        } else if (target == 0) {
            ESP_LOGW(TAG, "RPC set_bulb: invalid target address=0");
        } else if (target == own_addr) {
            // Own device: update + publish
            ble_mesh_set_bulb_attribute(on);
            ESP_LOGI(TAG, "RPC set_bulb: %s (own device, direct update)", on ? "ON" : "OFF");

            char json[64];
            snprintf(json, sizeof(json), "{\"bulb\":\"%s\",\"address\":%u}", on ? "on" : "off", (unsigned)own_addr);
            publish_sensor_data(TOPIC_DEVICE_TELEMETRY, json);
        } else {
            // Other node: mesh command
            ble_mesh_send_bulb_command(target, on);
            ESP_LOGI(TAG, "RPC set_bulb: %s to node %u (0x%04x) (mesh command)",
                     on ? "ON" : "OFF", (unsigned)target, (unsigned)target);
        }
    }

    cJSON_Delete(root);
    free(payload);
    break;
}

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

            /* Gateway: update mesh sensor attribute (temp, hum, MAC) then send to ThingBoard */
            ble_mesh_update_sensor_mesh_attribute((int8_t)temp_val, (int8_t)humidity_val);
            char json_params[128];
            snprintf(json_params, sizeof(json_params),
                     "{\"temp\":%d,\"humidity\":%d}", temp_val, humidity_val);
            publish_sensor_data(TOPIC_DEVICE_TELEMETRY, json_params);
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

    char uri[256];
    char token[128];

    memset(token, 0, sizeof(token));
    nvs_get_string_value("mqtt_tok", token);

    if (strlen(token) == 0)
    {
        ESP_LOGW(TAG, "No MQTT token in NVS, skipping MQTT connect");
        return;
    }

    snprintf(uri, sizeof(uri), "mqtt://%s@%s:%s", token, MQTT_HOST, MQTT_PORT);
    ESP_LOGI(TAG, "MQTT connect using token from NVS");

    // Configure MQTT client: auto-reconnect with backoff to avoid hammering server after drops
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .network.disable_auto_reconnect = false, // Let client reconnect with backoff
        .session.keepalive = 30,                 // 30s keepalive (reduces idle disconnect)
        .network.timeout_ms = 10000,             // Connect/read timeout
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
    if (!s_mqtt_connected || client == NULL)
    {
        ESP_LOGW(TAG, "MQTT not connected, skip publish to %s", topic);
        return;
    }

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
