/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include <time.h>
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "custom_characteristic.h"
#include "ble_mesh_example_init.h"
#include "board.h"
#include "mqtt_manager.h"
#include "wifi_manager.h"
#include "nvs_manager.h"
#include "cJSON.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_err.h"
#include "mesh_handler.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "sensor_server.h"
#include "proxy_server.h"

/* Company ID for ESP */
#define CID_ESP 0x02E5
 
// Logging tag for ESP32 logging messages
#define TAG "BLE MESH"

/* Sensor Property IDs */
#define TEMP_PROPERTY_ID_0 0x0056 /* Present Temperature */
#define HUM_PROPERTY_ID_1 0x005B  /* Present Humidity */
#define MAC_PROPERTY_ID_2 0x0060  /* Device MAC Address or Identifier */

/* Vendor model */
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001
#define ESP_BLE_MESH_VND_MODEL_OP_SEND ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

/* Property values (simulated or default) */
static int8_t temperature = 40;                            
static int8_t humidity = 41;                               
uint8_t mac_address[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // dummy 6-byte MAC

/* Device UUID (16 octets) */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

/* Configuration Server */
static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20), /* 3 transmissions, 20ms interval */
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

/* Raw sensor data buffers */
NET_BUF_SIMPLE_DEFINE_STATIC(temperature_data_0, 1);
NET_BUF_SIMPLE_DEFINE_STATIC(humidity_data_1, 1);
NET_BUF_SIMPLE_DEFINE_STATIC(mac_address_data_2, 6);

/* Sensor states (multi-sensor example) */
static esp_ble_mesh_sensor_state_t sensor_states[3] = {
    /* 0: Temperature sensor */
    [0] = {
        .sensor_property_id = TEMP_PROPERTY_ID_0,
        .descriptor = {
            .update_interval = 0,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 0, /* 0 = 1 octet */
            .raw_value = &temperature_data_0,
        },
    },
    /* 1: Humidity sensor */
    [1] = {
        .sensor_property_id = HUM_PROPERTY_ID_1,
        .descriptor = {
            .update_interval = 0,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 0,
            .raw_value = &humidity_data_1,
        },
    },
    /* 2: Device MAC / Identifier */
    [2] = {
        .sensor_property_id = MAC_PROPERTY_ID_2,
        .descriptor = {
            .update_interval = 0,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 0,
            .raw_value = &mac_address_data_2,
        },
    },

};

/* ================= Sensor Server Definition ================= */

/* Sensor server publication context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 100, ROLE_NODE);

/* Standard Sensor Server */
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // Application handles GET responses
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // Application handles SET responses
    },
    .state_count = ARRAY_SIZE(sensor_states), // Number of sensor states supported
    .states = sensor_states,                  // Pointer to sensor states array
};

/* ================= Sensor Setup Server Definition ================= */

/* Sensor setup server publication context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 20, ROLE_NODE);

/* Sensor Setup Server */
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // Application handles GET responses
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP, // Application handles SET responses
    },
    .state_count = ARRAY_SIZE(sensor_states), // Number of sensor states supported
    .states = sensor_states,                  // Pointer to sensor states array
};

/* ================= Vendor Model Definition ================= */

/* Vendor model operations (opcodes) */
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 1),
    ESP_BLE_MESH_MODEL_OP_END,
};

/* Vendor models array */
static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(
        CID_ESP,                          // Company ID (ESP)
        ESP_BLE_MESH_VND_MODEL_ID_SERVER, // Vendor model ID (server)
        vnd_op,                           // Operations supported by this model
        NULL,                             // Pointer to model publication context (not used here)
        NULL                              // Pointer to user data (can be NULL)
        ),
};

/* ================= Root Element Models ================= */

/* Client instance for sensor messages */
static esp_ble_mesh_client_t sensor_client;

/* Root element models for this node */
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),                                   // Configuration server model
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),                   // Standard sensor server
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server), // Sensor setup server
    ESP_BLE_MESH_MODEL_SENSOR_CLI(NULL, &sensor_client)                           // Sensor client model
};

/* ================= Elements and Composition ================= */
/* Define elements for the node
 * Each element can contain multiple models (standard and vendor).
 * In this project, the root element contains both root_models and vendor models.
 */
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

/* Composition data
 * Describes the node composition for provisioning.
 * - cid: Company ID (ESP)
 * - element_count: Number of elements
 * - elements: Pointer to elements array
 */

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

/* ================= Provisioning Information ================= */

/* Our primary unicast address (set at prov_complete). Gateway/provisioner often uses 0x0001. */
static uint16_t s_primary_unicast_addr = 0;

/* Provisioning settings
 * - uuid: Unique device UUID used for identification during provisioning
 */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

/* ================= Custom Sensor Descriptor ================= */
/* Packed structure for a sensor descriptor
 * Provides additional settings for sensor states such as tolerance, sampling, and update interval
 */
struct custom_sensor_descriptor
{
    uint16_t sensor_prop_id;     // Sensor property ID
    uint32_t pos_tolerance : 12; // Positive tolerance
    uint32_t neg_tolerance : 12; // Negative tolerance
    uint32_t sample_func : 8;    // Sampling function
    uint8_t measure_period;      // Measurement period in seconds
    uint8_t update_interval;     // Update interval in seconds
} __attribute__((packed));

/* ================= Custom Sensor Setting ================= */
/* Packed structure for a sensor setting
 * Maps a sensor property to its corresponding setting property.
 * Used to configure adjustable sensor parameters in BLE Mesh.
 */
struct custom_sensor_setting
{
    uint16_t sensor_prop_id;         // Sensor property ID
    uint16_t sensor_setting_prop_id; // Corresponding setting property ID
} __attribute__((packed));

/* ================= Provisioning Complete Callback ================= */
/* This function is called when a node has been successfully provisioned
 * into the BLE Mesh network.
 *
 * Parameters:
 *   net_idx   - Network index assigned to the node
 *   addr      - Unicast address assigned to the node
 *   flags     - Network flags (Relay, Proxy, Friend, Low Power)
 *   iv_index  - Current Initialization Vector (IV) index
 */
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    s_primary_unicast_addr = addr; /* remember our address for own vs other device topic choice */
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
    // Turn off the green LED to indicate provisioning has completed
    board_led_operation(LED_G, LED_OFF);

    /* Initialize sensor data buffers for the provisioned node */

    // Add temperature to the buffer (1 byte)
    net_buf_simple_add_u8(&temperature_data_0, temperature);

    // Add humidity to the buffer (1 byte)
    net_buf_simple_add_u8(&humidity_data_1, humidity);

    // Add MAC address to the buffer (6 bytes)
    net_buf_simple_add_mem(&mac_address_data_2, mac_address, 6);
}

/* ================= BLE Mesh Provisioning Callback ================= */
/* This function handles various BLE Mesh provisioning events for the node.
 * It is called by the ESP BLE Mesh stack whenever a provisioning-related
 * event occurs.
 */
static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                     esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        /* Node registration complete event
         * Check node type from NVS ("sensor_server" or other)
         * - If node, start a task to store MAC address
         * - If gateway, start periodic sensor update task
         */
        char node_type[20]; // Array to hold the custom color
        nvs_get_string_value("node_type", node_type);
        if (strcmp(node_type, "sensor_server") == 0)
        {
            // For sensor server nodes, start a task to store MAC address
            xTaskCreate(store_mac_in_sensor, "store_mac_in_sensor", 2048, NULL, 5, NULL);
        }
        else
        {
            // Start task to request sensor data from other nodes periodically
            xTaskCreate(sensor_update_task, "sensor_update_task", 2048, NULL, 5, NULL);
        }
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        /* Provisioning enable complete event
         * Start Wi-Fi scan task after provisioning is enabled
         */
        xTaskCreate(&wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        nvs_save_string_value("node_type", "sensor_server");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

/**
 * @brief Callback for BLE Mesh Configuration Server events
 *
 * Handles configuration events like:
 * - Adding an AppKey (ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD)
 * - Binding a model to an AppKey (ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND)
 * - Subscribing/unsubscribing models to groups (ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD / DELETE)
 *
 * Why this callback is necessary:
 * The configuration server manages provisioning, AppKey binding, and subscription states
 * in a BLE Mesh node. It allows the device to react to configuration changes, e.g.,
 * promoting a sensor server to sensor client or erasing NVS if unsubscribed.
 */
static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                      esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_sub_add.element_addr,
                     param->value.state_change.mod_sub_add.sub_addr,
                     param->value.state_change.mod_sub_add.company_id,
                     param->value.state_change.mod_sub_add.model_id);
            if (param->value.state_change.mod_sub_add.model_id == 0x1102)
            {
                char node_type[20]; // Array to hold the custom color
                nvs_get_string_value("node_type", node_type);
                ESP_LOGI(TAG, "Current node_type: %s", node_type);

                // If this node was a sensor server, promote it to sensor client (gateway role)
                if (strcmp(node_type, "sensor_server") == 0)
                {
                    ESP_LOGI(TAG, "Changing node_type to 'sensor_client'");
                    nvs_save_string_value("node_type", "sensor_client");
                }
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_DELETE");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_sub_add.element_addr,
                     param->value.state_change.mod_sub_add.sub_addr,
                     param->value.state_change.mod_sub_add.company_id,
                     param->value.state_change.mod_sub_add.model_id);
            // If the unsubscribed model is 0x1102, erase NVS and restart.
            // This effectively converts a gateway back into a standard node.

            if (param->value.state_change.mod_sub_add.model_id == 0x1102)
            {
                ESP_LOGI(TAG, "Model 0x1102 unsubscribed — restart esp32...");
                nvs_erase_all_key();
                esp_restart();
            }
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Send Sensor Descriptor Status message in response to Sensor Descriptor Get
 *
 * Why this callback is necessary:
 * Sensor servers need to respond to descriptor queries from clients. This function
 * generates the descriptor payload for all sensors or a specific sensor property.
 */
static void ble_mesh_send_sensor_descriptor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct custom_sensor_descriptor descriptor = {0};
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    status = (uint8_t *)calloc(1, ARRAY_SIZE(sensor_states) * ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor descriptor status!");
        return;
    }

    if (param->value.get.sensor_descriptor.op_en == false)
    {
        /* Mesh Model Spec:
         * Upon receiving a Sensor Descriptor Get message with the Property ID field
         * omitted, the Sensor Server shall respond with a Sensor Descriptor Status
         * message containing the Sensor Descriptor states for all sensors within the
         * Sensor Server.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
        {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status + length, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length += ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
        }
        goto send;
    }

    for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
    {
        if (param->value.get.sensor_descriptor.property_id == sensor_states[i].sensor_property_id)
        {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length = ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
            goto send;
        }
    }

    /* Mesh Model Spec:
     * When a Sensor Descriptor Get message that identifies a sensor descriptor
     * property that does not exist on the element, the Descriptor field shall
     * contain the requested Property ID value and the other fields of the Sensor
     * Descriptor state shall be omitted.
     */
    memcpy(status, &param->value.get.sensor_descriptor.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Descriptor", status, length);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS, length, status);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Descriptor Status");
    }
    free(status);
}

/**
 * @brief Send Sensor Cadence Status message
 *
 * Why this callback is necessary:
 * Sends cadence (reporting rate) information for a sensor when requested. Currently,
 * cadence is not supported, but a placeholder message is sent to comply with Mesh spec.
 */
static void ble_mesh_send_sensor_cadence_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Cadence state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_STATUS,
                                             ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
                                             (uint8_t *)&param->value.get.sensor_cadence.property_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Cadence Status");
    }
}

/**
 * @brief Send Sensor Settings Status message
 *
 * Why this callback is necessary:
 * Sensor clients may request the list of settings available for a sensor. Currently,
 * settings are not supported, but this function sends a placeholder response.
 */
static void ble_mesh_send_sensor_settings_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Setting state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS,
                                             ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
                                             (uint8_t *)&param->value.get.sensor_settings.property_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Settings Status");
    }
}

/**
 * @brief Send Sensor Setting Status message
 *
 * Why this callback is necessary:
 * Responds to a client querying a specific sensor setting, including access and raw data
 * if available. Ensures the server adheres to the Mesh Model specification.
 */
static void ble_mesh_send_sensor_setting_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct custom_sensor_setting setting = {0};
    esp_err_t err;

    /* Mesh Model Spec:
     * If the message is sent as a response to the Sensor Setting Get message or
     * a Sensor Setting Set message with an unknown Sensor Property ID field or
     * an unknown Sensor Setting Property ID field, the Sensor Setting Access
     * field and the Sensor Setting Raw field shall be omitted.
     */

    setting.sensor_prop_id = param->value.get.sensor_setting.property_id;
    setting.sensor_setting_prop_id = param->value.get.sensor_setting.setting_property_id;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_STATUS,
                                             sizeof(setting), (uint8_t *)&setting);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Setting Status");
    }
}

/**
 * @brief Get Marshalled Sensor Data for a single sensor
 *
 * Why this function is necessary:
 * Converts a sensor's internal data into the BLE Mesh Marshalled Sensor Data format,
 * including the Property ID and raw value, for sending over the mesh network.
 */
static uint16_t ble_mesh_get_sensor_data(esp_ble_mesh_sensor_state_t *state, uint8_t *data)
{
    uint8_t mpid_len = 0, data_len = 0;
    uint32_t mpid = 0;

    if (state == NULL || data == NULL)
    {
        ESP_LOGE(TAG, "%s, Invalid parameter", __func__);
        return 0;
    }

    if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
    {
        /* For zero-length sensor data, the length is 0x7F, and the format is Format B. */
        mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
        mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        data_len = 0;
    }
    else
    {
        if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
        {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN;
        }
        else
        {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
        data_len = state->sensor_data.length + 1;
    }

    memcpy(data, &mpid, mpid_len);
    memcpy(data + mpid_len, state->sensor_data.raw_value->data, data_len);

    return (mpid_len + data_len);
}

/**
 * @brief Send Sensor Data Status message
 *
 * Why this callback is necessary:
 * Handles responses to Sensor Get messages from clients. Supports requests for all
 * properties or a specific property, including random temp/humidity simulation.
 */
static void ble_mesh_send_sensor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    uint32_t mpid = 0;
    esp_err_t err;
    int i;

    /**
     * Sensor Data state from Mesh Model Spec
     * |--------Field--------|-Size (octets)-|------------------------Notes-------------------------|
     * |----Property ID 1----|-------2-------|--ID of the 1st device property of the sensor---------|
     * |-----Raw Value 1-----|----variable---|--Raw Value field defined by the 1st device property--|
     * |----Property ID 2----|-------2-------|--ID of the 2nd device property of the sensor---------|
     * |-----Raw Value 2-----|----variable---|--Raw Value field defined by the 2nd device property--|
     * | ...... |
     * |----Property ID n----|-------2-------|--ID of the nth device property of the sensor---------|
     * |-----Raw Value n-----|----variable---|--Raw Value field defined by the nth device property--|
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
    {
        esp_ble_mesh_sensor_state_t *state = &sensor_states[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
        {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        else
        {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A)
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            }
            else
            {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = (uint8_t *)calloc(1, buf_size);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor status!");
        return;
    }

    if (param->value.get.sensor_data.op_en == false)
    {
        /* Mesh Model Spec:
         * If the message is sent as a response to the Sensor Get message, and if the
         * Property ID field of the incoming message is omitted, the Marshalled Sensor
         * Data field shall contain data for all device properties within a sensor.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
        {
            length += ble_mesh_get_sensor_data(&sensor_states[i], status + length);
        }
        goto send;
    }

    /* Mesh Model Spec:
     * Otherwise, the Marshalled Sensor Data field shall contain data for the requested
     * device property only.
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++)
    {
        if (param->value.get.sensor_data.property_id == sensor_states[i].sensor_property_id)
        {
            // Set random value between 0 and 40
            if (sensor_states[i].sensor_property_id == TEMP_PROPERTY_ID_0 ||
                sensor_states[i].sensor_property_id == HUM_PROPERTY_ID_1)
            {
                if (sensor_states[i].sensor_data.raw_value)
                {
                    sensor_states[i].sensor_data.raw_value->data[0] = (uint8_t)(rand() % 41);
                    sensor_states[i].sensor_data.raw_value->len = 1; // length of your raw value
                }
            }

            length = ble_mesh_get_sensor_data(&sensor_states[i], status);
            goto send;
        }
        if (param->value.get.sensor_data.property_id == sensor_states[i].sensor_property_id)
        {
            length = ble_mesh_get_sensor_data(&sensor_states[i], status);
            goto send;
        }
    }

    /* Mesh Model Spec:
     * Or the Length shall represent the value of zero and the Raw Value field shall
     * contain only the Property ID if the requested device property is not recognized
     * by the Sensor Server.
     */
    mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN,
                                                  param->value.get.sensor_data.property_id);
    memcpy(status, &mpid, ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
    length = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Status");
    }
    free(status);
}

/**
 * @brief Send Sensor Column Status message
 *
 * Why this callback is necessary:
 * Responds to Sensor Column Get messages, sending the requested X-axis raw value
 * for the column. Allows clients to get columnar sensor data from the server.
 */

static void ble_mesh_send_sensor_column_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;

    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN + param->value.get.sensor_column.raw_value_x->len;

    status = (uint8_t *)calloc(1, length);
    if (!status)
    {
        ESP_LOGE(TAG, "No memory for sensor column status!");
        return;
    }

    memcpy(status, &param->value.get.sensor_column.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    memcpy(status + ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN, param->value.get.sensor_column.raw_value_x->data,
           param->value.get.sensor_column.raw_value_x->len);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_STATUS, length, status);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
    free(status);
}

/**
 * @brief Send Sensor Series Status message
 *
 * Why this callback is necessary:
 * Responds to Sensor Series Get messages, sending the requested series data for a
 * specific property ID. Placeholder implementation currently.
 */
static void ble_mesh_send_sensor_series_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
                                             ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_STATUS,
                                             ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
                                             (uint8_t *)&param->value.get.sensor_series.property_id);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
}

/**
 * @brief Sensor Server callback
 *
 * Handles all server-side sensor events:
 * - Receiving GET messages for descriptors, data, cadence, settings, columns, series
 * - Receiving SET messages for cadence or settings
 *
 * Why this callback is necessary:
 * BLE Mesh sensor servers must respond to clients querying or setting sensor data.
 * This central callback dispatches the correct handler for each opcode/event type.
 */
static void ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                      esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
             event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    switch (event)
    {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET");
            ble_mesh_send_sensor_descriptor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET");
            ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            ble_mesh_send_sensor_settings_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            ble_mesh_send_sensor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET");
            ble_mesh_send_sensor_column_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET");
            ble_mesh_send_sensor_series_status(param);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04" PRIx32, param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET");
            ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET");
            ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK");
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04" PRIx32, param->ctx.recv_op);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Server event %d", event);
        break;
    }
}

/**
 * @brief Sensor Timeout Handler
 *
 * Handles timeout events for sensor GET/SET messages.
 *
 * Why this callback is necessary:
 * Allows the application to log and react to messages that did not receive a response
 * from a peer sensor server in time.
 */
static void ble_mesh_sensor_timeout(uint32_t opcode)
{
    switch (opcode)
    {
    case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
        ESP_LOGW(TAG, "Sensor Descriptor Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        ESP_LOGW(TAG, "Sensor Cadence Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
        ESP_LOGW(TAG, "Sensor Cadence Set timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        ESP_LOGW(TAG, "Sensor Settings Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
        ESP_LOGW(TAG, "Sensor Setting Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
        ESP_LOGW(TAG, "Sensor Setting Set timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
        ESP_LOGW(TAG, "Sensor Get timeout, 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
        ESP_LOGW(TAG, "Sensor Column Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
        ESP_LOGW(TAG, "Sensor Series Get timeout, opcode 0x%04" PRIx32, opcode);
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Get/Set opcode 0x%04" PRIx32, opcode);
        return;
    }

    // example_ble_mesh_send_sensor_message(opcode);
}

/**
 * @brief Sensor Client callback
 *
 * Handles sensor client events:
 * - Receiving GET/SET responses
 * - Processing published sensor data
 * - Handling timeouts
 *
 * Why this callback is necessary:
 * Sensor clients need to handle incoming data from servers, parse property IDs,
 * map data to MAC addresses, and optionally publish to a cloud gateway.
 */
static void ble_mesh_sensor_client_cb(esp_ble_mesh_sensor_client_cb_event_t event,
                                      esp_ble_mesh_sensor_client_cb_param_t *param)
{
    esp_ble_mesh_node_t *node = NULL;

    ESP_LOGI(TAG, "Sensor event %u, addr 0x%04x", event, param->params->ctx.addr);

    if (param->error_code)
    {
        ESP_LOGE(TAG, "Send sensor client message failed (err %d)", param->error_code);
        return;
    }

    // node = esp_ble_mesh_provisioner_get_node_with_addr(param->params->ctx.addr);
    // if (!node) {
    //     ESP_LOGE(TAG, "Node 0x%04x not exists", param->params->ctx.addr);
    //     return;
    // }

    switch (event)
    {
    case ESP_BLE_MESH_SENSOR_CLIENT_GET_STATE_EVT:
        switch (param->params->opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG, "Sensor Descriptor Status, opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            if (param->status_cb.descriptor_status.descriptor->len != ESP_BLE_MESH_SENSOR_SETTING_PROPERTY_ID_LEN &&
                param->status_cb.descriptor_status.descriptor->len % ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN)
            {
                ESP_LOGE(TAG, "Invalid Sensor Descriptor Status length %d", param->status_cb.descriptor_status.descriptor->len);
                return;
            }
            if (param->status_cb.descriptor_status.descriptor->len)
            {
                ESP_LOG_BUFFER_HEX("Sensor Descriptor", param->status_cb.descriptor_status.descriptor->data,
                                   param->status_cb.descriptor_status.descriptor->len);
                /* If running with sensor server example, sensor client can get two Sensor Property IDs.
                 * Currently we use the first Sensor Property ID for the following demonstration.
                 */
                // sensor_prop_id = param->status_cb.descriptor_status.descriptor->data[1] << 8 |
                //                  param->status_cb.descriptor_status.descriptor->data[0];
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG, "Sensor Cadence Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.cadence_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Cadence", param->status_cb.cadence_status.sensor_cadence_value->data,
                               param->status_cb.cadence_status.sensor_cadence_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG, "Sensor Settings Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.settings_status.sensor_property_id);
            ESP_LOG_BUFFER_HEX("Sensor Settings", param->status_cb.settings_status.sensor_setting_property_ids->data,
                               param->status_cb.settings_status.sensor_setting_property_ids->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG, "Sensor Setting Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x, Sensor Setting Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.setting_status.sensor_property_id,
                     param->status_cb.setting_status.sensor_setting_property_id);
            if (param->status_cb.setting_status.op_en)
            {
                ESP_LOGI(TAG, "Sensor Setting Access 0x%02x", param->status_cb.setting_status.sensor_setting_access);
                ESP_LOG_BUFFER_HEX("Sensor Setting Raw", param->status_cb.setting_status.sensor_setting_raw->data,
                                   param->status_cb.setting_status.sensor_setting_raw->len);
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "Sensor Status, opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            if (param->status_cb.sensor_status.marshalled_sensor_data &&
                param->status_cb.sensor_status.marshalled_sensor_data->len > 0)
            {
                uint16_t unicast_addr = param->params->ctx.addr;
                ESP_LOGI(TAG, "Sensor status received from server unicast address: 0x%04x", unicast_addr);

                uint8_t *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
                int len = param->status_cb.sensor_status.marshalled_sensor_data->len;
                uint16_t length = 0;

                while (length < len)
                {
                    uint8_t *chunk = data + length;
                    uint8_t fmt = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(chunk);
                    uint8_t data_len = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(chunk, fmt);
                    uint16_t prop_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(chunk, fmt);
                    uint8_t mpid_len = (fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN : ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);

                    ESP_LOGI(TAG, "Format %s, length 0x%02x, Sensor Property ID 0x%04x",
                             fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? "A" : "B",
                             data_len, prop_id);

                    if (data_len != ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
                    {
                        uint8_t *value_data = chunk + mpid_len;
                        ESP_LOG_BUFFER_HEX("Sensor Data", value_data, data_len);

                        if (prop_id == TEMP_PROPERTY_ID_0 || prop_id == HUM_PROPERTY_ID_1)
                        {
                            int8_t raw_val = value_data[0]; // temp or humidity
                            uint8_t mac[6];

                            if (mesh_handler_get_mac(unicast_addr, mac) == 0) // ✅ success
                            {
                                ESP_LOGI(TAG, "Found MAC for 0x%04X: %02X:%02X:%02X:%02X:%02X:%02X",
                                         unicast_addr, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

                                if (prop_id == TEMP_PROPERTY_ID_0)
                                    ESP_LOGI(TAG, "Temperature value from 0x%04X: %d°C", unicast_addr, raw_val);
                                else
                                    ESP_LOGI(TAG, "Humidity value from 0x%04X: %d%%", unicast_addr, raw_val);

                                /* Own device → v1/devices/me/telemetry; other device → v1/gateway/telemetry (gateway only). */
                                uint16_t our_addr = s_primary_unicast_addr;
                                char mesh_type[20];
                                nvs_get_string_value("mesh_type", mesh_type);
                                if (our_addr == 0 && strcmp(mesh_type, "mesh_gateway") == 0)
                                    our_addr = 0x0001; /* provisioner typically uses 0x0001 */

                                if (unicast_addr == our_addr)
                                {
                                    /* This device's own data → device telemetry topic */
                                    char json_params[128];
                                    if (prop_id == HUM_PROPERTY_ID_1)
                                        snprintf(json_params, sizeof(json_params), "{\"humidity\":%d}", raw_val);
                                    else
                                        snprintf(json_params, sizeof(json_params), "{\"temp\":%d}", raw_val);
                                    publish_sensor_data("v1/devices/me/telemetry", json_params);
                                }
                                else if (strcmp(mesh_type, "mesh_gateway") == 0)
                                {
                                    /* Other device's data → gateway telemetry topic (only when we are gateway) */
                                    char json_params[256];
                                    if (prop_id == HUM_PROPERTY_ID_1)
                                        snprintf(json_params, sizeof(json_params),
                                                 "{\"%02X:%02X:%02X:%02X:%02X:%02X\":[{\"humidity\":%d}]}",
                                                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                                                 raw_val);
                                    else
                                        snprintf(json_params, sizeof(json_params),
                                                 "{\"%02X:%02X:%02X:%02X:%02X:%02X\":[{\"temp\":%d}]}",
                                                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                                                 raw_val);
                                    publish_sensor_data("v1/gateway/telemetry", json_params);
                                }
                            }
                            else
                            {
                                ESP_LOGW(TAG, "No MAC found for unicast 0x%04X", unicast_addr);
                            }
                        }
                    }

                    length += mpid_len + data_len + 1;
                }
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG, "Sensor Column Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.column_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Column", param->status_cb.column_status.sensor_column_value->data,
                               param->status_cb.column_status.sensor_column_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG, "Sensor Series Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.series_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Series", param->status_cb.series_status.sensor_series_value->data,
                               param->status_cb.series_status.sensor_series_value->len);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            break;
        }
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_SET_STATE_EVT:
        switch (param->params->opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "Sensor Cadence Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.cadence_status.property_id);
            ESP_LOG_BUFFER_HEX("Sensor Cadence", param->status_cb.cadence_status.sensor_cadence_value->data,
                               param->status_cb.cadence_status.sensor_cadence_value->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG, "Sensor Setting Status, opcode 0x%04" PRIx32 ", Sensor Property ID 0x%04x, Sensor Setting Property ID 0x%04x",
                     param->params->ctx.recv_op, param->status_cb.setting_status.sensor_property_id,
                     param->status_cb.setting_status.sensor_setting_property_id);
            if (param->status_cb.setting_status.op_en)
            {
                ESP_LOGI(TAG, "Sensor Setting Access 0x%02x", param->status_cb.setting_status.sensor_setting_access);
                ESP_LOG_BUFFER_HEX("Sensor Setting Raw", param->status_cb.setting_status.sensor_setting_raw->data,
                                   param->status_cb.setting_status.sensor_setting_raw->len);
            }
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04" PRIx32, param->params->ctx.recv_op);
            break;
        }
        break;
    case ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT:
    {
        ESP_LOGI(TAG, "ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT");

        if (param->status_cb.sensor_status.marshalled_sensor_data &&
            param->status_cb.sensor_status.marshalled_sensor_data->len > 0)
        {
            uint16_t unicast_addr = param->params->ctx.addr;
            ESP_LOGI(TAG, "Sensor status received from server unicast address: 0x%04x", unicast_addr);

            uint8_t *data = param->status_cb.sensor_status.marshalled_sensor_data->data;
            int len = param->status_cb.sensor_status.marshalled_sensor_data->len;
            uint16_t length = 0;

            while (length < len)
            {
                uint8_t fmt = ESP_BLE_MESH_GET_SENSOR_DATA_FORMAT(data);
                uint8_t data_len = ESP_BLE_MESH_GET_SENSOR_DATA_LENGTH(data, fmt);
                uint16_t prop_id = ESP_BLE_MESH_GET_SENSOR_DATA_PROPERTY_ID(data, fmt);
                uint8_t mpid_len = (fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN : ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);

                ESP_LOGI(TAG, "Format %s, length 0x%02x, Sensor Property ID 0x%04x",
                         fmt == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A ? "A" : "B", data_len, prop_id);

                if (data_len != ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN)
                {
                    uint8_t *value_data = data + mpid_len;
                    ESP_LOG_BUFFER_HEX("Sensor Data", data + mpid_len, data_len + 1);
                    if (prop_id == MAC_PROPERTY_ID_2)
                    {
                        uint8_t mac[6] = {0};
                        if (data_len >= 6)
                        {
                            memcpy(mac, data + mpid_len, 6);
                            ESP_LOGI(TAG, "Extracted MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

                            // 🔹 Try to add node — skip if already stored
                            int result = mesh_handler_add_node(unicast_addr, mac);
                            if (result == 1)
                            {
                                ESP_LOGI(TAG, "New node stored: Unicast=0x%04X", unicast_addr);
                            }
                            else if (result == 0)
                            {
                                ESP_LOGI(TAG, "Node already exists, skipping");
                            }
                            else
                            {
                                ESP_LOGW(TAG, "Mesh handler storage full, cannot add node!");
                            }
                        }
                    }
                    length += mpid_len + data_len + 1;
                    data += mpid_len + data_len + 1;
                }
                else
                {
                    length += mpid_len;
                    data += mpid_len;
                }
            }
        }
    }

    break;
    case ESP_BLE_MESH_SENSOR_CLIENT_TIMEOUT_EVT:
        ble_mesh_sensor_timeout(param->params->opcode);
    default:
        break;
    }
}

/**
 * @brief Custom/Vendor Model callback
 *
 * Handles events for custom/vendor BLE Mesh models:
 * - ESP_BLE_MESH_MODEL_OPERATION_EVT: Incoming vendor-specific message
 * - ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT: Late/unsolicited response
 * - ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT: Timeout waiting for server
 * - ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT: Periodic publish update
 *
 * Why this callback is necessary:
 * Custom/vendor models do not have standard behavior defined, so this function allows
 * the application to receive, log, and act on custom messages or timeouts.
 */
static void custom_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
    {
        const uint8_t *data = param->model_operation.msg;
        uint16_t len = param->model_operation.length;

        char *received_msg = malloc(len + 1);
        if (!received_msg)
        {
            ESP_LOGE(TAG, "Failed to allocate memory for message");
            break;
        }

        memcpy(received_msg, data, len);
        received_msg[len] = '\0'; // Null terminate

        ESP_LOGI(TAG, "Received Vendor Message: %s", received_msg);
        free(received_msg);
        break;
    }

    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        ESP_LOGI(TAG, "Late response received from server");
        break;

    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        ESP_LOGW(TAG, "Timeout waiting for response");
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT");
        break;

    default:
        // ESP_LOGW(TAG, "Unknown event %d", event);
        break;
    }
}

// Initialize BLE Mesh and related callbacks
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_APP_ID);
    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(ble_mesh_sensor_server_cb);
    esp_ble_mesh_register_sensor_client_callback(ble_mesh_sensor_client_cb);
    esp_ble_mesh_register_custom_model_callback(custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh sensor server initialized");

    return ESP_OK;
}

// Task to fetch the ESP32 BLE MAC and store it in a sensor state for publishing
static void store_mac_in_sensor(void *arg)
{
    while (1)
    {
        uint8_t *mac = esp_bt_dev_get_address(); // Must be after esp_bluedroid_enable()
        if (mac)
        {
            ESP_LOGI(TAG, "Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get Bluetooth MAC");
        }
        net_buf_simple_reset(&mac_address_data_2);

        // Store the 6-byte MAC
        net_buf_simple_add_mem(&mac_address_data_2, mac, 6);

        // Update sensor state length
        sensor_states[2].sensor_data.length = 6;

        ESP_LOGI(TAG, "MAC stored in sensor_data_2: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        // Prepare payload to publish
        uint8_t status[10]; // buffer for sending sensor data
        uint16_t len = 0;
        len += ble_mesh_get_sensor_data(&sensor_states[2], status + len);

        esp_err_t err = esp_ble_mesh_model_publish(sensor_pub.model,
                                                   ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
                                                   len, status, ROLE_NODE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to publish sensor 2 (err %d)", err);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    } // Update every 60 seconds
}

// Helper function to setup common BLE Mesh client parameters
static void ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                    uint16_t unicast_addr,
                                    esp_ble_mesh_model_t *model,
                                    uint32_t opcode)
{
    common->opcode = opcode;         // Message operation code
    common->model = model;           // Associated BLE Mesh model
    common->ctx.net_idx = 0;         // Network index
    common->ctx.app_idx = 0;         // Application index
    common->ctx.addr = unicast_addr; // Target node unicast address
    common->ctx.send_ttl = 3;        // TTL for message
    common->msg_timeout = 0;         // Timeout
    common->msg_role = ROLE_NODE;    // Node role
}

// Send a SENSOR_GET message to a specific node for a given property
void ble_mesh_send_sensor_message(uint16_t unicast_addr, uint32_t opcode, uint16_t property_id)
{
    esp_ble_mesh_sensor_client_get_state_t get = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err;

    // Prepare message common parameters
    ble_mesh_set_msg_common(&common, unicast_addr, sensor_client.model, opcode);

    // Prepare GET request for specific property ID
    get.sensor_get.property_id = property_id;
    get.sensor_get.op_en = true;

    // Send the GET request
    err = esp_ble_mesh_sensor_client_get_state(&common, &get);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send SENSOR_GET (property 0x%04x) to node 0x%04x", property_id, unicast_addr);
    }
    else
    {
        ESP_LOGI(TAG, "Sent SENSOR_GET (property 0x%04x) to node 0x%04x", property_id, unicast_addr);
    }
}

// Periodic task to request sensor data from all nodes
static void sensor_update_task(void *arg)
{
    uint16_t property_ids[] = {
        TEMP_PROPERTY_ID_0,
        HUM_PROPERTY_ID_1};
    size_t property_count = sizeof(property_ids) / sizeof(property_ids[0]);

    while (1)
    {
        size_t count = mesh_handler_get_node_count();
        ESP_LOGI(TAG, "Starting periodic sensor read for %zu nodes", count);

        for (size_t i = 0; i < count; i++)
        {
            uint16_t unicast_addr;
            uint8_t mac[6];
            if (mesh_handler_get_node_info(i, &unicast_addr, mac))
            {
                ESP_LOGI(TAG, "Requesting sensor data from node 0x%04x", unicast_addr);

                for (size_t j = 0; j < property_count; j++)
                {
                    ble_mesh_send_sensor_message(
                        unicast_addr,
                        ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
                        property_ids[j]);
                    vTaskDelay(pdMS_TO_TICKS(1000)); // short gap between messages
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // Wait 5 minutes before next round
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Main function for BLE Mesh
void ble_mesh_main(void)
{
    esp_err_t err;
    // Initialize board peripherals
    board_init();

    // Initialize Bluetooth
    err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }
    char mesh_type[20]; // Array to hold the custom color
    nvs_get_string_value("mesh_type", mesh_type);
    if (strcmp(mesh_type, "mesh_node") == 0 || strcmp(mesh_type, "mesh_gateway") == 0)
    {

        // Generate device UUID
        ble_mesh_get_dev_uuid(dev_uuid);

        // Initialize BLE Mesh
        err = ble_mesh_init();
        if (err)
        {
            ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
        }
        const uint8_t *mac = esp_bt_dev_get_address();
        ESP_LOGI(TAG, "ESP32 BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        // Create a unique name based on MAC
        char mesh_name[30];
        snprintf(mesh_name, sizeof(mesh_name),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        bt_mesh_set_device_name(mesh_name);
    }
    // Initialize Wi-Fi station
    wifi_init_sta();
}
