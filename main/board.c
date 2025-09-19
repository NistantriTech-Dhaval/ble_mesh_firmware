/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"

#include "esp_ble_mesh_sensor_model_api.h"
// #include "iot_button.h"
#include "board.h"


extern void example_ble_mesh_send_sensor_message(uint32_t opcode);

#define TAG "BOARD"
#define BUTTON_IO_NUM 0
#define BUTTON_ACTIVE_LEVEL 0

struct _led_state led_state[3] = {
    {LED_OFF, LED_OFF, LED_R, "red"},
    {LED_OFF, LED_OFF, LED_G, "green"},
    {LED_OFF, LED_OFF, LED_B, "blue"},
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++)
    {
        if (led_state[i].pin != pin)
        {
            continue;
        }
        if (onoff == led_state[i].previous)
        {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++)
    {
        gpio_reset_pin(led_state[i].pin);
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        led_state[i].previous = LED_OFF;
    }
}

// static uint32_t send_opcode[] = {
//     [0] = ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET,
//     [1] = ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET,
//     [2] = ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET,
//     [3] = ESP_BLE_MESH_MODEL_OP_SENSOR_GET,
//     [4] = ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET,
// };
// static uint8_t press_count;

// static void button_tap_cb(void* arg)
// {
//    example_ble_mesh_send_sensor_message(send_opcode[press_count++]);
//     press_count = press_count % ARRAY_SIZE(send_opcode);
// }

// static void board_button_init(void)
// {
//     button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
//     if (btn_handle) {
//         iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
//     }
// }

void board_init(void)
{
    board_led_init();
    // board_button_init();
}
