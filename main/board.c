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
#include "iot_button.h"
#include "board.h"
#include "sensor_server.h"

#define TAG "BOARD"
#define BUTTON_IO_NUM GPIO_NUM_34
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

static void button_tap_cb(void* arg)
{
    ESP_LOGI(TAG, "Button pressed! Toggling LED and sending update to group");
    /* On button press: toggle LED and send update to group so gateway receives it */
    ble_mesh_bulb_toggle_from_button();
    ESP_LOGI(TAG, "Button press handled");
}

static void board_button_init(void)
{
    ESP_LOGI(TAG, "Initializing button on GPIO %d, active level %d", BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
        ESP_LOGI(TAG, "Button initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize button!");
    }
}

void board_init(void)
{
    board_led_init();
    board_button_init();
}
