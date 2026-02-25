/*
 * LED strip wrapper – uses esp_ws2811 (WS2811). Init at startup; led_strip_set(on/off) for RPC; led_set_rgb for single color.
 */
#include "led_strip.h"
#include "config.h"
#include "esp_ws2811.h"
#include "esp_log.h"
#include <stdbool.h>

#define TAG "LED_STRIP"

static bool s_initialized = false;
static led_strip_t *s_strip = NULL;

void led_strip_init(void)
{
    if (s_initialized)
    {
        return;
    }

    s_strip = ws2811_strip_create(RMT_CHANNEL_0, (gpio_num_t)LED_STRIP_GPIO, (uint16_t)LED_STRIP_LEN);
    if (!s_strip)
    {
        ESP_LOGE(TAG, "WS2811 strip init failed");
        return;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "WS2811 strip init: GPIO %d, %d LEDs", LED_STRIP_GPIO, LED_STRIP_LEN);
}

void led_strip_set(int on)
{
    if (!s_initialized || !s_strip)
    {
        return;
    }

    uint8_t r = on ? 255 : 0;
    uint8_t g = on ? 255 : 0;
    uint8_t b = on ? 255 : 0;

    for (uint16_t i = 0; i < (uint16_t)LED_STRIP_LEN; i++)
    {
        led_strip_set_pixel(s_strip, i, r, g, b);
    }
    led_strip_refresh(s_strip);
}

void led_set_rgb(uint32_t color)
{
    if (!s_strip)
    {
        return;
    }

    uint8_t R = (color >> 16) & 0xFF;
    uint8_t G = (color >> 8) & 0xFF;
    uint8_t B = color & 0xFF;

    for (uint16_t i = 0; i < (uint16_t)LED_STRIP_LEN; i++)
    {
        led_strip_set_pixel(s_strip, i, R, G, B);
    }
    led_strip_refresh(s_strip);
}
