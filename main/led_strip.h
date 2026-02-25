
 //-------------- esp_event -----------------
#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#include "esp_event.h"
#include "driver/gpio.h"

ESP_EVENT_DECLARE_BASE(LED_EVENT);

typedef enum {
    LED_ON_EVENT,
    LED_OFF_EVENT,
    LED_RGB_EVENT,
    LED_TOGGLE_EVENT
} led_event_id_t;

extern esp_event_loop_handle_t event_loop; // Declare event_loop

void led_handler_init();
void led_event_handler(void* handler_arg, esp_event_base_t base, int32_t event_id, void* event_data);
void led_turn_on();
void led_turn_off();
void led_set_rgb(uint32_t color) ;
void post_led_event(int32_t event_id, void *event_data, size_t data_size) ;
void led_strip_init(void);
void led_strip_set(int on);   /* 1 = all white, 0 = all off (for RPC led_status) */
void led_set_rgb(uint32_t color);
#endif // LED_HANDLER_H

