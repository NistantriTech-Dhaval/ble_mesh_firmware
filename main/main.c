

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sensor_server.h"
#include "nvs_manager.h"
#include "mesh_handler.h"
#include "led_strip.h"

static const char *TAG = "main";

void app_main(void)
{
    mesh_handler_init();
    led_strip_init();
    ble_mesh_main();
    return;
}
