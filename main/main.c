

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sensor_client.h"
#include "sensor_server.h"
#include "nvs_manager.h"
static const char *TAG = "main";

void app_main(void)
{
    nvs_init();
    sensorserver_main();
    return;
}
