#include <stdint.h>

void mqtt_app_start(void);
void publish_sensor_data(const char *topic, const char *data);
void deinit_mqtt_wifi(void);
static void sensor_data_update(void *arg);