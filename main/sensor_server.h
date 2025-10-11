#ifndef SENSOR_SERVER_H
#define SENSOR_SERVER_H

void sensorserver_main(void);

static void sensor_update_task(void *arg);
static void store_mac_in_sensor(void *arg);
#endif // SENSOR_SERVER_H
