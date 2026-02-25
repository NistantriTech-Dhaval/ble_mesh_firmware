#include "nvs_flash.h"
#include "nvs.h"
#include <stdbool.h>

/** Return true if custom MQTT config (mqtt_host, etc.) is stored in NVS. */
bool nvs_mqtt_config_available(void);

void nvs_save_wifi_credentials(char *ssid, char *password);
esp_err_t nvs_get_wifi_credentials(char *ssid, char *password);
void nvs_init();
void nvs_get_string_value(char *key,char *value);
void nvs_save_string_value(char *key,char *value);
void nvs_delete_key(char *key);
void nvs_erase_all_key();
void nvs_save_blob_value(const char *key, void *data, size_t len);
int nvs_load_blob_value(const char *key, void *data, size_t *len);
int nvs_save_u32_value(const char *key, uint32_t value);
int nvs_load_u32_value(const char *key, uint32_t *value);
