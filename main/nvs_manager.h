#include "nvs_flash.h"
#include "nvs.h"

void nvs_save_wifi_credentials(char *ssid, char *password);
esp_err_t nvs_get_wifi_credentials(char *ssid, char *password);
void nvs_init();
void nvs_get_string_value(char *key,char *value);
void nvs_save_string_value(char *key,char *value);
void nvs_delete_key(char *key);