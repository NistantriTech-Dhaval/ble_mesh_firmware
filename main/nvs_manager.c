#include "nvs_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

void nvs_delete_key(char *key)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t my_handle;
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    err = nvs_erase_key(my_handle, key);
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

void nvs_get_string_value(char *key, char *value)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;

    err = nvs_open("permenant", NVS_READONLY, &my_handle);
    size_t required_size = 512;
    err = nvs_get_str(my_handle, key, value, &required_size);

    if (err != ESP_OK)
    {
                if (strcmp(key, "node_type") == 0)
        {
            printf("Get Default Node Type\n");
            // If the key is led_strip_mode, return the default mode
            strncpy(value, "sensor_sever", 20);
        }
                  if (strcmp(key, "prov_status") == 0)
        {
            printf("Get Default Prov Status\n");
            // If the key is led_strip_mode, return the default mode
            strncpy(value, "true", 20);
        }
    }
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

void nvs_save_string_value(char *key, char *value)
{
    esp_err_t err = ESP_OK;

    nvs_handle_t my_handle;
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err);
    err = nvs_set_str(my_handle, key, value);
    if (err != ESP_OK)
    {
        printf("Error setting string in NVS: %s\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return;
    }
    ESP_ERROR_CHECK(err);

    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        printf("NVS COMMIT Failed\n");
    }
    nvs_close(my_handle);
}

void nvs_save_wifi_credentials(char *ssid, char *password)
{
    esp_err_t err = ESP_OK;

    nvs_handle_t my_handle;
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err);
    err = nvs_set_str(my_handle, "ssid", ssid);
    ESP_ERROR_CHECK(err);
    err = nvs_set_str(my_handle, "password", password);
    ESP_ERROR_CHECK(err);

    nvs_commit(my_handle);
    nvs_close(my_handle);
}
esp_err_t nvs_get_wifi_credentials(char *ssid, char *password)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open NVS namespace
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        return err;
    }

    // Read SSID
    size_t ssid_len = 32;
    err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
    if (err != ESP_OK)
    {
        nvs_close(my_handle);
        return err;
    }

    // Read password
    size_t password_len = 64;
    err = nvs_get_str(my_handle, "password", password, &password_len);
    if (err != ESP_OK)
    {
        nvs_close(my_handle);
        return err;
    }

    // Close NVS
    nvs_close(my_handle);

    return ESP_OK;
}
void nvs_init()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}
