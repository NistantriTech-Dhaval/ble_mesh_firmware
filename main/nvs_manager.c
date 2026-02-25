#include "nvs_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

// Delete a specific key from NVS
void nvs_delete_key(char *key)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t my_handle;

    // Open NVS namespace "permenant" with read/write access
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);

    // Erase the key
    err = nvs_erase_key(my_handle, key);

    // Commit changes to flash
    nvs_commit(my_handle);

    // Close handle
    nvs_close(my_handle);
}

// Erase all keys in NVS namespace "permenant"
void nvs_erase_all_key()
{
    esp_err_t err = ESP_OK;
    nvs_handle_t my_handle;

    // Open NVS namespace
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);

    // Erase all keys
    err = nvs_erase_all(my_handle);

    // Commit changes
    nvs_commit(my_handle);

    // Close handle
    nvs_close(my_handle);
}

// Read string value from NVS, with default fallback for specific keys
void nvs_get_string_value(char *key, char *value)
{
    nvs_handle_t my_handle;
    esp_err_t err = ESP_OK;

    // Open NVS namespace in read-only mode
    err = nvs_open("permenant", NVS_READONLY, &my_handle);

    size_t required_size = 512;
    err = nvs_get_str(my_handle, key, value, &required_size);

    // If key not found, return default values for specific keys
    if (err != ESP_OK)
    {
        if (strcmp(key, "node_type") == 0)
        {
            printf("Get Default Node Type\n");
            strncpy(value, "sensor_server", 20);
        }
        if (strcmp(key, "mesh_type") == 0)
        {
            printf("Get Default Mesh Type\n");
            strncpy(value, "mesh_node", 20);
        }
        if (strcmp(key, "prov_status") == 0)
        {
            printf("Get Default Prov Status\n");
            strncpy(value, "true", 20);
        }
    }

    // Commit (not strictly needed for read) and close
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

// Save string value to NVS
void nvs_save_string_value(char *key, char *value)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t my_handle;

    // Open NVS namespace with read/write access
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err);

    // Set string value
    err = nvs_set_str(my_handle, key, value);
    if (err != ESP_OK)
    {
        printf("Error setting string in NVS: %s\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return;
    }
    ESP_ERROR_CHECK(err);

    // Commit changes
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        printf("NVS COMMIT Failed\n");
    }

    // Close handle
    nvs_close(my_handle);
}

// Save arbitrary blob data to NVS
void nvs_save_blob_value(const char *key, void *data, size_t len)
{
    esp_err_t err;
    nvs_handle_t my_handle;

    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err);

    err = nvs_set_blob(my_handle, key, data, len);
    if (err != ESP_OK) {
        printf("Error setting blob in NVS: %s\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return;
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        printf("NVS commit failed for blob\n");
    }

    nvs_close(my_handle);
}

// Load blob data from NVS
int nvs_load_blob_value(const char *key, void *data, size_t *len)
{
    esp_err_t err;
    nvs_handle_t my_handle;

    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return -1;

    err = nvs_get_blob(my_handle, key, data, len);
    if (err != ESP_OK) {
        printf("Error reading blob from NVS: %s\n", esp_err_to_name(err));
        nvs_close(my_handle);
        return -1;
    }

    nvs_close(my_handle);
    return 0;
}

// Save 32-bit unsigned integer to NVS
int nvs_save_u32_value(const char *key, uint32_t value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("permenant", NVS_READWRITE, &handle);
    if (err != ESP_OK) return -1;

    err = nvs_set_u32(handle, key, value);
    if (err != ESP_OK) {
        nvs_close(handle);
        return -2;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return (err == ESP_OK) ? 0 : -3;
}

// Load 32-bit unsigned integer from NVS
int nvs_load_u32_value(const char *key, uint32_t *value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("permenant", NVS_READONLY, &handle);
    if (err != ESP_OK) return -1;

    err = nvs_get_u32(handle, key, value);
    nvs_close(handle);
    return (err == ESP_OK) ? 0 : -2;
}

// Save Wi-Fi credentials (SSID + password) to NVS
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

// Read Wi-Fi credentials from NVS
esp_err_t nvs_get_wifi_credentials(char *ssid, char *password)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open namespace
    err = nvs_open("permenant", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Get SSID
    size_t ssid_len = 32;
    err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    // Get password
    size_t password_len = 64;
    err = nvs_get_str(my_handle, "password", password, &password_len);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return err;
    }

    nvs_close(my_handle);
    return ESP_OK;
}

// Initialize NVS flash subsystem
void nvs_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated or outdated, erase and re-initialize
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}
