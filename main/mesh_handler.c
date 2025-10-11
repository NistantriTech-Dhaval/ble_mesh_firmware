#include "mesh_handler.h"
#include "nvs_manager.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static mesh_node_t node_map[MAX_NODES];
static size_t node_count = 0;
#define TAG "MESH HANDLER"

void mesh_handler_init(void) {
    node_count = 0;
    memset(node_map, 0, sizeof(node_map));

    nvs_init(); // initialize NVS
    mesh_handler_load_from_nvs();
}

int mesh_handler_add_node(uint16_t unicast_addr, uint8_t mac[6]) {
    ESP_LOGI(TAG, "Attempting to add/update node: Unicast=0x%04X, MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             unicast_addr, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Update existing node
    for (size_t i = 0; i < node_count; i++) {
        if (node_map[i].unicast_addr == unicast_addr) {
            memcpy(node_map[i].mac, mac, 6);
            mesh_handler_save_to_nvs();
            ESP_LOGI(TAG, "Node updated successfully");
            return 0;
        }
    }

    // Add new node
    if (node_count >= MAX_NODES) {
        ESP_LOGW(TAG, "Node map full, cannot add node");
        return -1;
    }

    node_map[node_count].unicast_addr = unicast_addr;
    memcpy(node_map[node_count].mac, mac, 6);
    node_count++;
    mesh_handler_save_to_nvs();

    ESP_LOGI(TAG, "Node added successfully. Total nodes = %zu", node_count);
    return 1;
}

int mesh_handler_get_mac(uint16_t unicast_addr, uint8_t mac_out[6]) {
    for (size_t i = 0; i < node_count; i++) {
        if (node_map[i].unicast_addr == unicast_addr) {
            memcpy(mac_out, node_map[i].mac, 6);
            return 0;
        }
    }
    return -1;
}

int mesh_handler_get_unicast(uint8_t mac[6], uint16_t *unicast_out) {
    for (size_t i = 0; i < node_count; i++) {
        if (memcmp(node_map[i].mac, mac, 6) == 0) {
            *unicast_out = node_map[i].unicast_addr;
            return 0;
        }
    }
    return -1;
}

void mesh_handler_print_nodes(void) {
    printf("Mesh Node Map (%zu nodes):\n", node_count);
    for (size_t i = 0; i < node_count; i++) {
        printf("Unicast: 0x%04X, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
               node_map[i].unicast_addr,
               node_map[i].mac[0], node_map[i].mac[1], node_map[i].mac[2],
               node_map[i].mac[3], node_map[i].mac[4], node_map[i].mac[5]);
    }
}

// ---------------- NVS Storage (Updated to Binary) ----------------

int mesh_handler_save_to_nvs(void) {
    // Save node count as uint32
    nvs_save_u32_value("mesh_node_count", (uint32_t)node_count);

    // Save node map as binary blob
    nvs_save_blob_value("mesh_nodes", node_map, sizeof(mesh_node_t) * node_count);

    ESP_LOGI(TAG, "Saved %zu nodes to NVS", node_count);
    return 0;
}

int mesh_handler_load_from_nvs(void) {
    uint32_t count = 0;
    if (nvs_load_u32_value("mesh_node_count", &count) == 0) {
        if (count > MAX_NODES) count = MAX_NODES;
        node_count = count;
    } else {
        node_count = 0;
    }

    size_t len = sizeof(mesh_node_t) * node_count;
    if (len > 0) {
        if (nvs_load_blob_value("mesh_nodes", node_map, &len) != 0) {
            ESP_LOGW(TAG, "Failed to load node map from NVS");
            node_count = 0;
        }
    }

    ESP_LOGI(TAG, "Loaded %zu nodes from NVS", node_count);
    return 0;
}

// ---------------- Helpers ----------------

size_t mesh_handler_get_node_count(void) {
    return node_count;
}

bool mesh_handler_get_node_info(size_t index, uint16_t *unicast_addr, uint8_t mac_out[6]) {
    if (index >= node_count) return false;
    if (unicast_addr) *unicast_addr = node_map[index].unicast_addr;
    if (mac_out) memcpy(mac_out, node_map[index].mac, 6);
    return true;
}
