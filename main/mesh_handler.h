#ifndef MESH_HANDLER_H
#define MESH_HANDLER_H

#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>
#define MAX_NODES 50  // You can adjust this depending on your expected mesh size

// Structure to store each mesh node’s information
typedef struct {
    uint16_t unicast_addr;  // Node's BLE Mesh unicast address
    uint8_t mac[6];         // Corresponding MAC address
} mesh_node_t;

// ===== Mesh Handler API =====

// Initialize mesh handler and load data from NVS
void mesh_handler_init(void);

// Add or update a node (returns 1 if added, 0 if updated, -1 if full)
int mesh_handler_add_node(uint16_t unicast_addr, uint8_t mac[6]);

// Fetch MAC address for a given unicast address (returns 0 if found, -1 if not found)
int mesh_handler_get_mac(uint16_t unicast_addr, uint8_t mac_out[6]);

// Fetch unicast address for a given MAC address (returns 0 if found, -1 if not found)
int mesh_handler_get_unicast(uint8_t mac[6], uint16_t *unicast_out);

// Print all stored nodes (for debugging)
void mesh_handler_print_nodes(void);

// Save all mesh nodes to NVS
int mesh_handler_save_to_nvs(void);

// Load all mesh nodes from NVS
int mesh_handler_load_from_nvs(void);
size_t mesh_handler_get_node_count(void);
bool mesh_handler_get_node_info(size_t index, uint16_t *unicast_addr, uint8_t mac_out[6]);


#endif // MESH_HANDLER_H
