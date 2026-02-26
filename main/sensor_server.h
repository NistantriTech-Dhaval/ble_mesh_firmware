#ifndef SENSOR_SERVER_H
#define SENSOR_SERVER_H

void ble_mesh_main(void);

/** Bulb attribute: update on/off state (drives LED strip). Call from RPC or mesh. */
void ble_mesh_set_bulb_attribute(int on);

/** Bulb attribute: get current on/off state (0 = off, 1 = on). */
int ble_mesh_get_bulb_attribute(void);

/** Send bulb on/off command over the mesh to given address (group or unicast). */
void ble_mesh_send_bulb_command(uint16_t unicast_addr, int on);

/** This node's primary unicast address (0 if not provisioned). */
uint16_t get_primary_unicast(void);

#endif // SENSOR_SERVER_H
