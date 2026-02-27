# BLE Mesh Flow Documentation
## Temperature, Humidity, and On/Off Control

---

## 📊 TEMPERATURE & HUMIDITY FLOW

### 🔵 **NODE (sensor_server) Flow**

```
┌─────────────────────────────────────────────────────────────┐
│ NODE: Temperature & Humidity Publishing                      │
└─────────────────────────────────────────────────────────────┘

1. Periodic Task (Every 10 seconds)
   └─> sensor_group_publish_task()
       ├─> Generate random temp (15-35°C) & humidity (30-70%)
       ├─> Get device MAC address
       ├─> Update mesh sensor attributes:
       │   ├─> temperature_data_0 (1 byte)
       │   ├─> humidity_data_1 (1 byte)
       │   └─> mac_address_data_2 (6 bytes)
       └─> Publish to GROUP 0xC000
           └─> ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS
               └─> Marshalled sensor data (all 3 properties)

2. Gateway receives via Sensor Client callback
   └─> ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT
       └─> Parses marshalled data
           └─> Extracts: temp, humidity, MAC
               └─> Uploads to ThingBoard
```

**Key Points:**
- Node publishes **every 10 seconds** to group `0xC000`
- Includes: Temperature, Humidity, MAC address
- Gateway subscribes to group and receives automatically

---

### 🟢 **GATEWAY (mesh_gateway) Flow**

```
┌─────────────────────────────────────────────────────────────┐
│ GATEWAY: Temperature & Humidity Handling                     │
└─────────────────────────────────────────────────────────────┘

A. Own Data (Gateway's own sensors)
   └─> sensor_data_update() task (Every 10 seconds)
       ├─> Generate random temp (10-49°C) & humidity (10-89%)
       ├─> Update mesh sensor attributes:
       │   └─> ble_mesh_update_sensor_mesh_attribute()
       └─> Send directly to ThingBoard
           └─> Topic: v1/devices/me/telemetry
           └─> JSON: {"temp":X, "humidity":Y}

B. Node Data (Received from mesh group)
   └─> ble_mesh_sensor_client_cb()
       └─> ESP_BLE_MESH_SENSOR_CLIENT_PUBLISH_EVT
           ├─> Parse marshalled sensor data
           ├─> Extract: temp, humidity, MAC
           ├─> Check if own MAC or node MAC
           └─> If node MAC:
               └─> Upload to ThingBoard
                   └─> Topic: v1/gateway/telemetry
                   └─> JSON: {"MAC:ADDR":[{"temp":X,"humidity":Y}]}
```

**Key Points:**
- Gateway has **two sources**:
  1. Own sensors → `v1/devices/me/telemetry`
  2. Node sensors → `v1/gateway/telemetry` (gateway format)
- Gateway distinguishes by comparing MAC addresses

---

## 💡 ON/OFF (BULB/LED) FLOW

### 🔵 **NODE (sensor_server) Flow**

```
┌─────────────────────────────────────────────────────────────┐
│ NODE: On/Off Control                                         │
└─────────────────────────────────────────────────────────────┘

A. Button Press (Local)
   └─> button_tap_cb()
       └─> ble_mesh_bulb_toggle_from_button()
           ├─> Get current state: ble_mesh_get_bulb_attribute()
           ├─> Toggle: new_on = !current_on
           ├─> Update LED: ble_mesh_set_bulb_attribute(new_on)
           │   └─> led_strip_set(new_on)
           └─> Publish to GROUP 0xC000
               └─> ble_mesh_publish_onoff_status_to_group()
                   └─> ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS
                       └─> Gateway receives and uploads to ThingBoard

B. RPC Command (From ThingBoard via Gateway)
   └─> Gateway receives RPC: set_bulb
       └─> Gateway sends: ble_mesh_send_bulb_command(node_addr, on)
           └─> Generic OnOff Set_UNACK message (mesh)
               └─> Node receives via: ble_mesh_generic_server_cb()
                   ├─> ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT
                   ├─> Update LED: ble_mesh_set_bulb_attribute(onoff)
                   │   └─> led_strip_set(onoff)
                   └─> Publish to GROUP 0xC000
                       └─> ble_mesh_publish_onoff_status_to_group()
                           └─> Gateway receives and uploads to ThingBoard
```

**Key Points:**
- Button press → Toggle LED → Publish to group
- RPC command → Update LED → Publish to group
- Gateway always receives status updates via group subscription

---

### 🟢 **GATEWAY (mesh_gateway) Flow**

```
┌─────────────────────────────────────────────────────────────┐
│ GATEWAY: On/Off Control                                      │
└─────────────────────────────────────────────────────────────┘

A. Button Press (Local)
   └─> button_tap_cb()
       └─> ble_mesh_bulb_toggle_from_button()
           ├─> Get current state: ble_mesh_get_bulb_attribute()
           ├─> Toggle: new_on = !current_on
           ├─> Update LED: ble_mesh_set_bulb_attribute(new_on)
           │   └─> led_strip_set(new_on)
           └─> Send directly to ThingBoard
               └─> Topic: v1/devices/me/telemetry
               └─> JSON: {"bulb":"on"} or {"bulb":"off"}

B. RPC Command (From ThingBoard)
   └─> MQTT RPC received: set_bulb
       └─> Parse params: state (on/off), address (optional)
           ├─> If address == own address OR omitted:
           │   └─> Update LED directly: ble_mesh_set_bulb_attribute(on)
           │       └─> led_strip_set(on)
           │       └─> Send telemetry: v1/devices/me/telemetry
           │
           └─> If address == other node:
               └─> Send mesh command: ble_mesh_send_bulb_command(addr, on)
                   └─> Generic OnOff Set_UNACK (unicast)
                       └─> Node receives and updates LED
                           └─> Node publishes to group
                               └─> Gateway receives via group subscription
                                   └─> Upload to ThingBoard: v1/gateway/telemetry

C. Group Subscription (Receive from Nodes)
   └─> ble_mesh_generic_client_cb()
       └─> ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT
           ├─> Check source address
           ├─> If own address: Skip (already handled directly)
           └─> If node address:
               ├─> Get node MAC from mesh_handler
               └─> Upload to ThingBoard
                   └─> Topic: v1/gateway/telemetry
                   └─> JSON: {"MAC:ADDR":[{"bulb":"on"}]}
```

**Key Points:**
- Gateway can control:
  1. **Own LED**: Direct update + telemetry
  2. **Node LEDs**: Send mesh command → Node updates → Node publishes → Gateway receives → Upload
- Gateway listens to group `0xC000` for node status updates

---

## 🔄 COMPLETE INTERACTION FLOWS

### **Scenario 1: Node Button Press**
```
Node Button Press
  ↓
Toggle LED (local)
  ↓
Publish OnOff Status → Group 0xC000
  ↓
Gateway receives (group subscription)
  ↓
Extract node MAC address
  ↓
Upload to ThingBoard: v1/gateway/telemetry
  └─> {"MAC:ADDR":[{"bulb":"on"}]}
```

### **Scenario 2: RPC Control Node from ThingBoard**
```
ThingBoard RPC: set_bulb (target: node address)
  ↓
Gateway receives RPC
  ↓
Send Generic OnOff Set_UNACK → Node (unicast)
  ↓
Node receives mesh message
  ↓
Update LED (local)
  ↓
Publish OnOff Status → Group 0xC000
  ↓
Gateway receives (group subscription)
  ↓
Upload to ThingBoard: v1/gateway/telemetry
  └─> {"MAC:ADDR":[{"bulb":"on"}]}
```

### **Scenario 3: RPC Control Gateway from ThingBoard**
```
ThingBoard RPC: set_bulb (target: gateway address or omitted)
  ↓
Gateway receives RPC
  ↓
Update LED directly (local)
  ↓
Send telemetry to ThingBoard: v1/devices/me/telemetry
  └─> {"bulb":"on"}
```

### **Scenario 4: Node Temperature/Humidity Update**
```
Node Periodic Task (10s)
  ↓
Generate temp/humidity values
  ↓
Publish Sensor Status → Group 0xC000
  └─> Marshalled data: temp, humidity, MAC
  ↓
Gateway receives (group subscription)
  ↓
Parse marshalled sensor data
  ↓
Extract: temp, humidity, MAC
  ↓
Upload to ThingBoard: v1/gateway/telemetry
  └─> {"MAC:ADDR":[{"temp":X,"humidity":Y}]}
```

---

## 📋 KEY DATA STRUCTURES

### **Group Addresses**
- **Sensor Group**: `0xC000` - Nodes publish sensor data here
- **OnOff Group**: `0xC000` - Nodes publish OnOff status here

### **MQTT Topics**
- **Device Telemetry**: `v1/devices/me/telemetry` (Gateway's own data)
- **Gateway Telemetry**: `v1/gateway/telemetry` (Node data aggregated)
- **RPC Request**: `v1/devices/me/rpc/request/+` (ThingBoard commands)

### **Mesh Models**
- **Sensor Server**: Node publishes sensor data
- **Sensor Client**: Gateway receives sensor data
- **Generic OnOff Server**: Handles OnOff state changes
- **Generic OnOff Client**: Gateway sends OnOff commands

---

## 🔧 CONFIGURATION

### **Node Configuration**
- `node_type`: `"sensor_server"`
- Publishes to group `0xC000` every 10 seconds
- Button GPIO: `0` (active low)

### **Gateway Configuration**
- `mesh_type`: `"mesh_gateway"`
- `node_type`: `"sensor_client"` (after subscription)
- Subscribes to group `0xC000`
- Own sensor task runs every 10 seconds

---

## 📝 NOTES

1. **LED Strip**: Controlled via `led_strip_set()` function
   - GPIO: Defined in `config.h` (`LED_STRIP_GPIO`)
   - Length: Defined in `config.h` (`LED_STRIP_LEN`)

2. **MAC Address Mapping**: Gateway maintains mapping of unicast addresses to MAC addresses via `mesh_handler`

3. **Own vs Node Data**: Gateway distinguishes by comparing MAC addresses or unicast addresses

4. **Telemetry Format**:
   - Own data: `{"temp":X, "humidity":Y}` or `{"bulb":"on"}`
   - Node data: `{"MAC:ADDR":[{"temp":X,"humidity":Y}]}` or `{"MAC:ADDR":[{"bulb":"on"}]}`
