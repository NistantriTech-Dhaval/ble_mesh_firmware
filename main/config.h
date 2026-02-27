#define MQTT_HOST "tbce.nistantritech.com"
#define MQTT_PORT "1883"
#define DEFAULT_WIFI_SSID "Airtel_NTPL"     // Change this to your Wi-Fi SSID (must be a 2.4 GHz network, 5 GHz not supported)
#define DEFAULT_WIFI_PASS "NTPL#1234"       // Change this to your Wi-Fi password 
#define PROV_DEVOCE_NAME "NTPL_DEVICE"

/* LED strip: WS2811/WS2812 on one GPIO (RMT). Set length to your strip size. */
#define LED_STRIP_GPIO  GPIO_NUM_13
#define LED_STRIP_LEN   1

/* BLE Mesh: bulb control is by unicast only (particular node). No group subscription required. */