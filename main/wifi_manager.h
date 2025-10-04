#include "esp_wifi.h"

#define MAX_AP_NUM 20

// Only declare globals here
extern wifi_ap_record_t g_ap_list[MAX_AP_NUM];
extern uint16_t g_ap_count;
#define CHUNK_SIZE 20 // max payload per read
extern uint16_t wifi_read_index;
extern uint16_t wifi_total_len;
extern char wifi_buffer[1024]; // buffer containing all SSIDs

void wifi_init_sta(void);
void wifi_scan_task(void *pvParameter);
void build_wifi_list();