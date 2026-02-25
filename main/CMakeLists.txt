set(srcs "main.c"
        "board.c" "led_strip.c" "esp_ws2811.c" "mqtt_manager.c" "wifi_manager.c" "nvs_manager.c"
    "sensor_server.c" "custom_characteristic.c" "mesh_handler.c")

idf_component_register(SRCS  "${srcs}"
                    INCLUDE_DIRS  ".")
