set(srcs "main.c"
        "board.c"     "mqtt_manager.c" "wifi_manager.c" "nvs_manager.c"
    "sensor_server.c")

idf_component_register(SRCS  "${srcs}"
                    INCLUDE_DIRS  ".")
