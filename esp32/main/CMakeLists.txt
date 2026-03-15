idf_component_register(
  SRCS
    "canOPEN.c"
    "microros_node.c"
    "app_main.c"
    "esp32_serial_transport.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES
    nvs_flash
    driver
    esp_timer
  REQUIRES
    micro_ros_espidf
)
