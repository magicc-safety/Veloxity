idf_component_register(
    SRCS "bridge.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart
        esp_driver_usb_serial_jtag
        esp_event
        esp_stdio
        esp_wifi
        nvs_flash
)
