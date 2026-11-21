idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ""
    REQUIRES
        board_config
        crash_detector
        data_logger
        gps_tracker
        headlight_controller
        motor_controller
        power_monitor
        telemetry_link
        esp_driver_i2c
        driver
        nvs_flash
)