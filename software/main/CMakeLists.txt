idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES motor_controller board_config telemetry_link freertos
                               crash_detector data_logger gps_tracker
                               headlight_controller power_monitor project_types)
