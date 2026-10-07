idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES crash_detector data_logger freertos gps_tracker
                                headlight_controller motor_controller power_monitor
                                project_types telemetry_link)
