idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES motor_controller board_config telemetry_link freertos)
