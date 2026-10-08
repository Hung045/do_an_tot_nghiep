idf_component_register(SRCS "DataLogger.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES board_config driver esp_common freertos project_types)
