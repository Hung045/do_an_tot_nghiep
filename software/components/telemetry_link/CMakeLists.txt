idf_component_register(SRCS "TelemetryLink.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_event esp_netif esp_wifi mqtt nvs_flash project_types)
