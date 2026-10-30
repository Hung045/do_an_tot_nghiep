#pragma once

#include <atomic>
#include <stdbool.h>

#include "ProjectTypes.h"
#include "esp_err.h"
#include "esp_event.h"
#include "mqtt_client.h"

class TelemetryLink {
public:
    // Empty URI starts Wi-Fi only. Call once; keep this object alive for callbacks.
    esp_err_t initialize(const char *brokerUri);
    esp_err_t publishTelemetry(const TelemetryRecord &record);
    esp_err_t publishMotorTelemetry(uint32_t uptimeMs, const char *phase, int raw,
                                    uint32_t throttlePercent, float filteredPercent,
                                    uint32_t dutyPermille, bool relayOn);
    esp_err_t publishEvent(VehicleEvent event, const GpsFix &fix);

private:
    static void wifiEventHandler(
        void *argument,
        esp_event_base_t eventBase,
        int32_t eventId,
        void *eventData);
    static void mqttEventHandler(
        void *argument,
        esp_event_base_t eventBase,
        int32_t eventId,
        void *eventData);
    esp_err_t startMqttClient();

    char brokerUri_[128] = {};
    std::atomic<esp_mqtt_client_handle_t> mqttClient_{nullptr};
    std::atomic<bool> mqttStarted_{false};
    std::atomic<bool> mqttConnected_{false};
};
