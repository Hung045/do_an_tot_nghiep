#include "TelemetryLink.h"

#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace {

constexpr char TAG[] = "telemetry_link";
constexpr char kTelemetryTopic[] = "vehicle/telemetry";
constexpr char kMotorTopic[] = "vehicle/motor";
constexpr char kEventTopic[] = "vehicle/event";

const char *eventName(VehicleEvent event)
{
    switch (event) {
    case VehicleEvent::CrashDetected:
        return "crash_detected";
    case VehicleEvent::PowerWarning:
        return "power_warning";
    case VehicleEvent::SensorFault:
        return "sensor_fault";
    }
    return nullptr;
}

void formatCoordinate(double coordinate, bool valid, char *output, size_t outputSize)
{
    if (valid) {
        std::snprintf(output, outputSize, "%.7f", coordinate);
    } else {
        std::snprintf(output, outputSize, "null");
    }
}

} // namespace

esp_err_t TelemetryLink::initialize(const char *brokerUri)
{
    if (brokerUri == nullptr ||
        CONFIG_DEMO_WIFI_SSID[0] == '\0') {
        ESP_LOGE(TAG, "Set Wi-Fi SSID in menuconfig > Vehicle telemetry");
        return ESP_ERR_INVALID_ARG;
    }

    const size_t brokerUriLength = std::strlen(brokerUri);
    if (brokerUriLength >= sizeof(brokerUri_)) {
        return ESP_ERR_INVALID_ARG;
    }
    std::memcpy(brokerUri_, brokerUri, brokerUriLength + 1);

    wifi_config_t wifiConfig = {};
    const size_t ssidLength = std::strlen(CONFIG_DEMO_WIFI_SSID);
    const size_t passwordLength = std::strlen(CONFIG_DEMO_WIFI_PASSWORD);
    if (ssidLength > sizeof(wifiConfig.sta.ssid) ||
        passwordLength >= sizeof(wifiConfig.sta.password)) {
        return ESP_ERR_INVALID_ARG;
    }
    std::memcpy(wifiConfig.sta.ssid, CONFIG_DEMO_WIFI_SSID, ssidLength);
    std::memcpy(wifiConfig.sta.password, CONFIG_DEMO_WIFI_PASSWORD, passwordLength);

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        // Do not erase application NVS automatically on initialization failure.
        ESP_LOGE(TAG, "NVS initialization failed: %s; Wi-Fi not started", esp_err_to_name(err));
        return err;
    }
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (esp_netif_create_default_wifi_sta() == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifiInitConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &TelemetryLink::wifiEventHandler,
        this);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &TelemetryLink::wifiEventHandler,
        this);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_config(WIFI_IF_STA, &wifiConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "Wi-Fi started asynchronously; %s",
             brokerUri_[0] == '\0' ? "Wi-Fi-only test (MQTT disabled)" : "MQTT starts after obtaining IP");
    return ESP_OK;
}

esp_err_t TelemetryLink::publishMotorTelemetry(
    uint32_t uptimeMs, const char *phase, int raw, uint32_t throttlePercent,
    float filteredPercent, uint32_t dutyPermille, bool relayOn)
{
    const esp_mqtt_client_handle_t client = mqttClient_.load();
    if (client == nullptr || !mqttConnected_.load()) {
        return ESP_ERR_INVALID_STATE; // Skip offline samples instead of accumulating them.
    }
    // Phase is supplied by the application's fixed state names.
    if (phase == nullptr || throttlePercent > 100U || dutyPermille > 1000U) {
        return ESP_ERR_INVALID_ARG;
    }
    char payload[256] = {};
    const int length = std::snprintf(
        payload, sizeof(payload),
        "{\"uptime_ms\":%lu,\"phase\":\"%s\",\"raw\":%d,\"throttle_pct\":%lu,"
        "\"filtered_pct\":%.1f,\"pwm_pct\":%.1f,\"relay_cmd\":%s}",
        static_cast<unsigned long>(uptimeMs), phase, raw,
        static_cast<unsigned long>(throttlePercent),
        static_cast<double>(filteredPercent), static_cast<double>(dutyPermille / 10.0f),
        relayOn ? "true" : "false");
    if (length < 0 || static_cast<size_t>(length) >= sizeof(payload)) {
        return ESP_ERR_INVALID_SIZE;
    }
    // Network transmission runs in the MQTT task, not the motor control loop.
    // QoS 0 requires store=true to enqueue; the outbox has a bounded size.
    return esp_mqtt_client_enqueue(client, kMotorTopic, payload, length, 0, 0, true) < 0
               ? ESP_FAIL : ESP_OK;
}

esp_err_t TelemetryLink::publishTelemetry(const TelemetryRecord &record,
                                           bool mpuAvailable,
                                           bool gpsAvailable,
                                           bool powerAvailable,
                                           bool eepromAvailable)
{
    const esp_mqtt_client_handle_t client = mqttClient_.load();
    if (client == nullptr || !mqttConnected_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    char payload[384] = {};
    char latitude[24] = {};
    char longitude[24] = {};
    char tilt[24] = {};
    char voltage[24] = {};
    char current[24] = {};
    const bool gpsFixValid = gpsAvailable && record.gps.valid;
    formatCoordinate(record.gps.latitudeDegrees, gpsFixValid,
                     latitude, sizeof(latitude));
    formatCoordinate(record.gps.longitudeDegrees, gpsFixValid,
                     longitude, sizeof(longitude));
    if (mpuAvailable) {
        std::snprintf(tilt, sizeof(tilt), "%.2f", static_cast<double>(record.tiltDegrees));
    } else {
        std::snprintf(tilt, sizeof(tilt), "null");
    }
    if (powerAvailable) {
        std::snprintf(voltage, sizeof(voltage), "%.3f", static_cast<double>(record.busVoltageVolts));
        std::snprintf(current, sizeof(current), "%.1f", static_cast<double>(record.currentMilliamps));
    } else {
        std::snprintf(voltage, sizeof(voltage), "null");
        std::snprintf(current, sizeof(current), "null");
    }
    const int payloadLength = std::snprintf(
        payload,
        sizeof(payload),
        "{\"uptime_ms\":%lu,\"mpu_available\":%s,\"tilt_deg\":%s,"
        "\"gps_available\":%s,\"gps_valid\":%s,\"power_available\":%s,"
        "\"eeprom_available\":%s,\"throttle_percent\":%lu,"
        "\"bus_voltage_v\":%s,\"current_ma\":%s,"
        "\"latitude_deg\":%s,\"longitude_deg\":%s,"
        "\"speed_mps\":%.2f,\"satellites\":%lu}",
        static_cast<unsigned long>(record.uptimeMilliseconds),
        mpuAvailable ? "true" : "false",
        tilt,
        gpsAvailable ? "true" : "false",
        gpsFixValid ? "true" : "false",
        powerAvailable ? "true" : "false",
        eepromAvailable ? "true" : "false",
        static_cast<unsigned long>(record.throttlePercent),
        voltage,
        current,
        latitude,
        longitude,
        static_cast<double>(record.gps.speedMetersPerSecond),
        static_cast<unsigned long>(record.gps.satellitesInView));
    if (payloadLength < 0 || static_cast<size_t>(payloadLength) >= sizeof(payload)) {
        return ESP_ERR_INVALID_SIZE;
    }

    const int messageId = esp_mqtt_client_enqueue(
        client, kTelemetryTopic, payload, payloadLength, 0, 0, false);
    return messageId < 0 ? ESP_FAIL : ESP_OK;
}

esp_err_t TelemetryLink::publishEvent(VehicleEvent event, const GpsFix &fix)
{
    const esp_mqtt_client_handle_t client = mqttClient_.load();
    if (client == nullptr || !mqttStarted_.load()) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *name = eventName(event);
    if (name == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    char payload[192] = {};
    char latitude[24] = {};
    char longitude[24] = {};
    formatCoordinate(fix.latitudeDegrees, fix.valid, latitude, sizeof(latitude));
    formatCoordinate(fix.longitudeDegrees, fix.valid, longitude, sizeof(longitude));
    const int payloadLength = std::snprintf(
        payload,
        sizeof(payload),
        "{\"event\":\"%s\",\"gps_valid\":%s,"
        "\"latitude_deg\":%s,\"longitude_deg\":%s}",
        name,
        fix.valid ? "true" : "false",
        latitude,
        longitude);
    if (payloadLength < 0 || static_cast<size_t>(payloadLength) >= sizeof(payload)) {
        return ESP_ERR_INVALID_SIZE;
    }

    const int messageId = esp_mqtt_client_enqueue(
        client, kEventTopic, payload, payloadLength, 1, 0, false);
    return messageId < 0 ? ESP_FAIL : ESP_OK;
}

void TelemetryLink::wifiEventHandler(
    void *argument,
    esp_event_base_t eventBase,
    int32_t eventId,
    void *eventData)
{
    auto *self = static_cast<TelemetryLink *>(argument);
    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
        const esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Wi-Fi connect request failed: %s", esp_err_to_name(err));
        }
        return;
    }
    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        self->mqttConnected_.store(false);
        const auto *event = static_cast<const wifi_event_sta_disconnected_t *>(eventData);
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%u; reconnecting",
                 event == nullptr ? 0U : static_cast<unsigned>(event->reason));
        const esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Wi-Fi reconnect request failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        const auto *event = static_cast<const ip_event_got_ip_t *>(eventData);
        ESP_LOGI(TAG, "Wi-Fi connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        if (self->brokerUri_[0] == '\0') {
            ESP_LOGI(TAG, "STEP 1 PASS: IP obtained; MQTT remains disabled");
            return;
        }
        const esp_err_t err = self->startMqttClient();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "MQTT client start failed: %s", esp_err_to_name(err));
        }
    }
}

esp_err_t TelemetryLink::startMqttClient()
{
    if (mqttStarted_.load()) {
        return ESP_OK;
    }

    esp_mqtt_client_config_t config = {};
    config.broker.address.uri = brokerUri_;
    config.outbox.limit = 4096;
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&config);
    if (client == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        client,
        MQTT_EVENT_ANY,
        &TelemetryLink::mqttEventHandler,
        this);
    if (err != ESP_OK) {
        esp_mqtt_client_destroy(client);
        return err;
    }
    err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
        esp_mqtt_client_destroy(client);
        return err;
    }

    mqttClient_.store(client);
    mqttStarted_.store(true);
    return ESP_OK;
}

void TelemetryLink::mqttEventHandler(
    void *argument,
    esp_event_base_t eventBase,
    int32_t eventId,
    void *eventData)
{
    auto *self = static_cast<TelemetryLink *>(argument);
    (void)eventBase;
    (void)eventData;
    if (eventId == MQTT_EVENT_CONNECTED) {
        self->mqttConnected_.store(true);
        ESP_LOGI(TAG, "Connected to MQTT broker");
    } else if (eventId == MQTT_EVENT_DISCONNECTED) {
        self->mqttConnected_.store(false);
        ESP_LOGW(TAG, "Disconnected from MQTT broker; client will reconnect");
    } else if (eventId == MQTT_EVENT_ERROR) {
        ESP_LOGE(TAG, "MQTT transport error");
    }
}
