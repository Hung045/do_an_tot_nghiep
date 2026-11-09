#include "GpsTracker.h"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdlib>
#include <cstring>

#include "BoardConfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"

namespace {

constexpr size_t kUartBufferSize = 1024;
constexpr TickType_t kFixMaximumAge = pdMS_TO_TICKS(5000);
constexpr float kKnotsToMetersPerSecond = 0.514444F;
constexpr float kMinValidSpeedMps = 0.30F;

bool parseCoordinate(const char *value, const char *hemisphere, double *degrees)
{
    if (value == nullptr || hemisphere == nullptr || degrees == nullptr ||
        value[0] == '\0') {
        return false;
    }

    char *end = nullptr;
    const double degreesAndMinutes = std::strtod(value, &end);
    if (end == value || *end != '\0' || !std::isfinite(degreesAndMinutes) ||
        degreesAndMinutes < 0.0 || degreesAndMinutes > 18000.0 || hemisphere[1] != '\0') {
        return false;
    }

    const int wholeDegrees = static_cast<int>(degreesAndMinutes / 100.0);
    const double minutes = degreesAndMinutes - (static_cast<double>(wholeDegrees) * 100.0);
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }

    double coordinate = static_cast<double>(wholeDegrees) + (minutes / 60.0);
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        coordinate = -coordinate;
    } else if (hemisphere[0] != 'N' && hemisphere[0] != 'E') {
        return false;
    }

    if ( (hemisphere[0] == 'N' || hemisphere[0] == 'S') && (coordinate < -90.0 || coordinate > 90.0) ) {
        return false;
    }
    if ( (hemisphere[0] == 'E' || hemisphere[0] == 'W') && (coordinate < -180.0 || coordinate > 180.0) ) {
        return false;
    }

    *degrees = coordinate;
    return true;
}

bool verifyChecksum(char *sentence)
{
    if (sentence == nullptr || sentence[0] != '$') {
        return false;
    }

    char *checksumMarker = std::strchr(sentence, '*');
    if (checksumMarker == nullptr || checksumMarker[1] == '\0' ||
        checksumMarker[2] == '\0') {
        return false;
    }

    char *end = nullptr;
    const long expected = std::strtol(checksumMarker + 1, &end, 16);
    if (end != checksumMarker + 3 || *end != '\0' || expected < 0 || expected > 0xFF) {
        return false;
    }

    uint8_t checksum = 0;
    for (char *cursor = sentence + 1; cursor < checksumMarker; ++cursor) {
        checksum ^= static_cast<uint8_t>(*cursor);
    }
    *checksumMarker = '\0';
    return checksum == static_cast<uint8_t>(expected);
}

}

esp_err_t GpsTracker::initialize()
{
    initialized_ = false;
    uartError_ = false;
    receivedAny_ = receivedNmea_ = receivedNavigation_ = receivedGga_ = false;
    diagnostics_ = {};
    const auto uartPort = static_cast<uart_port_t>(DEMO_GPS_UART_PORT);

    uart_config_t config = {};
    config.baud_rate = DEMO_GPS_BAUD_RATE;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(uartPort, &config);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_pin(
        uartPort,
        DEMO_GPS_UART_TX_GPIO,
        DEMO_GPS_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    if (!uart_is_driver_installed(uartPort)) {
        err = uart_driver_install(
            uartPort,
            kUartBufferSize * 2U,
            0,
            0,
            nullptr,
            0);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        uart_flush_input(uartPort);
    }

    lineLength_ = 0;
    hasFix_ = false;
    latestFix_ = {};
    lastFixAt_ = lastSpeedAt_ = 0;
    initialized_ = true;
    ESP_LOGI("GpsTracker", "UART%d RX=%d <- GPS TX, TX=%d -> GPS RX, %d baud; waiting for NMEA",
             DEMO_GPS_UART_PORT, DEMO_GPS_UART_RX_GPIO, DEMO_GPS_UART_TX_GPIO, DEMO_GPS_BAUD_RATE);
    return ESP_OK;
}

GpsDiagnostics GpsTracker::diagnostics() const
{
    GpsDiagnostics result = diagnostics_;
    const TickType_t now = xTaskGetTickCount();
    const auto age = [now](TickType_t at) -> int32_t {
        const uint64_t ms = static_cast<uint64_t>(static_cast<TickType_t>(now - at)) * portTICK_PERIOD_MS;
        return static_cast<int32_t>(std::min<uint64_t>(ms, INT32_MAX));
    };
    result.rxAgeMs = receivedAny_ ? age(lastRxAt_) : -1;
    result.nmeaAgeMs = receivedNmea_ ? age(lastNmeaAt_) : -1;
    if (!initialized_) result.status = GpsStatus::InitError;
    else if (uartError_) result.status = GpsStatus::UartError;
    else if (!receivedAny_) result.status = GpsStatus::NoData;
    else if (static_cast<TickType_t>(now - lastRxAt_) >= kFixMaximumAge) result.status = GpsStatus::Stale;
    else if (!receivedNmea_ || static_cast<TickType_t>(now - lastNmeaAt_) >= kFixMaximumAge)
        result.status = GpsStatus::InvalidNmea;
    else if (!receivedNavigation_ || static_cast<TickType_t>(now - lastNavigationAt_) >= kFixMaximumAge)
        result.status = GpsStatus::NoNavigation;
    else if (hasFix_ && static_cast<TickType_t>(now - lastFixAt_) < kFixMaximumAge)
        result.status = GpsStatus::FixOk;
    else result.status = GpsStatus::NoFix;
    return result;
}

esp_err_t GpsTracker::readFix(GpsFix *fix)
{
    if (fix == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *fix = {};
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    uartError_ = false;

    const auto uartPort = static_cast<uart_port_t>(DEMO_GPS_UART_PORT);
    uint8_t bytes[128];
    size_t totalRead = 0;
    constexpr size_t kMaxDrainBytes = kUartBufferSize * 2U;

    while (totalRead < kMaxDrainBytes) {
        size_t bufferedBytes = 0;
        esp_err_t err = uart_get_buffered_data_len(uartPort, &bufferedBytes);
        if (err != ESP_OK) {
            uartError_ = true;
            return err;
        }
        if (bufferedBytes == 0U) {
            break;
        }

        const size_t bytesToRead = std::min(bufferedBytes, sizeof(bytes));
        const int bytesRead = uart_read_bytes(uartPort, bytes, bytesToRead, 0);
        if (bytesRead < 0) {
            uartError_ = true;
            return ESP_FAIL;
        }
        if (bytesRead == 0) {
            break;
        }
        receivedAny_ = true;
        lastRxAt_ = xTaskGetTickCount();
        diagnostics_.receivedBytes += static_cast<uint32_t>(bytesRead);
        for (int i = 0; i < bytesRead; ++i) {
            consumeByte(bytes[i]);
        }
        totalRead += static_cast<size_t>(bytesRead);
    }

    const TickType_t now = xTaskGetTickCount();
    if (!receivedGga_ || static_cast<TickType_t>(now - lastGgaAt_) >= kFixMaximumAge)
        latestFix_.satellitesInView = 0;
    if (static_cast<TickType_t>(now - lastSpeedAt_) >= kFixMaximumAge)
        latestFix_.speedMetersPerSecond = 0;
    if (!hasFix_ || static_cast<TickType_t>(now - lastFixAt_) >= kFixMaximumAge) {
        hasFix_ = false;
        latestFix_.valid = false;
        *fix = {};
        fix->satellitesInView = latestFix_.satellitesInView;
        return ESP_ERR_NOT_FOUND;
    }

    *fix = latestFix_;
    return ESP_OK;
}

void GpsTracker::consumeByte(uint8_t byte)
{
    if (byte == '$') {
        lineLength_ = 0;
        line_[lineLength_++] = '$';
        return;
    }

    if (lineLength_ == 0) {
        return;
    }

    if (byte == '\n' || byte == '\r') {
        if (lineLength_ > 6) {
            line_[lineLength_] = '\0';
            parseLine();
        }
        lineLength_ = 0;
        return;
    }

    if (lineLength_ + 1 < sizeof(line_)) {
        line_[lineLength_++] = static_cast<char>(byte);
    } else {
        diagnostics_.rejectedSentences++;
        lineLength_ = 0;
    }
}

void GpsTracker::parseLine()
{
    if (!verifyChecksum(line_)) {
        diagnostics_.rejectedSentences++;
        return;
    }
    diagnostics_.validSentences++;
    receivedNmea_ = true;
    lastNmeaAt_ = xTaskGetTickCount();

    char *fields[24] = {};
    size_t fieldCount = 1;
    fields[0] = line_ + 1;
    for (char *cursor = fields[0]; *cursor != '\0'; ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            if (fieldCount < (sizeof(fields) / sizeof(fields[0]))) {
                fields[fieldCount++] = cursor + 1;
            }
        }
    }

    if (std::strcmp(fields[0], "GPGGA") == 0 ||
        std::strcmp(fields[0], "GNGGA") == 0) {
        if (fieldCount > 7) {
            // Only well-formed quality/satellite fields count as navigation data.
            char *qualityEnd = nullptr;
            char *satEnd = nullptr;
            const long quality = std::strtol(fields[6], &qualityEnd, 10);
            const long satellites = std::strtol(fields[7], &satEnd, 10);
            if (qualityEnd == fields[6] || *qualityEnd || quality < 0 || quality > 8 ||
                satEnd == fields[7] || *satEnd || satellites < 0 || satellites > 99) return;
            receivedNavigation_ = receivedGga_ = true;
            lastNavigationAt_ = lastGgaAt_ = xTaskGetTickCount();
            latestFix_.satellitesInView =
                static_cast<uint32_t>(satellites);

            const uint32_t fixQuality = static_cast<uint32_t>(quality);
            if (fixQuality == 0U) {
                hasFix_ = false;
                latestFix_.valid = false;
                latestFix_.speedMetersPerSecond = 0;
            }
            if (fixQuality > 0U) {
                double latitude = 0.0;
                double longitude = 0.0;
                if (parseCoordinate(fields[2], fields[3], &latitude) &&
                    parseCoordinate(fields[4], fields[5], &longitude)) {
                    latestFix_.latitudeDegrees = latitude;
                    latestFix_.longitudeDegrees = longitude;
                    latestFix_.valid = true;
                    lastFixAt_ = xTaskGetTickCount();
                    hasFix_ = true;
                }
            }
        }
        return;
    }

    if ((std::strcmp(fields[0], "GPRMC") == 0 ||
         std::strcmp(fields[0], "GNRMC") == 0) &&
        fieldCount >= 9) {

        if (std::strcmp(fields[2], "A") != 0 && std::strcmp(fields[2], "V") != 0) return;
        receivedNavigation_ = true;
        lastNavigationAt_ = xTaskGetTickCount();
        if (fields[2][0] != 'A') {
            hasFix_ = false;
            latestFix_.valid = false;
            return;
        }

        double latitude = 0.0;
        double longitude = 0.0;
        if (!parseCoordinate(fields[3], fields[4], &latitude) ||
            !parseCoordinate(fields[5], fields[6], &longitude)) {
            return;
        }

        char *speedEnd = nullptr;
        const double knots = std::strtod(fields[7], &speedEnd);
        const float rawSpeedMps = static_cast<float>(knots * kKnotsToMetersPerSecond);
        const bool speedValid = speedEnd != fields[7] && *speedEnd == '\0' &&
                                std::isfinite(rawSpeedMps) && rawSpeedMps >= 0.0F;

        latestFix_.latitudeDegrees = latitude;
        latestFix_.longitudeDegrees = longitude;
        latestFix_.speedMetersPerSecond = (speedValid && rawSpeedMps >= kMinValidSpeedMps) ? rawSpeedMps : 0.0F;
        lastSpeedAt_ = xTaskGetTickCount();
        latestFix_.valid = true;
        lastFixAt_ = xTaskGetTickCount();
        hasFix_ = true;
    }
}
