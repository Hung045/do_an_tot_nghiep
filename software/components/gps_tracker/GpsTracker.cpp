#include "GpsTracker.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "BoardConfig.h"
#include "driver/uart.h"
#include "freertos/task.h"

namespace {

constexpr size_t kUartBufferSize = 1024;
constexpr TickType_t kFixMaximumAge = pdMS_TO_TICKS(5000);
constexpr float kKnotsToMetersPerSecond = 0.514444F;

bool parseCoordinate(const char *value, const char *hemisphere, double *degrees)
{
    if (value == nullptr || hemisphere == nullptr || degrees == nullptr ||
        value[0] == '\0') {
        return false;
    }

    char *end = nullptr;
    const double degreesAndMinutes = std::strtod(value, &end);
    if (end == value || *end != '\0') {
        return false;
    }

    const double wholeDegrees = static_cast<int>(degreesAndMinutes / 100.0);
    const double minutes = degreesAndMinutes - (wholeDegrees * 100.0);
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }

    double coordinate = wholeDegrees + (minutes / 60.0);
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        coordinate = -coordinate;
    } else if (hemisphere[0] != 'N' && hemisphere[0] != 'E') {
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
    if (end != checksumMarker + 3 || expected < 0 || expected > 0xFF) {
        return false;
    }

    uint8_t checksum = 0;
    for (char *cursor = sentence + 1; cursor < checksumMarker; ++cursor) {
        checksum ^= static_cast<uint8_t>(*cursor);
    }
    *checksumMarker = '\0';
    return checksum == static_cast<uint8_t>(expected);
}

} // namespace

esp_err_t GpsTracker::initialize()
{
    uart_config_t config = {};
    config.baud_rate = DEMO_GPS_BAUD_RATE;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(
        static_cast<uart_port_t>(DEMO_GPS_UART_PORT),
        &config);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_pin(
        static_cast<uart_port_t>(DEMO_GPS_UART_PORT),
        DEMO_GPS_UART_TX_GPIO,
        DEMO_GPS_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_driver_install(
        static_cast<uart_port_t>(DEMO_GPS_UART_PORT),
        kUartBufferSize,
        0,
        0,
        nullptr,
        0);
    if (err != ESP_OK) {
        return err;
    }

    lineLength_ = 0;
    hasFix_ = false;
    return ESP_OK;
}

esp_err_t GpsTracker::readFix(GpsFix *fix)
{
    if (fix == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t bytes[128];
    size_t totalRead = 0;
    while (totalRead < kUartBufferSize) {
        size_t bufferedBytes = 0;
        esp_err_t err = uart_get_buffered_data_len(
            static_cast<uart_port_t>(DEMO_GPS_UART_PORT),
            &bufferedBytes);
        if (err != ESP_OK) {
            return err;
        }
        if (bufferedBytes == 0U) {
            break;
        }

        const size_t bytesToRead =
            std::min(bufferedBytes, sizeof(bytes));
        const int bytesRead = uart_read_bytes(
            static_cast<uart_port_t>(DEMO_GPS_UART_PORT),
            bytes,
            bytesToRead,
            0);
        if (bytesRead < 0) {
            return ESP_FAIL;
        }
        if (bytesRead == 0) {
            break;
        }
        for (int i = 0; i < bytesRead; ++i) {
            consumeByte(bytes[i]);
        }
        totalRead += static_cast<size_t>(bytesRead);
    }

    const TickType_t now = xTaskGetTickCount();
    if (!hasFix_ || (now - lastFixAt_) > kFixMaximumAge) {
        *fix = {};
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
        if (lineLength_ > 1) {
            line_[lineLength_] = '\0';
            parseLine();
        }
        lineLength_ = 0;
        return;
    }

    if (lineLength_ + 1 < sizeof(line_)) {
        line_[lineLength_++] = static_cast<char>(byte);
    } else {
        lineLength_ = 0;
    }
}

void GpsTracker::parseLine()
{
    if (!verifyChecksum(line_)) {
        return;
    }

    char *fields[16] = {};
    size_t fieldCount = 1;
    fields[0] = line_ + 1;
    for (char *cursor = fields[0]; *cursor != '\0'; ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            if (fieldCount == (sizeof(fields) / sizeof(fields[0]))) {
                return;
            }
            fields[fieldCount++] = cursor + 1;
        }
    }

    if (std::strcmp(fields[0], "GPGGA") == 0 ||
        std::strcmp(fields[0], "GNGGA") == 0) {
        if (fieldCount > 7) {
            latestFix_.satellitesInView =
                static_cast<uint32_t>(std::strtoul(fields[7], nullptr, 10));
        }
        return;
    }

    if ((std::strcmp(fields[0], "GPRMC") != 0 &&
         std::strcmp(fields[0], "GNRMC") != 0) ||
        fieldCount < 9) {
        return;
    }

    if (fields[2][0] != 'A') {
        hasFix_ = false;
        return;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    if (!parseCoordinate(fields[3], fields[4], &latitude) ||
        !parseCoordinate(fields[5], fields[6], &longitude) ||
        latitude < -90.0 || latitude > 90.0 ||
        longitude < -180.0 || longitude > 180.0) {
        return;
    }

    latestFix_.latitudeDegrees = latitude;
    latestFix_.longitudeDegrees = longitude;
    latestFix_.speedMetersPerSecond =
        static_cast<float>(std::strtod(fields[7], nullptr)) * kKnotsToMetersPerSecond;
    latestFix_.valid = true;
    lastFixAt_ = xTaskGetTickCount();
    hasFix_ = true;
}
