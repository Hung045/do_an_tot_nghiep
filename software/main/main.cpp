#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "CrashDetector.h"
#include "DataLogger.h"
#include "GpsTracker.h"
#include "ProjectTypes.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "my_3_modules_test";
constexpr uint32_t kLoopPeriodMs = 50;          
constexpr uint32_t kTelemetryPeriodMs = 1000;    

CrashDetector crashDetector;
DataLogger dataLogger;
GpsTracker gpsTracker;

void latchSafetyFault(bool *faultLatched, const char *reason)
{
    if (*faultLatched) {
        return;
    }

    *faultLatched = true;
    ESP_LOGE(TAG, "SAFETY LOCK kích hoạt: %s. (Nếu có động cơ, Relay sẽ bị ngắt tại đây!)", reason);
}

void controlTask(void *)
{
    esp_err_t err;

    err = crashDetector.initialize();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Crash detector (MPU6050) unavailable: %s", esp_err_to_name(err));
    }

    err = gpsTracker.initialize();
    const bool gpsTrackerReady = (err == ESP_OK);
    if (!gpsTrackerReady) {
        ESP_LOGW(TAG, "NEO-6M unavailable: %s", esp_err_to_name(err));
    }

    err = dataLogger.initialize();
    const bool dataLoggerReady = (err == ESP_OK);
    if (!dataLoggerReady) {
        ESP_LOGW(TAG, "AT24C256 unavailable: %s", esp_err_to_name(err));
    } else {
        TelemetryRecord lastSaved = {};
        if (dataLogger.readLatest(&lastSaved) == ESP_OK) {
            ESP_LOGI(TAG, "[EEPROM Khôi phục] Uptime: %lu ms | Tilt: %.1f deg | GPS: %s (Sat=%lu)",
                     static_cast<unsigned long>(lastSaved.uptimeMilliseconds),
                     static_cast<double>(lastSaved.tiltDegrees),
                     lastSaved.gps.valid ? "VALID" : "NO-FIX",
                     static_cast<unsigned long>(lastSaved.gps.satellitesInView));
        }
    }

    ESP_LOGI(TAG, "=== SẴN SÀNG TEST 3 MODULE (CRASH, LOGGER, GPS) ===");

    bool faultLatched = false;
    TickType_t lastStatusAt = xTaskGetTickCount();
    TickType_t lastTelemetryAt = lastStatusAt;
    GpsFix gpsFix = {};

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = 0.0F;
        bool crashDetected = false;

        err = crashDetector.update(&tiltDegrees, &crashDetected);
        if (err != ESP_OK) {
            latchSafetyFault(&faultLatched, "MPU6050 rớt kết nối I2C");
        } else if (crashDetected) {
            latchSafetyFault(&faultLatched, "Xe nghiêng vượt ngưỡng 60 độ!");
        }

        if ((now - lastStatusAt) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "Góc nghiêng: %.1f deg | Trạng thái: %s",
                     static_cast<double>(tiltDegrees),
                     faultLatched ? "LOCKED (Bị ngã/Lỗi)" : "BÌNH THƯỜNG");
            lastStatusAt = now;
        }

        if ((now - lastTelemetryAt) >= pdMS_TO_TICKS(kTelemetryPeriodMs)) {
            if (gpsTrackerReady) {
                err = gpsTracker.readFix(&gpsFix);
                if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                    ESP_LOGW(TAG, "Lỗi đọc GPS: %s", esp_err_to_name(err));
                    gpsFix = {};
                }
            }

            TelemetryRecord record = {};
            record.uptimeMilliseconds = static_cast<uint32_t>(now * portTICK_PERIOD_MS);
            record.tiltDegrees = tiltDegrees;
            record.throttlePercent = 0;       // Giá trị giả lập tay ga
            record.busVoltageVolts = 12.0F;   // Giá trị giả lập điện áp pin 3S
            record.currentMilliamps = 0.0F;   // Giá trị giả lập dòng điện
            record.gps = gpsFix;

            bool eepromWriteOk = false;
            if (dataLoggerReady) {
                err = dataLogger.append(record);
                if (err == ESP_OK) {
                    eepromWriteOk = true;
                } else {
                    ESP_LOGE(TAG, "Lỗi ghi EEPROM: %s", esp_err_to_name(err));
                }
            }

            const char *eepromStatus = !dataLoggerReady ? "Chưa cắm EEPROM"
                                                        : (eepromWriteOk ? "Đã ghi EEPROM" : "Ghi EEPROM lỗi");

            if (record.gps.valid) {
                ESP_LOGI(TAG, "-> %s | GPS BẮT ĐƯỢC (Sat=%lu, Lat=%.6f, Lon=%.6f, Spd=%.1f m/s)",
                         eepromStatus,
                         static_cast<unsigned long>(record.gps.satellitesInView),
                         record.gps.latitudeDegrees,
                         record.gps.longitudeDegrees,
                         static_cast<double>(record.gps.speedMetersPerSecond));
            } else {
                ESP_LOGI(TAG, "-> %s | GPS ĐANG TÌM SÓNG (Sat=%lu)",
                         eepromStatus,
                         static_cast<unsigned long>(record.gps.satellitesInView));
            }

            lastTelemetryAt = now;
        }

        vTaskDelay(pdMS_TO_TICKS(kLoopPeriodMs));
    }
}

} 

extern "C" void app_main(void)
{
    if (xTaskCreate(controlTask, "test_my_modules", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Không thể tạo Task!");
    }
}