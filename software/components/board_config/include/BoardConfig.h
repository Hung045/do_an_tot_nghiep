#pragma once

#include "driver/gpio.h"

// ============================================================================
// 1. CẤU HÌNH CHÂN GPIO (ESP32 DOIT DevKit V1 - Bản 30 chân)
// ============================================================================
// Tay ga dùng GPIO34 (ADC1_CH6) - Chân Input-Only, không bị xung đột khi bật Wi-Fi/ESP-NOW
#define DEMO_THROTTLE_ADC_CHANNEL ADC_CHANNEL_6
#define DEMO_THROTTLE_GPIO        GPIO_NUM_34

// Khối chấp hành: Băm xung MOSFET LR7843 (Động cơ JGB37-310 & Đèn pha LED 12V) + Relay 5V
#define DEMO_MOTOR_PWM_GPIO       GPIO_NUM_25
#define DEMO_HEADLIGHT_PWM_GPIO   GPIO_NUM_26
#define DEMO_RELAY_GPIO           GPIO_NUM_27

// Bus I2C dùng chung cho 3 module: MPU6050 (GY-521), EEPROM AT24C256, INA226
#define DEMO_I2C_SDA_GPIO         GPIO_NUM_21
#define DEMO_I2C_SCL_GPIO         GPIO_NUM_22

// Định vị vệ tinh GPS Ublox NEO-6M (GY-GPS6MV2) giao tiếp qua UART2
#define DEMO_GPS_UART_PORT        2
#define DEMO_GPS_UART_TX_GPIO     GPIO_NUM_17
#define DEMO_GPS_UART_RX_GPIO     GPIO_NUM_16
#define DEMO_GPS_BAUD_RATE        9600

// ============================================================================
// 2. THÔNG SỐ GIAO TIẾP I2C & ĐỊA CHỈ NGOẠI VI
// ============================================================================
// Tốc độ 400kHz (Fast Mode) giúp giảm thời gian quét 32KB EEPROM từ ~3.1s xuống ~0.9s.
// Lưu ý: Nếu dây cắm testboard dài (>20cm) bị nhiễu, có thể hạ về 100000 (100kHz).
#define DEMO_I2C_FREQUENCY_HZ            100000

#define DEMO_MPU6050_ADDRESS             0x68
#define DEMO_INA226_ADDRESS              0x40
#define DEMO_AT24C256_ADDRESS            0x50

// Cấu hình INA226 bản 36V/20A (Điện trở Shunt R002 = 0.002 Ohm = 2000 micro-Ohm)
#define DEMO_INA226_SHUNT_MICRO_OHMS     2000
#define DEMO_INA226_MAX_CURRENT_MILLIAMPS 20000

// Cấu hình bộ nhớ hộp đen EEPROM AT24C256 (32KB = 512 bản ghi x 64 Bytes/Page)
#define DEMO_AT24C256_CAPACITY_BYTES     32768
#define DEMO_AT24C256_PAGE_SIZE_BYTES    64
#define DEMO_AT24C256_ADDRESS_BYTES      2

// ============================================================================
// 3. THÔNG SỐ NGUỒN PIN 18650 3S & CƠ CẤU CHẤP HÀNH
// ============================================================================
// Hệ pin 3S 18650 (10800mAh): Điện áp danh định 11.1V, sạc đầy tự ngắt 12.6V, cạn 9.6V
#define DEMO_BATTERY_SERIES_CELLS        3
#define DEMO_BATTERY_EMPTY_MILLIVOLTS    9600
#define DEMO_BATTERY_NOMINAL_MILLIVOLTS  11100
#define DEMO_BATTERY_FULL_MILLIVOLTS     12600

// Module Relay 1 kênh 5V có cách ly quang (Opto):
// Đặt là 1 nếu kích mức Cao (Active-High), đổi thành 0 nếu kích mức Thấp (Active-Low).
#define DEMO_RELAY_ACTIVE_LEVEL          1

// Tần số PWM 20kHz (vượt ngưỡng nghe của tai người, giúp MOSFET LR7843 và động cơ không kêu rít)
#define DEMO_PWM_FREQUENCY_HZ            20000
#define DEMO_PWM_RESOLUTION_BITS         10

// ============================================================================
// 4. QUẢN LÝ ĐỒNG BỘ ĐA LUỒNG I2C (THREAD-SAFE I2C BUS GUARD CHO FREERTOS)
// ============================================================================
#ifdef __cplusplus
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace BoardI2c {

/**
 * @brief Khởi tạo Driver I2C_NUM_0 và tạo Mutex duy nhất 1 lần cho toàn hệ thống.
 * Giải quyết triệt để lỗi "E i2c: i2c driver install error" khi nhiều module cùng khởi tạo
 * và kích hoạt cơ chế Priority Inheritance chống đảo ngược độ ưu tiên giữa các Task.
 */
inline SemaphoreHandle_t getMutex()
{
    static SemaphoreHandle_t mutex = nullptr;
    static bool driverInstalled = false;

    // 1. Tạo Mutex an toàn bằng vùng cấm ngắt (Critical Section) chống Race Condition lúc khởi động
    static portMUX_TYPE initSpinlock = portMUX_INITIALIZER_UNLOCKED;
    if (mutex == nullptr) {
        portENTER_CRITICAL(&initSpinlock);
        if (mutex == nullptr) {
            mutex = xSemaphoreCreateMutex();
        }
        portEXIT_CRITICAL(&initSpinlock);
    }

    // 2. Cài đặt I2C Driver đúng 1 lần duy nhất
    if (!driverInstalled && mutex != nullptr) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (!driverInstalled) {
                i2c_config_t config = {};
                config.mode = I2C_MODE_MASTER;
                config.sda_io_num = DEMO_I2C_SDA_GPIO;
                config.scl_io_num = DEMO_I2C_SCL_GPIO;
                config.sda_pullup_en = GPIO_PULLUP_ENABLE;
                config.scl_pullup_en = GPIO_PULLUP_ENABLE;
                config.master.clk_speed = DEMO_I2C_FREQUENCY_HZ;

                if (i2c_param_config(I2C_NUM_0, &config) == ESP_OK) {
                    // Nếu một module khác (như PowerMonitor) đã lỡ gọi i2c_driver_install trước đó,
                    // vẫn đánh dấu driverInstalled = true để không gọi lặp lại.
                    esp_err_t err = i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0);
                    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
                        driverInstalled = true;
                    }
                }
            }
            xSemaphoreGive(mutex);
        }
    }
    return mutex;
}

/**
 * @brief Khóa RAII (Resource Acquisition Is Initialization):
 * Tự động gọi xSemaphoreTake khi khai báo biến và tự động gọi xSemaphoreGive
 * ngay khi thoát khỏi hàm (kể cả khi return sớm do lỗi I2C), chống treo bus (Deadlock).
 */
class LockGuard {
public:
    explicit LockGuard(TickType_t timeoutTicks = pdMS_TO_TICKS(200))
        : mutex_(getMutex()), locked_(false)
    {
        if (mutex_ != nullptr) {
            locked_ = (xSemaphoreTake(mutex_, timeoutTicks) == pdTRUE);
        }
    }

    ~LockGuard()
    {
        if (locked_ && mutex_ != nullptr) {
            xSemaphoreGive(mutex_);
        }
    }

    // Cấm sao chép khóa để tránh nhả Mutex 2 lần
    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

    bool isLocked() const { return locked_; }

private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

} // namespace BoardI2c
#endif