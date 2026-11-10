#pragma once

#include "driver/gpio.h"

#define DEMO_THROTTLE_ADC_CHANNEL    ADC_CHANNEL_6
#define DEMO_THROTTLE_GPIO           GPIO_NUM_34

#define DEMO_THROTTLE_ADC_MIN        0
#define DEMO_THROTTLE_ADC_MAX        4095

#define DEMO_MOTOR_PWM_GPIO          GPIO_NUM_25
#define DEMO_HEADLIGHT_PWM_GPIO      GPIO_NUM_26
#define DEMO_RELAY_GPIO              GPIO_NUM_27

// Positional SG90 (not continuous rotation), independent LEDC timer at 50 Hz.
#define DEMO_HEADLIGHT_SERVO_GPIO     GPIO_NUM_23
#define DEMO_SERVO_CENTER_DEG         90.0F
#define DEMO_SERVO_MIN_DEG            60.0F
#define DEMO_SERVO_MAX_DEG            120.0F
#define DEMO_SERVO_MIN_PULSE_US       1000U
#define DEMO_SERVO_MAX_PULSE_US       2000U
#define DEMO_SERVO_GAIN               1.0F
#define DEMO_SERVO_DIRECTION          1
#define DEMO_SERVO_ROLL_ZERO_DEG      0.0F
#define DEMO_SERVO_DEADBAND_DEG       3.0F
#define DEMO_SERVO_RATE_DEG_PER_SEC   60.0F
#define DEMO_SERVO_FILTER_MS          150.0F
// MPU mounting: X forward, Y lateral, Z up. Set 0 to use X as lateral axis.
#define DEMO_MPU_ROLL_USE_Y           1

// MPU6050 has a dedicated hardware bus; slow peripherals share I2C1.
#define DEMO_MPU_I2C_PORT            I2C_NUM_0
#define DEMO_MPU_I2C_SDA_GPIO        GPIO_NUM_21
#define DEMO_MPU_I2C_SCL_GPIO        GPIO_NUM_22
#define DEMO_AUX_I2C_PORT            I2C_NUM_1
#define DEMO_AUX_I2C_SDA_GPIO        GPIO_NUM_18
#define DEMO_AUX_I2C_SCL_GPIO        GPIO_NUM_19

#define DEMO_GPS_UART_PORT           2
#define DEMO_GPS_UART_TX_GPIO        GPIO_NUM_17
#define DEMO_GPS_UART_RX_GPIO        GPIO_NUM_16
#define DEMO_GPS_BAUD_RATE           9600

#define DEMO_I2C_FREQUENCY_HZ        100000

#define DEMO_BH1750_ADDRESS          0x23
#define DEMO_MPU6050_ADDRESS         0x68
#define DEMO_INA226_ADDRESS          0x40
#define DEMO_AT24C256_ADDRESS        0x50

// R020 = 20 mOhm. 4 A produces 80 mV (INA226 shunt range: ~81.92 mV).
// Measurement scaling only: this does not limit motor current.
#define DEMO_INA226_SHUNT_MICRO_OHMS 20000
#define DEMO_INA226_MAX_CURRENT_MILLIAMPS 4000

#define DEMO_AT24C256_CAPACITY_BYTES     32768
#define DEMO_AT24C256_PAGE_SIZE_BYTES    64
#define DEMO_AT24C256_ADDRESS_BYTES      2

#define DEMO_BATTERY_SERIES_CELLS        3
#define DEMO_BATTERY_EMPTY_MILLIVOLTS    9600
#define DEMO_BATTERY_NOMINAL_MILLIVOLTS  11100
#define DEMO_BATTERY_FULL_MILLIVOLTS     12600

// Relay module is active-low; verify against the actual module before use.
#define DEMO_RELAY_ACTIVE_LEVEL          0

// Diagnostic frequency for the LR7843 module; confirm output on hardware.
#define DEMO_PWM_FREQUENCY_HZ            1000
#define DEMO_PWM_RESOLUTION_BITS         10

#ifdef __cplusplus
#include <cstddef>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace BoardI2c {

// Initialize before using a bus. Failed initialization can be retried.
esp_err_t initialize(i2c_port_t port);
// Caller must hold this bus's LockGuard. Timeouts are FreeRTOS ticks.
esp_err_t write(i2c_port_t port, uint8_t address, const uint8_t *data, size_t size, TickType_t timeout);
esp_err_t read(i2c_port_t port, uint8_t address, uint8_t *data, size_t size, TickType_t timeout);
esp_err_t writeRead(i2c_port_t port, uint8_t address, const uint8_t *tx, size_t txSize,
                    uint8_t *rx, size_t rxSize, TickType_t timeout);
// Returns only the port's mutex; does not initialize or lock the bus.
SemaphoreHandle_t getMutex(i2c_port_t port);

class LockGuard {
public:
    explicit LockGuard(i2c_port_t port, TickType_t timeoutTicks = pdMS_TO_TICKS(200))
        : mutex_(getMutex(port)), locked_(false)
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

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

    bool isLocked() const { return locked_; }

private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

} // namespace BoardI2c
#endif
