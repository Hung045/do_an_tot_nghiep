#pragma once

#include "driver/gpio.h"

#define DEMO_THROTTLE_ADC_CHANNEL    ADC_CHANNEL_6
#define DEMO_THROTTLE_GPIO           GPIO_NUM_34

#define DEMO_THROTTLE_ADC_MIN        0
#define DEMO_THROTTLE_ADC_MAX        4095

#define DEMO_MOTOR_PWM_GPIO          GPIO_NUM_25
#define DEMO_HEADLIGHT_PWM_GPIO      GPIO_NUM_26
#define DEMO_RELAY_GPIO              GPIO_NUM_27

#define DEMO_I2C_SDA_GPIO            GPIO_NUM_21
#define DEMO_I2C_SCL_GPIO            GPIO_NUM_22

#define DEMO_GPS_UART_PORT           2
#define DEMO_GPS_UART_TX_GPIO        GPIO_NUM_17
#define DEMO_GPS_UART_RX_GPIO        GPIO_NUM_16
#define DEMO_GPS_BAUD_RATE           9600

#define DEMO_I2C_FREQUENCY_HZ        100000

#define DEMO_BH1750_ADDRESS          0x23
#define DEMO_MPU6050_ADDRESS         0x68
#define DEMO_INA226_ADDRESS          0x40
#define DEMO_AT24C256_ADDRESS        0x50

#define DEMO_INA226_SHUNT_MICRO_OHMS 2000
#define DEMO_INA226_MAX_CURRENT_MILLIAMPS 20000

#define DEMO_AT24C256_CAPACITY_BYTES     32768
#define DEMO_AT24C256_PAGE_SIZE_BYTES    64
#define DEMO_AT24C256_ADDRESS_BYTES      2

#define DEMO_BATTERY_SERIES_CELLS        3
#define DEMO_BATTERY_EMPTY_MILLIVOLTS    9600
#define DEMO_BATTERY_NOMINAL_MILLIVOLTS  11100
#define DEMO_BATTERY_FULL_MILLIVOLTS     12600

#define DEMO_RELAY_ACTIVE_LEVEL          0

#define DEMO_PWM_FREQUENCY_HZ            20000
#define DEMO_PWM_RESOLUTION_BITS         10

#ifdef __cplusplus
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace BoardI2c {

SemaphoreHandle_t getMutex();

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

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

    bool isLocked() const { return locked_; }

private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

} // namespace BoardI2c
#endif
