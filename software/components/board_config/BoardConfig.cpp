#include "BoardConfig.h"

#include <atomic>
#include <algorithm>
#include "esp_log.h"
#include "freertos/task.h"

namespace BoardI2c {
namespace {

struct BusState {
    std::atomic<SemaphoreHandle_t> mutex{nullptr};
    i2c_master_bus_handle_t handle = nullptr; // All state protected by bus mutex.
    i2c_master_dev_handle_t devices[128] = {};
    bool coolingDown = false;
    TickType_t failedAt = 0;
};

BusState buses[2];
static_assert(DEMO_MPU_I2C_PORT == I2C_NUM_0 && DEMO_AUX_I2C_PORT == I2C_NUM_1);
constexpr gpio_num_t assignedPins[] = {
    DEMO_MPU_I2C_SDA_GPIO, DEMO_MPU_I2C_SCL_GPIO,
    DEMO_AUX_I2C_SDA_GPIO, DEMO_AUX_I2C_SCL_GPIO,
    DEMO_THROTTLE_GPIO, DEMO_MOTOR_PWM_GPIO, DEMO_HEADLIGHT_PWM_GPIO,
    DEMO_RELAY_GPIO, DEMO_GPS_UART_TX_GPIO, DEMO_GPS_UART_RX_GPIO,
    DEMO_HEADLIGHT_SERVO_GPIO
};
constexpr bool pinsAreUnique()
{
    for (unsigned i = 0; i < sizeof(assignedPins) / sizeof(assignedPins[0]); ++i) {
        for (unsigned j = i + 1; j < sizeof(assignedPins) / sizeof(assignedPins[0]); ++j) {
            if (assignedPins[i] == assignedPins[j]) return false;
        }
    }
    return true;
}
static_assert(pinsAreUnique(), "I2C pins conflict with another peripheral");

bool validPort(i2c_port_t port)
{
    return port == DEMO_MPU_I2C_PORT || port == DEMO_AUX_I2C_PORT;
}

} // namespace

SemaphoreHandle_t getMutex(i2c_port_t port)
{
    if (!validPort(port)) return nullptr;
    auto &slot = buses[static_cast<unsigned>(port)].mutex;
    SemaphoreHandle_t mutex = slot.load();
    if (mutex == nullptr) {
        // Allocate outside a critical section. If another task wins the race,
        // discard our unused mutex and use the already-published one.
        SemaphoreHandle_t candidate = xSemaphoreCreateMutex();
        if (candidate == nullptr) return nullptr;
        if (slot.compare_exchange_strong(mutex, candidate)) {
            mutex = candidate;
        } else {
            vSemaphoreDelete(candidate);
        }
    }
    return mutex;
}

esp_err_t initialize(i2c_port_t port)
{
    if (!validPort(port)) return ESP_ERR_INVALID_ARG;
    if (getMutex(port) == nullptr) return ESP_ERR_NO_MEM;
    LockGuard lock(port, pdMS_TO_TICKS(500));
    if (!lock.isLocked()) return ESP_ERR_TIMEOUT;
    auto &bus = buses[static_cast<unsigned>(port)];
    if (bus.handle != nullptr) return ESP_OK;

    const bool isMpu = port == DEMO_MPU_I2C_PORT;
    i2c_master_bus_config_t config = {};
    config.i2c_port = port;
    config.sda_io_num = isMpu ? DEMO_MPU_I2C_SDA_GPIO : DEMO_AUX_I2C_SDA_GPIO;
    config.scl_io_num = isMpu ? DEMO_MPU_I2C_SCL_GPIO : DEMO_AUX_I2C_SCL_GPIO;
    config.clk_source = I2C_CLK_SRC_DEFAULT;
    config.glitch_ignore_cnt = 7;
    config.flags.enable_internal_pullup = true;

    const esp_err_t err = i2c_new_master_bus(&config, &bus.handle);
    if (err != ESP_OK) {
        // Do not hide a failed/foreign driver installation or mark it ready.
        ESP_LOGE("BoardI2c", "I2C%d init failed: %s", static_cast<int>(port), esp_err_to_name(err));
        return err;
    }
    ESP_LOGI("BoardI2c", "I2C%d new master driver %s: SDA=%d SCL=%d, %lu Hz",
             static_cast<int>(port), isMpu ? "MPU6050" : "BH1750/EEPROM/INA226",
             config.sda_io_num, config.scl_io_num,
             static_cast<unsigned long>(DEMO_I2C_FREQUENCY_HZ));
    return ESP_OK;
}

namespace {
esp_err_t deviceFor(i2c_port_t port, uint8_t address, i2c_master_dev_handle_t *device)
{
    if (!validPort(port) || address > 0x7f) return ESP_ERR_INVALID_ARG;
    auto &bus = buses[static_cast<unsigned>(port)];
    if (!bus.handle) return ESP_ERR_INVALID_STATE;
    const TickType_t cooldown = pdMS_TO_TICKS(port == DEMO_MPU_I2C_PORT ? 20 : 100);
    if (bus.coolingDown && static_cast<TickType_t>(xTaskGetTickCount() - bus.failedAt) < cooldown)
        return ESP_ERR_TIMEOUT;
    bus.coolingDown = false;
    if (!bus.devices[address]) {
        i2c_device_config_t config = {};
        config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        config.device_address = address;
        config.scl_speed_hz = DEMO_I2C_FREQUENCY_HZ;
        const esp_err_t err = i2c_master_bus_add_device(bus.handle, &config, &bus.devices[address]);
        if (err != ESP_OK) return err;
    }
    *device = bus.devices[address];
    return ESP_OK;
}

esp_err_t finish(i2c_port_t port, esp_err_t err)
{
    if (err == ESP_ERR_TIMEOUT) {
        auto &bus = buses[static_cast<unsigned>(port)];
        bus.coolingDown = true;
        bus.failedAt = xTaskGetTickCount();
    }
    // No retry loops or driver delete/reset while tasks are using device handles.
    return err;
}

int timeoutMs(TickType_t ticks)
{
    // Never pass -1 (wait forever), even if a caller supplies portMAX_DELAY.
    return static_cast<int>(std::min<uint64_t>(1000, std::max<uint64_t>(1,
        static_cast<uint64_t>(ticks) * portTICK_PERIOD_MS)));
}
}

esp_err_t write(i2c_port_t port, uint8_t address, const uint8_t *data, size_t size, TickType_t timeout)
{
    if (!data || !size) return ESP_ERR_INVALID_ARG;
    i2c_master_dev_handle_t device = nullptr;
    const esp_err_t err = deviceFor(port, address, &device);
    return err == ESP_OK ? finish(port, i2c_master_transmit(device, data, size, timeoutMs(timeout))) : err;
}

esp_err_t read(i2c_port_t port, uint8_t address, uint8_t *data, size_t size, TickType_t timeout)
{
    if (!data || !size) return ESP_ERR_INVALID_ARG;
    i2c_master_dev_handle_t device = nullptr;
    const esp_err_t err = deviceFor(port, address, &device);
    return err == ESP_OK ? finish(port, i2c_master_receive(device, data, size, timeoutMs(timeout))) : err;
}

esp_err_t writeRead(i2c_port_t port, uint8_t address, const uint8_t *tx, size_t txSize,
                    uint8_t *rx, size_t rxSize, TickType_t timeout)
{
    if (!tx || !txSize || !rx || !rxSize) return ESP_ERR_INVALID_ARG;
    i2c_master_dev_handle_t device = nullptr;
    const esp_err_t err = deviceFor(port, address, &device);
    return err == ESP_OK ? finish(port, i2c_master_transmit_receive(device, tx, txSize, rx, rxSize, timeoutMs(timeout))) : err;
}

} // namespace BoardI2c
