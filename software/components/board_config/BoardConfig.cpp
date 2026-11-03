#include "BoardConfig.h"

#include "esp_log.h"

namespace BoardI2c {

SemaphoreHandle_t getMutex()
{
    static SemaphoreHandle_t mutex = nullptr;
    static bool initAttempted = false;
    static portMUX_TYPE initSpinlock = portMUX_INITIALIZER_UNLOCKED;

    if (mutex == nullptr) {
        portENTER_CRITICAL(&initSpinlock);
        if (mutex == nullptr) {
            mutex = xSemaphoreCreateMutex();
        }
        portEXIT_CRITICAL(&initSpinlock);
    }

    if (mutex != nullptr && xSemaphoreTake(mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (!initAttempted) {
            initAttempted = true;

            i2c_config_t config = {};
            config.mode = I2C_MODE_MASTER;
            config.sda_io_num = DEMO_I2C_SDA_GPIO;
            config.scl_io_num = DEMO_I2C_SCL_GPIO;
            config.sda_pullup_en = GPIO_PULLUP_ENABLE;
            config.scl_pullup_en = GPIO_PULLUP_ENABLE;
            config.master.clk_speed = DEMO_I2C_FREQUENCY_HZ;

            esp_err_t err = i2c_param_config(I2C_NUM_0, &config);
            if (err == ESP_OK) {
                err = i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0);
                if (err == ESP_ERR_INVALID_STATE) {
                    err = ESP_OK;
                }
            }
            if (err != ESP_OK) {
                ESP_LOGE("BoardI2c", "I2C bus initialization failed: %s", esp_err_to_name(err));
            }
        }
        xSemaphoreGive(mutex);
    }

    return mutex;
}

} // namespace BoardI2c