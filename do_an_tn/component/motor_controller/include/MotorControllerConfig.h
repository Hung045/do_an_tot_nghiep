#pragma once

#include "driver/adc.h"

// Defaults for the classic ESP32 DOIT DevKit V1. Change these after wiring
// the actual mock-up; the throttle must use an ADC1 pin when Wi-Fi is enabled.
#define DEMO_THROTTLE_ADC_CHANNEL ADC1_CHANNEL_6
#define DEMO_THROTTLE_GPIO 34
#define DEMO_MOTOR_PWM_GPIO 25
#define DEMO_HEADLIGHT_PWM_GPIO 26
#define DEMO_RELAY_GPIO 27
#define DEMO_I2C_SDA_GPIO 21
#define DEMO_I2C_SCL_GPIO 22

// Set to 0 if the relay module is active-low.
#define DEMO_RELAY_ACTIVE_LEVEL 1

#define DEMO_MPU6050_ADDRESS 0x68
#define DEMO_PWM_FREQUENCY_HZ 20000
#define DEMO_PWM_RESOLUTION_BITS 10
