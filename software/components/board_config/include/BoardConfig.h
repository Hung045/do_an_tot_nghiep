#pragma once

#include "driver/gpio.h"

// Default pins for the classic ESP32 DOIT DevKit V1. Update these to match
// the final wiring. Keep the throttle on ADC1 if Wi-Fi will be enabled.
#define DEMO_THROTTLE_ADC_CHANNEL ADC_CHANNEL_6
#define DEMO_THROTTLE_GPIO GPIO_NUM_34
#define DEMO_MOTOR_PWM_GPIO GPIO_NUM_25
#define DEMO_HEADLIGHT_PWM_GPIO GPIO_NUM_26
#define DEMO_RELAY_GPIO GPIO_NUM_27
#define DEMO_I2C_SDA_GPIO GPIO_NUM_21
#define DEMO_I2C_SCL_GPIO GPIO_NUM_22
#define DEMO_GPS_UART_PORT 2
#define DEMO_GPS_UART_TX_GPIO GPIO_NUM_17
#define DEMO_GPS_UART_RX_GPIO GPIO_NUM_16
#define DEMO_GPS_BAUD_RATE 9600

// I2C peripherals share GPIO21 (SDA) and GPIO22 (SCL).
#define DEMO_I2C_FREQUENCY_HZ 100000
#define DEMO_MPU6050_ADDRESS 0x68
#define DEMO_INA226_ADDRESS 0x40
// Reference only: R002 shunt, at most 20 A. Match these to the purchased module.
#define DEMO_INA226_SHUNT_MICRO_OHMS 2000
#define DEMO_INA226_MAX_CURRENT_MILLIAMPS 20000
#define DEMO_AT24C256_ADDRESS 0x50
#define DEMO_AT24C256_CAPACITY_BYTES 32768
#define DEMO_AT24C256_PAGE_SIZE_BYTES 64
#define DEMO_AT24C256_ADDRESS_BYTES 2

// 3S Li-ion pack. Voltage-based state of charge needs calibrated thresholds.
#define DEMO_BATTERY_SERIES_CELLS 3
#define DEMO_BATTERY_NOMINAL_MILLIVOLTS 11100
#define DEMO_BATTERY_FULL_MILLIVOLTS 12600

// Set to 0 if the relay module is active-low.
#define DEMO_RELAY_ACTIVE_LEVEL 1

#define DEMO_PWM_FREQUENCY_HZ 20000
#define DEMO_PWM_RESOLUTION_BITS 10
