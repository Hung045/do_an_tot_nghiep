#pragma once

#include "driver/adc.h"
#include "driver/gpio.h"

// Default pins for the classic ESP32 DOIT DevKit V1. Update these to match
// the final wiring. Keep the throttle on ADC1 if Wi-Fi will be enabled.
#define DEMO_THROTTLE_ADC_CHANNEL ADC1_CHANNEL_6
#define DEMO_THROTTLE_GPIO GPIO_NUM_34
#define DEMO_MOTOR_PWM_GPIO GPIO_NUM_25
#define DEMO_HEADLIGHT_PWM_GPIO GPIO_NUM_26
#define DEMO_RELAY_GPIO GPIO_NUM_27
#define DEMO_I2C_SDA_GPIO GPIO_NUM_21
#define DEMO_I2C_SCL_GPIO GPIO_NUM_22
#define DEMO_GPS_UART_PORT 2
#define DEMO_GPS_UART_TX_GPIO 17
#define DEMO_GPS_UART_RX_GPIO 16
#define DEMO_GPS_BAUD_RATE 9600

// Set to 0 if the relay module is active-low.
#define DEMO_RELAY_ACTIVE_LEVEL 1

#define DEMO_MPU6050_ADDRESS 0x68
#define DEMO_PWM_FREQUENCY_HZ 20000
#define DEMO_PWM_RESOLUTION_BITS 10
