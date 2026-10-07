# ESP32 vehicle safety demo

This ESP-IDF demo is an initial hardware prototype for the graduation project.
It implements throttle ADC input, PWM motor control, an MPU6050 tilt safety
lock, relay motor-power cut-off, and a tilt-adaptive PWM headlight.

GPS, INA226, EEPROM logging, Wi-Fi, MQTT, and email alerts are not implemented
in this demo.

## Default pin map

The defaults target a classic ESP32 DOIT DevKit V1. Change the definitions in
`component/motor_controller/include/MotorControllerConfig.h` to match the final
wiring.

| Function | Default ESP32 connection |
| --- | --- |
| 10 kOhm throttle potentiometer wiper | GPIO34 / ADC1_CHANNEL_6 |
| Motor driver PWM input | GPIO25 |
| Headlight MOSFET PWM input | GPIO26 |
| Relay control input | GPIO27 |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |

Connect the potentiometer ends to 3.3 V and GND (not 5 V). The motor and
headlight must be powered through suitable driver modules; do not connect them
directly to ESP32 GPIO pins. Check the relay board's active level and update
`DEMO_RELAY_ACTIVE_LEVEL` if needed. GPIO34 is input-only and is suitable for
the throttle ADC.

## Demo behavior

1. On startup the relay and motor PWM are off.
2. The MPU6050 must be detected and the throttle must stay below 5% for one
   second before the demo arms the motor relay.
3. The potentiometer then controls motor PWM. The headlight starts softly and
   dims as the measured tilt approaches 60 degrees.
4. Tilt at or above 60 degrees for 200 ms, or an MPU6050/throttle read failure,
   latches a safety lock: motor PWM and relay are turned off. Restart the board
   after checking the setup to clear the lock.
5. Serial logging reports tilt, throttle, and safety state once per second.

The fall detector uses accelerometer-only tilt, which is a bench-demo
approximation and can be inaccurate during strong acceleration or vibration.
It is not a certified vehicle safety system. Test with the driven wheel raised,
use a current-limited supply where possible, and verify relay polarity and
emergency cut-off before connecting the motor.

## Build and flash

Open this directory as an ESP-IDF project in VS Code, select the classic ESP32
target, then build and flash using the ESP-IDF extension. Keep the wheel raised
for the first power-on test.
