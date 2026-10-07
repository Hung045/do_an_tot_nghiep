# ESP32 vehicle safety demo

This is the firmware workspace for the ESP32 DOIT DevKit V1 graduation-project
mock-up. It is an early prototype, not a complete or certified vehicle safety
system.

## Project structure

```text
do_an_tn/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt           Dependencies for the app component
│   └── main.cpp                 app_main creates the FreeRTOS control task
└── components/
    ├── board_config/            Shared default GPIO and peripheral settings
    ├── crash_detector/          MPU6050 tilt measurement and crash threshold
    ├── motor_controller/        Throttle ADC, motor PWM, and relay
    ├── headlight_controller/    Headlight PWM output
    ├── power_monitor/           INA226 interface scaffold
    ├── gps_tracker/             NEO-6M UART interface scaffold
    ├── data_logger/             EEPROM telemetry interface scaffold
    ├── telemetry_link/          MQTT/event publishing interface scaffold
    └── project_types/            Shared telemetry and event data types
```

Each hardware/feature component owns its `CMakeLists.txt`, public headers in
`include/`, and implementation source. `board_config` and `project_types` are
header-only shared components. `main.cpp` coordinates the demo rather than
containing sensor drivers. The current prototype uses one FreeRTOS control
task; separate sensor/network tasks and I2C synchronization should be added as
those modules are implemented.

## Implemented prototype

- Read a throttle potentiometer using ADC1.
- Drive motor and headlight driver inputs using separate LEDC timers.
- Read MPU6050 acceleration over I2C and detect sustained tilt over 60 degrees.
- Keep the motor relay off at startup; arm only after the throttle stays near
  zero.
- Latch the motor off after a sustained tilt threshold or sensor read failure.
- Print basic status to the serial console.

`PowerMonitor`, `GpsTracker`, `DataLogger`, and `TelemetryLink` currently expose
interfaces only; their methods return `ESP_ERR_NOT_SUPPORTED` until INA226,
NEO-6M, EEPROM, and network drivers are implemented. Circular logging,
dashboard, and email alerts are not implemented yet. Shared pin defaults are
in `components/board_config/include/BoardConfig.h`.

## Component hand-off

| Component | Start in | Main implementation work |
| --- | --- | --- |
| `crash_detector` | `CrashDetector.cpp` | Calibrate MPU6050 axes, filter tilt, validate fall threshold |
| `motor_controller` | `MotorController.cpp` | Calibrate ADC range, verify PWM/relay polarity and limits |
| `headlight_controller` | `HeadlightController.cpp` | Tune soft-start and adaptive brightness curve |
| `power_monitor` | `PowerMonitor.cpp` | Configure INA226 I2C address, shunt and battery scaling |
| `gps_tracker` | `GpsTracker.cpp` | Configure UART, parse NMEA and reject invalid/stale fixes |
| `data_logger` | `DataLogger.cpp` | Implement AT24C256 page writes, ring index and recovery metadata |
| `telemetry_link` | `TelemetryLink.cpp` | Add Wi-Fi/MQTT lifecycle, payload format and reconnect policy |

Keep motor cut-off local and independent of Wi-Fi. Coordinate I2C ownership
before enabling concurrent sensor and EEPROM tasks; protect a shared bus with
one mutex or a single bus-owner task.

## Default pin map

Defaults target the classic ESP32 DOIT DevKit V1. Confirm every pin against the
actual board and modules before wiring.

| Function | ESP32 pin |
| --- | --- |
| Throttle potentiometer wiper | GPIO34 / ADC1_CHANNEL_6 |
| Motor driver PWM input | GPIO25 |
| Headlight MOSFET PWM input | GPIO26 |
| Relay control input | GPIO27 |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |
| NEO-6M TX → ESP32 RX | GPIO16 / UART2 |
| NEO-6M RX ← ESP32 TX | GPIO17 / UART2 |

Power the potentiometer from 3.3 V and GND; never connect its wiper to 5 V.
Drive the motor and lamp through appropriately rated modules, not directly
from ESP32 pins. Check relay active polarity and update
`DEMO_RELAY_ACTIVE_LEVEL` if necessary.

## Build in VS Code

1. Open this `do_an_tn` directory as the ESP-IDF project (not the repository
   parent directory).
2. Use ESP-IDF 5.3.x and select target `esp32`. `sdkconfig.defaults` selects
   this target for a fresh build.
3. Run **ESP-IDF: Build your project**.
4. Connect the hardware, keep the driven wheel raised, then flash and monitor.

Generated `build/`, local `sdkconfig`, and editor-specific settings are
intentionally not shared. Each collaborator generates their own build and
`sdkconfig` from the committed defaults.

## Safety and known limitations

The tilt estimate uses only the accelerometer, so vibration and motor
acceleration can produce inaccurate readings. The threshold and behavior are
for a bench demo only. Test with a current-limited supply where possible and
verify the relay's de-energized state before connecting the motor.

Before a hardware test, review pin assignments, relay polarity, driver ratings,
and the physical emergency cut-off. Do not use this prototype on a road-going
vehicle.
