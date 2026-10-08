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
    ├── power_monitor/           INA226 register driver and configurable shunt calibration
    ├── gps_tracker/             NEO-6M UART/NMEA RMC+GGA reader
    ├── data_logger/             AT24C256 CRC-protected circular telemetry log
    ├── telemetry_link/          MQTT/event publishing interface scaffold
    └── project_types/            Shared telemetry and event data types
```

Each hardware/feature component owns its `CMakeLists.txt`, public headers in
`include/`, and implementation source. `board_config` and `project_types` are
header-only shared components. `main.cpp` coordinates the demo rather than
containing sensor drivers. The prototype uses one FreeRTOS control task. I2C
devices are accessed sequentially from that task; add a shared mutex or a
single bus-owner task before moving access into concurrent tasks.

## Implemented prototype

- Read a throttle potentiometer using ADC1.
- Drive motor and headlight driver inputs using separate LEDC timers.
- Read MPU6050 acceleration over I2C and detect sustained tilt over 60 degrees.
- Keep the motor relay off at startup; arm only after the throttle stays near
  zero.
- Latch the motor off after a sustained tilt threshold or sensor read failure.
- Print basic status to the serial console.

`PowerMonitor`, `GpsTracker`, and `DataLogger` now contain initial drivers.
`TelemetryLink` remains an interface-only scaffold; Wi-Fi/MQTT, dashboard,
and email alerts are not implemented. Shared pin defaults and device settings
are in `components/board_config/include/BoardConfig.h`.

## Component hand-off

| Component | Start in | Main implementation work |
| --- | --- | --- |
| `crash_detector` | `CrashDetector.cpp` | Calibrate MPU6050 axes, filter tilt, validate fall threshold |
| `motor_controller` | `MotorController.cpp` | Calibrate ADC range, verify PWM/relay polarity and limits |
| `headlight_controller` | `HeadlightController.cpp` | Tune soft-start and adaptive brightness curve |
| `power_monitor` | `PowerMonitor.cpp` | Confirm module address/shunt calibration; calibrate battery state-of-charge model |
| `gps_tracker` | `GpsTracker.cpp` | Test UART2 wiring and NMEA fixes outdoors; validate stale-fix handling |
| `data_logger` | `DataLogger.cpp` | Test page writes, power-loss recovery, and circular-log reads on the actual EEPROM |
| `telemetry_link` | `TelemetryLink.cpp` | Add Wi-Fi/MQTT lifecycle, payload format and reconnect policy |

Keep motor cut-off local and independent of Wi-Fi. Coordinate I2C ownership
before enabling concurrent sensor and EEPROM tasks; protect a shared bus with
one mutex or a single bus-owner task.

## Default pin map

Defaults target the classic ESP32 DOIT DevKit V1. Confirm every pin against the
actual board and modules before wiring.

| Function | ESP32 pin |
| --- | --- |
| Throttle potentiometer wiper | GPIO34 / ADC1_CHANNEL_6 (oneshot `ADC_CHANNEL_6`) |
| Motor driver PWM input | GPIO25 |
| Headlight MOSFET PWM input | GPIO26 |
| Relay control input | GPIO27 |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |
| NEO-6M TX → ESP32 RX | GPIO16 / UART2 |
| NEO-6M RX ← ESP32 TX | GPIO17 / UART2 |

The purchased MPU6050, INA226, and AT24C256 share the I2C bus on GPIO21/22.
Default I2C addresses in `BoardConfig.h` assume MPU6050 AD0 low (0x68), INA226
A0/A1 strapped for 0x40, and AT24C256 A0-A2 grounded (0x50); verify the actual
module straps before wiring. AT24C256 uses 32 KiB capacity, 64-byte pages, and
a 16-bit memory address. Do not run its I2C pull-ups at 5 V on the ESP32 bus.

The INA226 module's current calibration depends on the value printed on its
onboard shunt resistor. Confirm that marking before using current and power
readings; a generic "20 A" module label is not enough to select a safe,
accurate calibration. The initial code assumes an R002 (2 mOhm) shunt with a
20 A range; update `DEMO_INA226_SHUNT_MICRO_OHMS` and
`DEMO_INA226_MAX_CURRENT_MILLIAMPS` in `BoardConfig.h` to match the exact module
before connecting the load. The driver rejects configurations whose maximum
current would exceed the INA226 shunt-voltage range. State of charge currently
reports unknown (`-1`); it needs a calibrated 3S battery model and is not
inferred by linear voltage scaling.

The logger stores 64-byte CRC-protected records in a circular region and scans
the EEPROM at startup to recover the newest complete record after power loss.
At a 1 Hz sampling rate, 32 KiB stores about 8.5 minutes of history. Verify the
EEPROM address pins, page size, and I2C voltage before first use.

Power the potentiometer from 3.3 V and GND; never connect its wiper to 5 V.
The throttle is on GPIO34 / ADC1_CH6. Drive the JGB37-310 motor through the
LR7843 MOSFET module and the relay's appropriately rated contacts; drive the
12 V headlight LED through a suitable current-limited driver and MOSFET module.
Never connect motor or lamp current directly to ESP32 pins. Check relay active
polarity and update `DEMO_RELAY_ACTIVE_LEVEL` if necessary.

Use the 3S pack's BMS and a charger intended for 3S Li-ion packs (12.6 V
termination). Fuse the battery output near the pack. The LM2596 is for the
low-voltage electronics supply, not for powering the motor or headlight.

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
