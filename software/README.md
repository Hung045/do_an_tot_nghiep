# ESP32 vehicle safety demo

This is the firmware workspace for the ESP32 DOIT DevKit V1 graduation-project
mock-up. It is an early prototype, not a complete or certified vehicle safety
system.

## Project structure

```text
software/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt           Dependencies for the app component
│   └── main.cpp                 app_main runs a minimal module test
└── components/
    ├── board_config/            Shared default GPIO and peripheral settings
    ├── crash_detector/          MPU6050 tilt measurement and crash threshold
    ├── motor_controller/        Throttle ADC, motor PWM, and relay
    ├── headlight_controller/    Headlight PWM output
    ├── power_monitor/           INA226 register driver and configurable shunt calibration
    ├── gps_tracker/             NEO-6M UART/NMEA RMC+GGA reader
    ├── data_logger/             AT24C256 CRC-protected circular telemetry log
    ├── telemetry_link/          Wi-Fi and MQTT telemetry/event publishing
    └── project_types/            Shared telemetry and event data types
```

Each hardware/feature component owns its `CMakeLists.txt`, public headers in
`include/`, and implementation source. `board_config` and `project_types` are
header-only shared components. Test modules directly from `main/main.cpp`;
there are no separate example projects or selectable test modes.

## Current main: integrated vehicle test

PWM frequency is temporarily 1 kHz (previously 20 kHz) to diagnose the
LR7843 output appearing near supply voltage at low duty. Compare settled
10%, 20%, and 30% duty readings and motor response; a DC meter does not
verify waveform quality. This shared frequency also applies to the headlight
driver when it is initialized. The current main integrates motor control,
MPU6050 crash detection, GPS, AT24C256 logging, INA226 power measurement,
headlight output, Wi-Fi, MQTT, and the local web dashboard.

`main/main.cpp` starts with relay OFF and PWM zero. Hold throttle at 0-2%
for one continuous second to arm, then raise it to at least 5% to start.
The relay closes first; PWM starts after a 100 ms settling interval.
PWM uses 0.1% command steps with the existing 10-bit LEDC timer. Raw ADC is
mapped before rounding and smoothed with an EMA (alpha 0.2 per 20 ms).
Duty rises by at most 1 percentage point per iteration and falls by at most
2 points. A full-scale rise takes about 2 s plus relay/filter latency.
Starting also requires filtered throttle >=5%. Stop and fault paths bypass
the ramp; the existing 200 ms stop confirmation uses unfiltered input.
Short low-input glitches hold the filter and PWM; confirmed stops clear both.
Logs show raw throttle, filtered throttle and PWM with one decimal place.
The 50-kohm potentiometer still uses the configured ADC endpoints; resistance
is not a multiplier for duty. Wire its ends to 3V3/GND and wiper to GPIO34.
This is open-loop duty control, not proportional measured motor speed.
The startup lock, 5% start threshold and relay settling delay still apply.
Returning to 0-2% for 200 ms sets PWM zero
before opening the relay and returns directly to READY. The one-second
low-throttle hold is required only at startup/reset, not after normal stops.
ADC or output-command errors latch a stop until reset, retrying both stop
commands. The control loop runs every 20 ms. Relay control is active-low.
Short low-throttle glitches hold the previous PWM command; a sustained low
input stops after 200 ms from its first sample (plus sampling/scheduling latency).
ADC faults bypass this debounce. Confirmed stops log immediately.
Every 1 second, it logs throttle raw ADC and percent from the same sample,
alongside the relay and PWM commands and
LOCKED/READY/RELAY_WAIT/RUNNING/FAULT_LOCKED state.
Command values are not physical feedback from the relay contacts or PWM pin.
Missing modules are reported through logs and telemetry. With
`DEMO_REQUIRE_MPU6050` enabled (the default), a missing MPU6050 locks motor
PWM and relay off while networking and telemetry continue. Disable it only
for an identified motor bench test.

Initially verify all commands with motor power disconnected. Full throttle
now commands 100% duty; there is no motor current limit. Fit an appropriately rated flyback diode and
fuse before powered testing; this firmware does not provide overcurrent
protection or detect a disconnected/floating throttle input reliably.
Identify the module terminals by their printed labels, not physical position.
Supply the relay module at its rated 5 V, connect control GND to ESP32 GND,
and connect IN to GPIO27 only after verifying 3.3 V input compatibility and
that IN does not expose ESP32 to 5 V. Use a separate regulated relay supply
if needed; do not connect its positive output to ESP32 3V3.
Use a continuity meter on isolated COM-NO contacts: open when off, closed
when on. Serial logs and a clicking sound alone do not verify the contacts.
Flash and open Monitor. Verify high throttle at reset stays LOCKED/OFF/0%,
low throttle for one second produces READY, and returning to low throttle
stops output. EN/RESET restarts the startup lock. Software cannot guarantee
relay state before GPIO initialization; verify the hardware default state.

MPU6050, INA226, and EEPROM share the board I2C bus guard. Test each module
alone before connecting all three, then verify that a missing module is
reported without publishing a plausible-looking fake measurement.

`PowerMonitor`, `GpsTracker`, and `DataLogger` contain initial drivers.
When called by the application, `TelemetryLink` can connect to Wi-Fi and publish telemetry/event JSON messages
over MQTT. Configure its Wi-Fi SSID, password, and broker URI with
**ESP-IDF: SDK Configuration editor (menuconfig)** under **Vehicle telemetry**.
The local `sdkconfig` may contain the Wi-Fi password and must not be committed.
TelemetryLink initializes NVS before Wi-Fi and reports failures without
automatically erasing stored data. The current main uses CONFIG_DEMO_MQTT_BROKER_URI from menuconfig. An empty
URI tests Wi-Fi only; a nonempty URI starts MQTT after obtaining an IP.
While connected, main enqueues motor telemetry once per second on
`vehicle/motor`, using the same snapshot as the Monitor log:
`uptime_ms`, `phase`, `raw`, `throttle_pct`, `filtered_pct`, `pwm_pct`,
and `relay_cmd` (boolean command, not physical contact feedback).
System measurements and module availability are published on
`vehicle/telemetry`; unavailable measurements use JSON `null`.
Offline samples are skipped; network transmission runs in the MQTT task with
a 4096-byte outbox limit. A sample queued just before disconnect may arrive
after reconnect; use `uptime_ms` to identify its age. QoS 0 is not guaranteed delivery.

On the broker laptop, subscribe before turning the potentiometer:

```powershell
& "<Mosquitto-install-folder>\mosquitto_sub.exe" -h localhost -p 1883 -t "vehicle/#" -v
```

Compare received values with Monitor, then disconnect/reconnect Wi-Fi and
verify the motor loop continues and telemetry resumes. Build and flash the
updated firmware before testing; connection alone does not verify publishing.
Connection and reconnection run through events; the motor loop does not wait
for an access point or IP. An empty SSID skips network startup with an error
log and leaves the motor test running offline.
Motor commands use `vehicle/motor`, system telemetry uses
`vehicle/telemetry` (QoS 0), and crash events use `vehicle/event` (QoS 1).
The dashboard subscribes to `vehicle/#`. Email alerts are not implemented.
Shared pin defaults and device settings are in
`components/board_config/include/BoardConfig.h`.

## Component hand-off

For the connection test, enter a 2.4 GHz network's SSID and password in
menuconfig > Vehicle telemetry, save, build, flash, and monitor with motor
power disconnected. Success logs `Wi-Fi connected, IP=...` and
`STEP 1 PASS: IP obtained; MQTT remains disabled`. Motor logs continue once
per second. Turn the access point off/on to check disconnect/reconnect;
disconnect logs include a reason code. To test MQTT, set MQTT broker URI to
`mqtt://<broker-LAN-IP>:1883` for a plain local broker, then rebuild and flash.
Expect `Connected to MQTT broker` instead of the Wi-Fi-only STEP 1 message.
The broker must be reachable from the ESP32 network and allow this client.
Do not publish sdkconfig or passwords.

| Component | Start in | Main implementation work |
| --- | --- | --- |
| `crash_detector` | `CrashDetector.cpp` | Calibrate MPU6050 axes, filter tilt, validate fall threshold |
| `motor_controller` | `MotorController.cpp` | Calibrate ADC range, verify PWM/relay polarity and limits |
| `headlight_controller` | `HeadlightController.cpp` | Tune soft-start and adaptive brightness curve |
| `power_monitor` | `PowerMonitor.cpp` | Confirm module address/shunt calibration; calibrate battery state-of-charge model |
| `gps_tracker` | `GpsTracker.cpp` | Test UART2 wiring and NMEA fixes outdoors; validate stale-fix handling |
| `data_logger` | `DataLogger.cpp` | Test page writes, power-loss recovery, and circular-log reads on the actual EEPROM |
| `telemetry_link` | `TelemetryLink.cpp` | Configure a private broker if needed; verify topic/payload and reconnect behavior |

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
The throttle is on GPIO34 / ADC1_CH6. Drive the JGA45-370 motor (12 V, marked 1364 rpm) through the
LR7843 MOSFET module and the relay's appropriately rated contacts; drive the
12 V headlight LED through a suitable current-limited driver and MOSFET module.
Never connect motor or lamp current directly to ESP32 pins. Check relay active
polarity and update `DEMO_RELAY_ACTIVE_LEVEL` if necessary.

Use the 3S pack's BMS and a charger intended for 3S Li-ion packs (12.6 V
termination). Fuse the battery output near the pack. The LM2596 is for the
low-voltage electronics supply, not for powering the motor or headlight.

## Build in VS Code

1. Open this `software` directory as the ESP-IDF project (not the repository
   parent directory).
2. Use ESP-IDF 5.5.5 and select target `esp32`. `sdkconfig.defaults` selects
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

## Sharing this workspace with Git

Commit source files, component CMakeLists, Kconfig.projbuild, README,
.gitignore and sdkconfig.defaults. Keep all component folders for integration;
not being initialized by the current main does not make a module obsolete.
The portable .clangd and .vscode/launch.json are optional editor helpers.

Do not commit build/, build-backup-*/, .cache/, sdkconfig, sdkconfig.old,
local credentials or machine-specific VS Code settings. These remain local.
Use ESP-IDF build commands rather than generic CMake/MinGW configuration in
this workspace to avoid conflicting build generators.

Before sharing, run `git diff --check`, build using ESP-IDF, and review
`git status --short` and `git diff`. Configure each developer's Wi-Fi and
broker settings locally through menuconfig. Hardware test results apply to
the current motor bench setup; the other modules still need integration tests.
