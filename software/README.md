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
    ├── headlight_controller/    Headlight brightness and adaptive SG90 steering
    ├── power_monitor/           INA226 register driver and configurable shunt calibration
    ├── gps_tracker/             NEO-6M UART/NMEA RMC+GGA reader
    ├── data_logger/             AT24C256 CRC-protected circular telemetry log
    ├── telemetry_link/          Wi-Fi and MQTT telemetry/event publishing
    └── project_types/            Shared telemetry and event data types
```

Each hardware/feature component owns its `CMakeLists.txt`, public headers in
`include/`, and implementation source. `board_config` also initializes the two
hardware I2C buses; `project_types` is header-only. Test modules directly from `main/main.cpp`;
there are no separate example projects or selectable test modes.

## Current main: integrated vehicle test

The application now has four FreeRTOS tasks (priorities below are application
priorities; ESP-IDF also runs its own Wi-Fi/MQTT tasks):

| Task | Work | Timing / priority |
| --- | --- | --- |
| `vehicle_control` | MPU6050, throttle, safety lock, motor PWM/relay and SG90 | target 20 ms / 5 |
| `vehicle_light` | BH1750 and headlight, including fault blinking | 100 ms delay between updates / 4 |
| `vehicle_telemetry` | GPS, INA226, EEPROM and Monitor logs | GPS polled between snapshots (normally 100 ms); snapshot every second / 3 |
| `vehicle_network` | Wi-Fi startup and MQTT publishing | waits for queued snapshots / 2 |

Network writes run only in `vehicle_network`. A one-element queue replaces
pending periodic telemetry with the latest sample. Frames older than 1.5 s are
skipped before publishing; an already-started socket write cannot be recalled.
Individual MQTT network operations use a 2 s timeout, which is not a guarantee
of total end-to-end delivery latency. Slow sends log `MQTT send=... ms`.
GPS/EEPROM/logs and headlight work therefore do not wait on MQTT socket writes.
MPU6050 uses I2C0 with its own mutex. BH1750, EEPROM and INA226 share I2C1
and a separate mutex. EEPROM releases the bus during its internal write cycle.
Task periods remain targets, not hard real-time guarantees.

The crash flag stays latched until reset. Telemetry puts the first crash in a
separate queue and the network task retries until MQTT accepts it into the
bounded QoS 1 outbox. Periodic sample replacement cannot overwrite this event.
This queue/outbox is in RAM: reset or extended offline expiry can still lose an
undelivered event. EEPROM history is separate from MQTT event delivery.

After flashing, verify with motor load power disconnected:

1. Compare throttle in Monitor, `vehicle/motor` and the web with a healthy
   network. Live samples run at 5 Hz; record any persistent delay.
2. Stop the broker for 15 seconds. Monitor and EEPROM should continue updating;
   cover/uncover BH1750 to check that the headlight still responds.
3. Trigger a tilt fault while offline. PWM/relay commands must go to OFF/zero
   and the light must blink. Do not reset yet.
4. Restart the broker. Check that current telemetry resumes without replaying
   a backlog of periodic samples and a `vehicle/event` crash message arrives
   (subscribe before the broker reconnects the ESP32). QoS 1 can duplicate it.
5. Reset only after checking the event. Return throttle to zero to re-arm.

Build checks cannot replace these hardware/network tests.

PWM frequency is temporarily 1 kHz (previously 20 kHz) to diagnose the
LR7843 output appearing near supply voltage at low duty. Compare settled
10%, 20%, and 30% duty readings and motor response; a DC meter does not
verify waveform quality. This shared frequency also applies to the headlight
driver when it is initialized. The current main integrates motor control,
MPU6050 crash detection, GPS, AT24C256 logging, INA226 power measurement,
headlight output, Wi-Fi, MQTT, and the local web dashboard.

`main/main.cpp` starts with relay OFF and PWM zero. Return throttle to 0-2%
to arm on the next valid control sample, then raise it to at least 5% to start.
The relay closes first; PWM starts after a 100 ms settling interval.
PWM uses 0.1% command steps with the existing 10-bit LEDC timer. Raw ADC is
mapped before rounding and smoothed with an EMA (alpha 0.2 per 20 ms).
Duty rises by at most 1 percentage point per iteration and falls by at most
2 points. A full-scale rise takes about 2 s plus relay/filter latency.
Starting also requires filtered throttle >=5%. Stop and fault paths bypass
the ramp. A valid unfiltered throttle sample <=2% stops immediately and clears
the filter and PWM, including brief low-input dips. Logs show throttle and PWM.
The 50-kohm potentiometer still uses the configured ADC endpoints; resistance
is not a multiplier for duty. Wire its ends to 3V3/GND and wiper to GPIO34.
This is open-loop duty control, not proportional measured motor speed.
The startup lock, 5% start threshold and relay settling delay still apply.
Returning to 0-2% sets PWM zero before opening the relay and returns directly
to READY. Startup/reset also requires only one valid low-throttle sample.
ADC or output-command errors latch a stop until reset, retrying both stop
commands. The control loop runs every 20 ms. Relay control is active-low.
There is no 200 ms stop debounce; response is subject to sampling/scheduling
latency. ADC faults latch a safety stop. Successful stop commands log immediately.
Every 1 second, it logs throttle percent,
alongside the relay and PWM commands and
LOCKED/READY/RELAY_WAIT/RUNNING/FAULT_LOCKED state.
Command values are not physical feedback from the relay contacts or PWM pin.
Live motor, MPU and lighting snapshots are queued every 200 ms (5 Hz), plus
control-state changes. A separate low-priority storage task reads INA226 and
writes EEPROM at most once per second; its I2C waits do not block live telemetry.
Power readings older than 2.5 seconds are invalidated. EEPROM write status is
the latest completed write result, not an acknowledgement of each live frame.
Storage and network queues each retain only the newest pending sample.
MQTT connection failures still interrupt web updates; 200 ms is the production
interval, not a guaranteed end-to-end delivery time.
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
one valid low-throttle sample produces READY, and returning to low throttle
stops output. EN/RESET restarts the startup lock. Software cannot guarantee
relay state before GPIO initialization; verify the hardware default state.

MPU6050 has a dedicated board I2C bus guard; BH1750, INA226 and EEPROM share
the auxiliary bus guard. Test each module alone, then verify that a missing module is
reported without publishing a plausible-looking fake measurement.

`PowerMonitor`, `GpsTracker`, and `DataLogger` contain initial drivers.
When called by the application, `TelemetryLink` can connect to Wi-Fi and publish telemetry/event JSON messages
over MQTT. Configure its Wi-Fi SSID, password, and broker URI with
**ESP-IDF: SDK Configuration editor (menuconfig)** under **Vehicle telemetry**.
The local `sdkconfig` may contain the Wi-Fi password and must not be committed.
TelemetryLink initializes NVS before Wi-Fi and reports failures without
automatically erasing stored data. The current main uses CONFIG_DEMO_MQTT_BROKER_URI from menuconfig. An empty
URI tests Wi-Fi only; a nonempty URI starts MQTT after obtaining an IP.
While connected, main publishes motor telemetry once per second on
`vehicle/motor`, using the same snapshot as the Monitor log:
`uptime_ms`, `phase`, `raw`, `throttle_pct`, `filtered_pct`, `pwm_pct`,
and `relay_cmd` (boolean command, not physical contact feedback).
System measurements and module availability are published on
`vehicle/telemetry`; unavailable measurements use JSON `null`.
Offline samples are skipped. The live motor and system streams use direct
QoS 0 publishing so stale periodic samples do not fill the outbox. QoS 1
events use a bounded 8192-byte outbox and may arrive after reconnect. Use
`uptime_ms` to identify sample age. QoS 0 does not guarantee delivery.

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

Keep motor cut-off local and independent of Wi-Fi. Initialize buses through
`BoardI2c::initialize(port)` and protect each transaction with that port's
`BoardI2c::LockGuard`. Keep EEPROM access in the telemetry task; the bus mutex
does not serialize whole multi-page EEPROM operations across multiple callers.

## Default pin map

Defaults target the classic ESP32 DOIT DevKit V1. Confirm every pin against the
actual board and modules before wiring.

| Function | ESP32 pin |
| --- | --- |
| Throttle potentiometer wiper | GPIO34 / ADC1_CHANNEL_6 (oneshot `ADC_CHANNEL_6`) |
| Motor driver PWM input | GPIO25 |
| Headlight MOSFET PWM input | GPIO26 |
| Relay control input | GPIO27 |
| SG90 positional servo signal | GPIO23 (LEDC timer 2/channel 2, 50 Hz) |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |
| BH1750 + AT24C256 + INA226 SDA (I2C1) | GPIO18 |
| BH1750 + AT24C256 + INA226 SCL (I2C1) | GPIO19 |
| NEO-6M TX → ESP32 RX | GPIO16 / UART2 |
| NEO-6M RX ← ESP32 TX | GPIO17 / UART2 |

MPU6050 alone uses I2C0 on GPIO21/22. BH1750, INA226 and AT24C256 share
I2C1 on GPIO18/19. Both buses run at 100 kHz and have independent mutexes.
Before flashing this pin-map change, turn power off and move the auxiliary
modules' SDA/SCL wires from GPIO21/22 to GPIO18/19. Do not bridge the buses.
Keep a common ground and suitable pull-ups to 3.3 V on each bus; account for
pull-ups already fitted to modules. ESP32 internal pull-ups alone may be too weak.

Default I2C addresses in `BoardConfig.h` assume MPU6050 AD0 low (0x68), INA226
A0/A1 strapped for 0x40, and AT24C256 A0-A2 grounded (0x50); verify the actual
module straps before wiring. BH1750 is probed at 0x23 and then 0x5C.
AT24C256 uses 32 KiB capacity, 64-byte pages, and
a 16-bit memory address. Do not run its I2C pull-ups at 5 V on the ESP32 bus.

After rewiring, test with motor load power disconnected:

1. Monitor should report I2C0 on SDA21/SCL22 and I2C1 on SDA18/SCL19.
2. Confirm MPU6050 and EEPROM are available, and BH1750 lux changes when covered.
   INA226 may remain MISSING until installed; its readings remain unavailable.
3. Tilt the MPU6050 to verify the safety lock, then reset with throttle at zero.
4. Verify EEPROM records can be read after reset and GPS obtains a valid fix.
5. To test bus separation, power off, disconnect auxiliary modules, and restart.
   MPU6050 should still be detected. Restore wiring with power off afterwards.

The INA226 module's current calibration depends on the value printed on its
onboard shunt resistor. Confirm that marking before using current and power
readings; a generic "20 A" module label is not enough to select a safe,
accurate calibration. The fitted module is marked R020 (20 mOhm); the code uses a
4 A measurement range (80 mV across the shunt). This is measurement scaling,
not motor current protection or a verified continuous-current rating of the board.
The INA226 shunt input range of about 81.92 mV corresponds to about 4.096 A
with this resistor; currents beyond the configured range cannot be trusted.
See the [TI INA226 datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf).
Update `DEMO_INA226_SHUNT_MICRO_OHMS` and
`DEMO_INA226_MAX_CURRENT_MILLIAMPS` in `BoardConfig.h` to match the exact module
before connecting the load. The driver rejects configurations whose maximum
current would exceed the INA226 shunt-voltage range. State of charge currently
reports unknown (`-1`); it needs a calibrated 3S battery model and is not
inferred by linear voltage scaling.

INA226 initialization and each sample's validity are separate. Monitor prints
`INA226=READ_ERROR` with the driver error on a failed read instead of zero volts.
MQTT keeps `power_available` for initialization and adds `power_valid` for the
current sample; invalid voltage/current are JSON `null`. The existing EEPROM
float fields store NaN for unavailable power readings, preserving the 64-byte
layout and CRC. Historical zero records cannot retrospectively be classified.
The native test `python tests/power_host_test.py [path-to-g++]` checks init/read
failures, valid zero readings, signed current and recovery with fake I2C.

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

## Adaptive headlight with SG90

Use a positional SG90, not a continuous-rotation servo. The implementation lives
in `components/headlight_controller/AdaptiveServo.cpp`; the control task updates
it from the same MPU6050 sample used for crash detection. It uses independent
LEDC timer 2/channel 2 at 50 Hz and 16-bit resolution, leaving motor and lamp
PWM timers unchanged. See [Espressif LEDC documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32/api-reference/peripherals/ledc.html)
and [TowerPro SG90 specifications](https://towerpro.com.tw/product/sg90-7/).

| SG90 lead (verify markings on your servo) | Connection |
| --- | --- |
| Orange/yellow, signal | ESP32 GPIO23 |
| Red, supply | Separate regulated servo supply around 5 V, suitable for its starting/stall current |
| Brown/black, ground | Servo supply ground and ESP32 GND |

Do not power the servo from ESP32 3V3, a GPIO, or the 3S battery directly. With
ESP32 on USB, connect only the external servo supply's ground to the ESP32;
do not join its positive rail to ESP32 3V3/VIN. Turn power off before rewiring.
Given the earlier overheated board, use a verified working kit and keep the
suspected damaged I2C modules disconnected during initial servo tests.

Defaults in `BoardConfig.h`:

- Center command 90 degrees; range 60..120 degrees. Mapping uses 1000..2000 us
  for nominal 0..180 degrees, giving a restricted initial pulse range around
  1500 us. Actual angle depends on the servo; calibrate without forcing stops.
- Gain 1, direction +1, zero offset 0 degrees; 3-degree continuous deadband,
  150 ms roll filter, maximum command rate 60 degrees/second.
- MPU mounted X forward, Y lateral, Z up: roll is `atan2(ay, az)` in degrees.
  `DEMO_MPU_ROLL_USE_Y=0` uses X as the lateral axis for a different mounting.
  `DEMO_SERVO_DIRECTION=-1` reverses steering; `DEMO_SERVO_ROLL_ZERO_DEG` corrects
  the stationary mounting offset without changing the crash threshold.

Start with motor power disconnected, servo horn/lamp detached and the MPU flat.
After initialization, install the horn so the lamp faces forward at the center
command. Orient the servo shaft vertically for horizontal left/right steering.
Mount only a lightweight lamp supported without forcing the servo shaft.
Tilt the MPU slowly in both directions below 60 degrees and check the Monitor's
`SERVO TRACKING | roll=... | command=... | target=...` line once per second.
Reverse the direction setting if the lamp turns the wrong way. Return flat and
verify centering. A missing/failed MPU sample, non-finite roll, roll magnitude
at least 60 degrees, or latched safety fault commands a smooth return to center.
After a crash lock, place the MPU upright, return throttle to zero and reset.
PWM errors stop servo pulses and log UNAVAILABLE; reset is required to retry.

There is no servo position or power feedback: TRACKING means PWM commands are
being generated, not that the servo is physically connected. On startup the
center pulse can move an off-center servo immediately; do the first test with
the horn detached. Lamp and servo commands are published in the `lighting`
object on `vehicle/telemetry` and shown on the dashboard, as well as Monitor.
They are not stored in EEPROM. The headlight follows BH1750 and blinks on a safety fault.
Servo movement is independent of ambient light, so it can be tested in daylight.

This is a bench prototype based on accelerometer roll. Vehicle acceleration
affects this estimate; it does not predict a bend before the vehicle leans, and
one horizontal servo does not also correct beam roll. Dynamic vehicle use needs
validated orientation estimation and optical/mechanical calibration.

Run `python tests/adaptive_servo_host_test.py [path-to-g++]` to check pulse setup,
limits, smoothing, failure behavior, signed roll and the existing crash threshold
using mocked hardware. Physical servo travel and supply stability need bench tests.

## Build in VS Code

GPS diagnostics use UART2 RX16 (GPS TX), TX17 (GPS RX), 9600 baud. The Monitor
prints `GPS UART status=...` once per telemetry snapshot, with cumulative byte,
checksum-valid sentence and rejected-line counts. `rx_age`/`nmea_age` are ages
in milliseconds; -1 means never received. UART initialization alone does not
prove a module is connected. See `web/README.md` for each diagnostic state.
These fields travel in copied network snapshots; the EEPROM record format is
unchanged. A GGA quality of zero or an RMC void status invalidates the current
fix, and fixes expire after 5 seconds without a valid position update. GGA's
satellite count reports satellites used in the fix, not all satellites visible.

Host regression checks (no hardware): `python tests/gps_host_test.py`, with a
native `g++` on PATH, or pass its executable path as the first argument. This
compiles the actual GPS driver with a fake UART/clock to exercise fragmented
NMEA, checksum errors, no-fix reports, stale data, UART failures and recovery.
It does not validate the physical GPS, power supply or antenna.

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

### Sensor failure handling and regression checks

- Total tilt is 0..180 degrees, including an upside-down MPU. Tilt >=60 degrees
  must persist for 200 ms; a failed read breaks that confirmation interval.
- All four I2C modules use the new ESP-IDF master driver through BoardI2c.
  Device handles are cached per bus/address under the existing bus mutex.
  No legacy I2C driver is installed. See the [ESP-IDF I2C guide](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32/api-reference/peripherals/i2c.html).
- Runtime MPU reads use a 20 ms transaction timeout and do not wait for the
  dedicated bus mutex. The first failure stops PWM/relay and enters `SENSOR_WAIT`.
  Three good readings spanning at least 40 ms are required to recover, within
  200 ms of the first failure. Recovery returns to `LOCKED`, requiring throttle
  <=2% on a valid control sample; it never automatically resumes motor power.
- Failure to recover by the deadline latches `FAULT_LOCKED`. MPU reads then
  pause until reset, while lamp warning, servo centering and telemetry continue.
  Logs and motor MQTT payload include `fault_reason`. The read deadline is not
  a guarantee of physical relay release time or recovery of damaged hardware.
- Timed-out buses wait at least 20 ms (MPU) or 100 ms (AUX) before another
  transaction. There is no busy retry/reset loop. Control skips missed periods
  instead of issuing catch-up reads, and runs on core 1 on dual-core ESP32.
  Watchdogs remain enabled with their existing timeouts.
- BH1750 read failures invalidate lux (`-1`) and command full lamp brightness
  on that update. Successful subsequent reads restore automatic brightness.
  Safety-fault blinking still takes priority.
- MQTT adds `mpu_valid`, `eeprom_write_ok` and `crash_latched`. Initialization
  flags retain their old meaning; invalid MPU tilt is JSON `null`. EEPROM write
  status describes the current record's write result, not a read-back verification.
- EEPROM byte 29 now stores the latched crash event rather than the instantaneous
  tilt threshold. The 64-byte format and CRC remain compatible with old records.
  `readLatest` returns the flag and startup logs it. Old records cannot recover
  events that the previous firmware never wrote. Logging remains periodic (1 Hz),
  so power loss before a successful write can still lose the latest event.

Run `python tests/sensor_recovery_host_test.py path-to-native-g++` for BH1750
failure/recovery and EEPROM crash persistence through restart/write failure.
`tests/adaptive_servo_host_test.py` also covers inverted orientation and interrupted
crash confirmation. These tests mock hardware; they do not replace bench tests.
`tests/mpu_health_host_test.py` covers transient faults, recovery confirmation,
deadline and clock wrap; `tests/board_i2c_host_test.py` covers handle reuse,
bus isolation, timeout conversion and cooldown. Both take a native g++ path.

After flashing, look for `I2C0 new master driver` / `I2C1 new master driver`.
With motor power disconnected, test startup and repeated Wi-Fi reconnects.
An isolated MPU error should log `MPU RECOVERING`, then `MPU RECOVERED` and
require zero throttle again. A persistent failure must keep relay/PWM off,
show a repeating lock reason and avoid repeated MPU reads. To simulate unplugged
hardware, power down before changing wiring; do not short SDA/SCL for testing.
If an interrupt watchdog panic recurs, capture the complete new backtrace;
the previous legacy-driver backtrace cannot prove the new driver is fault-free.

If the normal build cache contains a Windows GCC compiler or a missing
`CMakeFiles/rules.ninja`, use a clean, isolated directory from an ESP-IDF terminal:
`idf.py -B build/verified build`. Flash that same build with
`idf.py -B build/verified -p COM9 flash monitor` (replace COM9 as needed).
In VS Code set `idf.buildPath` to `${workspaceFolder}/build/verified` to use it
for Build/Flash as well. Keep generic CMake Tools builds out of this directory.

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
