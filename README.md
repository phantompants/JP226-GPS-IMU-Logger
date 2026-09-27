# JP226-GPS-IMU-Logger

An Arduino/PlatformIO telemetry logger for both the **M5Stack Cardputer ADV**
and the **original Cardputer**. The same source and the same compiled app binary
run on either model. M5Unified detects the board at runtime:

- Cardputer ADV: logs the onboard BMI270 accelerometer and gyroscope.
- Original Cardputer: leaves the IMU numeric fields empty and writes
  `imu_available=0` and `imu_type=none`.

The project builds successfully against Arduino-ESP32 3.3.9, M5Cardputer tag
1.2.0, M5Unified 0.2.22 and TinyGPSPlus 1.1.0. The included app binary is ready
for Cardputer Launcher.

## Acknowledgements

A sincere thank you to
[geo-tp/M5Cardputer-GPS-Logger](https://github.com/geo-tp/M5Cardputer-GPS-Logger)
for creating the original GPS Logger and inspiring us to write this newer GPS +
IMU version. JP226-GPS-IMU-Logger retains its familiar live GPS display and
display controls while adding IMU telemetry, daily rollover, and an always-on
movement-aware logging schedule.

## Hardware

- M5Stack Cardputer ADV or original Cardputer
- M5Stack Unit GPS with AT6558, default 9600 baud
- FAT32 microSD card (32 GB or smaller is the conservative choice for Launcher)
- HY2.0-4P/Grove cable

### AT6558 wiring

The Cardputer and Cardputer ADV use the same Grove pinout.

| GPS Unit wire/pin | GPS function | Cardputer | Logger role |
|---|---|---|---|
| Black | GND | GND | Ground |
| Red | 5 V | 5 V | Power |
| Yellow | GPS RX | GPIO2 | Cardputer TX (only needed for GPS configuration) |
| White | GPS TX | GPIO1 | Cardputer RX (receives NMEA) |

With an unmodified M5Stack Grove cable, simply plug the GPS Unit into the
Cardputer's HY2.0-4P port. The firmware opens UART1 as RX=GPIO1, TX=GPIO2,
9600/8-N-1. Do not connect the GPS UART to the ADV internal I2C pins GPIO8/9.

The microSD slot uses SCK=GPIO40, MISO=GPIO39, MOSI=GPIO14 and CS=GPIO12.

Hardware/API references:

- [Cardputer ADV product and pin map](https://docs.m5stack.com/en/core/Cardputer-Adv)
- [Official Cardputer ADV IMU example](https://docs.m5stack.com/en/arduino/m5cardputer/imu)
- [M5Stack AT6558 GPS Unit](https://docs.m5stack.com/en/unit/gps)

## Open and build in VS Code

1. Open this `JP226-GPS-IMU-Logger` folder in VS Code.
2. Install the **PlatformIO IDE** extension if it is not already installed.
3. Wait for PlatformIO to fetch the pinned framework and libraries.
4. Run **PlatformIO: Build**, or use the `m5stack-stamps3` environment in the
   PlatformIO sidebar.

The build artifacts are generated under `.pio/build/m5stack-stamps3/`:

- `firmware.bin`: app-only binary for Launcher.
- `firmware.factory.bin`: merged bootloader + partition table + app for direct
  USB recovery/first-time flashing. Do not choose this one for Launcher's normal
  SD install workflow.

Prebuilt copies from the verified build are in `dist/`.

## Controls

| Key | Action |
|---|---|
| `S` | Turn the display off/on without stopping GPS monitoring |
| `-` / `=` | Decrease/increase display brightness |

CSV logging is always enabled and cannot be switched off from the keyboard. If
the SD card is missing or fails, the logger keeps retrying automatically.

The original manual light-sleep mode is intentionally replaced by display-off
mode. Light sleep pauses continuous GPS processing and therefore cannot satisfy
the requirement to detect resumed vehicle movement immediately.

## Install with Cardputer Launcher

Use `dist/JP226-GPS-IMU-Logger.bin`.

1. Copy the app binary to the FAT32 microSD card, or upload it with Launcher's
   WebUI.
2. In Launcher, open **SD**, select the `.bin`, then choose **Install**.
3. Alternatively, use **WUI** and upload/install the same app binary.
4. Reboot. Launcher can start the installed app automatically; use its boot-screen
   key/menu behaviour to return to Launcher.

Launcher accepts ordinary PlatformIO `firmware.bin` application images. See the
[Launcher binary/install guide](https://github.com/bmorcelli/Launcher/wiki/Obtaining-binaries-to-launch).

## Files and time

Logs are written to:

```text
/telemetry/telemetry_YYYY-MM-DD.csv
```

`YYYY-MM-DD` is the **local calendar date**, not UTC. A new file is opened on the
first record after local midnight. `timestamp` remains UTC (`...Z`) for stable
analysis, while `local_timestamp` includes the local UTC offset.

The default timezone is Australia/Sydney, including daylight-saving transitions:

```cpp
constexpr char kPosixTimezone[] = "AEST-10AEDT,M10.1.0,M4.1.0/3";
```

Change that value in `include/Config.h` for another location.

On the ADV, M5Unified restores system UTC from the onboard RTC at boot. A fresh
GPS date/time periodically corrects the clock and updates that RTC. The original
Cardputer has no RTC, so after a cold restart it waits for a valid GPS UTC date
and time before creating a file; this prevents wrongly dated files.

Each append is flushed immediately. If the SD card is missing or an append
fails, the logger closes the file and retries the card every 10 seconds. An
existing daily file is appended to and does not receive a second header.

## Logging state machine

| State | Entry condition | Record schedule |
|---|---|---|
| `WAITING_FIX` | Boot, before a usable fix | No CSV until time and movement are known |
| `MOVING` | One valid fresh fix at or above `kMoveStartKmh` | Immediately, then every `kMovingLogIntervalMs` |
| `STOPPED_15M` | Speed stays at or below `kMoveStopKmh` for `kStopConfirmMs` | Immediately on stop confirmation, then stop+15, +30, +45 and +60 minutes |
| `STOPPED_HOURLY` | The same stop has lasted at least one hour | At stop+2 hours, +3 hours, and hourly thereafter |
| `FIX_LOST` | No quality-approved fix for `kFixLossTimeoutMs` | Diagnostic row every `kFixLostLogIntervalMs`; GPS fields are blank/stale-safe |

Movement resumption is deliberately asymmetric: a single quality-approved speed
at or above the start threshold returns to `MOVING` immediately. Stopping is
debounced to reject low-speed GPS jitter.

The stopped-session start and next due UTC time are saved in NVS. After a restart,
a stationary fix continues a plausible saved stop schedule rather than resetting
the one-hour timer. A moving fix clears the saved stop immediately. Saved stops
older than `kPersistedStopMaxAgeSec` are rejected. Missed intervals are not
backfilled; the logger writes one current row and advances to the next boundary.

## GPS fix rules

A movement decision requires all of the following:

- fresh location and speed;
- fresh GPS UTC date and time;
- at least `kMinimumSatellites` satellites;
- valid HDOP no greater than `kMaximumHdop`;
- location age no greater than `kMaxFixAgeMs`.

Bad or stale fixes never manufacture a stop. They transition to `FIX_LOST` after
the configured timeout. `vdop` is read from either `$GNGSA` or `$GPGSA`; it is
left empty if the receiver does not emit either sentence.

## Configuration

All user-tunable constants are in `include/Config.h`. The main ones are:

| Constant | Default | Meaning |
|---|---:|---|
| `kGpsBaud` | 9600 | AT6558 UART baud |
| `kMoveStartKmh` | 3.0 km/h | Immediate moving threshold |
| `kMoveStopKmh` | 1.5 km/h | Candidate stopped threshold |
| `kStopConfirmMs` | 10 s | Low-speed dwell before stopped |
| `kMovingLogIntervalMs` | 1 s | Normal moving telemetry rate |
| `kStoppedFirstHourIntervalSec` | 15 min | First stopped-hour interval |
| `kStoppedHourlyIntervalSec` | 1 h | Long-stop interval |
| `kFixLostLogIntervalMs` | 1 min | No-fix diagnostic rate |
| `kMinimumSatellites` | 4 | Fix quality gate |
| `kMaximumHdop` | 5.0 | Fix quality gate |
| `kMaxFixAgeMs` | 3 s | Maximum location/speed age |
| `kImuSampleIntervalMs` | 10 ms | ADV IMU sampling target (100 Hz) |

The AT6558's factory NMEA update rate is normally 1 Hz, so the default moving
log rate is also 1 Hz. If the receiver is separately configured for a faster
NMEA update rate, lower `kMovingLogIntervalMs` to match it.

## CSV schema

The uploaded source file was inspected before implementation. It contains 7,191
valid 19-field data rows spanning `2026-09-26T06:01:36.500Z` through
`2026-09-26T21:27:26.000Z`, but also contains a repeated header, 871 duplicate
timestamp rows, and a trailing NUL-filled record. This logger preserves the
original first 19 columns in the same order while avoiding those structural
errors. Seven diagnostic columns are appended.

| Column | Meaning |
|---|---|
| `timestamp` | UTC ISO-8601 timestamp with milliseconds |
| `lat`, `lon` | Decimal degrees; empty if position is stale/unavailable |
| `alt_m` | GPS altitude in metres |
| `speed_kmh` | GPS ground speed |
| `heading_deg` | GPS course over ground |
| `satellites` | Satellites used |
| `hdop`, `vdop` | Horizontal/vertical dilution of precision |
| `acc_x_g`, `acc_y_g`, `acc_z_g` | ADV acceleration in g |
| `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps` | ADV angular rate in degrees/second |
| `pitch_deg`, `roll_deg` | Gravity-derived attitude; meaningful when linear acceleration is modest |
| `g_total` | Magnitude of the acceleration vector in g |
| `roughness_index` | RMS high-pass change in acceleration magnitude since the previous row, in g |
| `local_timestamp` | ISO-8601 local timestamp with UTC offset |
| `fix_valid` | `1` only when all configured GPS quality gates pass |
| `fix_age_ms` | Age of the last parsed position |
| `imu_available` | `1` on a detected ADV IMU, otherwise `0` |
| `imu_type` | Detected M5Unified IMU type (`BMI270`, `none`, etc.) |
| `log_state` | State-machine mode that caused the row |
| `uptime_ms` | 64-bit boot uptime for diagnostics |

On the original Cardputer, all ten IMU-derived numeric columns are empty. This is
intentional and keeps them numeric-friendly for CSV import.

## Project layout

```text
include/Config.h          hardware pins, thresholds and intervals
include/LogSchedule.h     pure stopped-schedule interface
src/LogSchedule.cpp       15-minute/hourly boundary calculations
src/main.cpp              GPS, IMU, clock, NVS, SD, CSV and display logic
test/test_schedule/       boundary and persisted-state unit tests
dist/                     verified Launcher and factory binaries
```

`pio test -e native` runs the schedule boundary tests on systems with a desktop
C/C++ compiler installed. The firmware build itself compiles the same scheduling
source for ESP32-S3.
