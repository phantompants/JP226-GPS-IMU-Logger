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

## Support the project

If you like JP226-GPS-IMU-Logger and would like to support its continued
development, donations are welcome through
[JP226 Prints](https://jp226prints.au/).

## Hardware

- M5Stack Cardputer ADV or original Cardputer
- One supported UART NMEA GPS source:
  - M5Stack Unit GPS with AT6558 (or another Grove NMEA receiver), or
  - M5Stack Cap LoRa-1262 onboard ATGM336H-6N GPS on Cardputer ADV
- FAT32 microSD card (32 GB or smaller is the conservative choice for Launcher)
- HY2.0-4P/Grove cable when using an external GPS Unit

### AT6558 wiring

The Cardputer and Cardputer ADV use the same Grove pinout.

| GPS Unit wire/pin | GPS function | Cardputer | Logger role |
|---|---|---|---|
| Black | GND | GND | Ground |
| Red | 5 V | 5 V | Power |
| Yellow | GPS RX | GPIO2 | Cardputer TX (only needed for GPS configuration) |
| White | GPS TX | GPIO1 | Cardputer RX (receives NMEA) |

With an unmodified M5Stack Grove cable, simply plug the GPS Unit into the
Cardputer's HY2.0-4P port. The firmware uses UART1 as RX=GPIO1, TX=GPIO2 and
automatically scans the common NMEA baud rates 9600, 115200, 38400, 4800,
19200 and 57600. This supports the AT6558 plus other NMEA 0183 UART receivers.
Do not connect a GPS UART to the ADV internal I2C pins GPIO8/9.

### Cardputer ADV Cap LoRa-1262 GPS

The ADV can instead use the ATGM336H-6N GPS built into the
[M5Stack Cap LoRa-1262](https://docs.m5stack.com/en/cap/Cap_LoRa-1262). Attach
the Cap to the ADV expansion header before powering on. Its GPS runs at
115200/8-N-1 on UART2:

| Cap signal | Cardputer ADV pin | Logger role |
|---|---:|---|
| GPS_TX | GPIO15 | Cardputer RX |
| GPS_RX | GPIO13 | Cardputer TX |

The logger monitors the Cap GPS and Grove GPS concurrently on the ADV. `AUTO`
prefers a live Cap GPS and falls back to a live Grove GPS. Press `G` to cycle
`AUTO`, `GROVE` and `CAP`; the selection is saved across restarts. On the
original Cardputer, `G` cycles only `AUTO` and `GROVE`.

> **Cap safety:** Install the supplied LoRa antenna before powering the Cap,
> even though this logger does not transmit LoRa. M5Stack warns that powering
> the Cap without its antenna can permanently damage the hardware. Power the
> Cardputer off before attaching or removing the Cap.

The Cap SX1262 and microSD share SPI pins. The firmware holds the unused LoRa
NSS/CS pin (GPIO5) high so normal SD logging remains available.

The microSD slot uses SCK=GPIO40, MISO=GPIO39, MOSI=GPIO14 and CS=GPIO12.

Hardware/API references:

- [Cardputer ADV product and pin map](https://docs.m5stack.com/en/core/Cardputer-Adv)
- [Official Cardputer ADV IMU example](https://docs.m5stack.com/en/arduino/m5cardputer/imu)
- [M5Stack AT6558 GPS Unit](https://docs.m5stack.com/en/unit/gps)
- [M5Stack Cap LoRa-1262 pin map and GPS specifications](https://docs.m5stack.com/en/cap/Cap_LoRa-1262)

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
| `G` | Cycle GPS source (`AUTO`/`GROVE`/`CAP` on ADV) |
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

`YYYY-MM-DD` is the **resolved local calendar date**, not UTC. A new file is
opened on the first record after local midnight. If the timezone changes while
travelling, the next record is routed to the correct local-date file.
`timestamp` remains UTC (`...Z`) for stable analysis, while `local_timestamp`
includes the active local UTC offset.

Time and timezone are resolved independently:

1. GPS provides authoritative UTC whenever a fresh NMEA date/time is available.
2. If Starlink or another configured Wi-Fi network is available, NTP also keeps
   UTC synchronized.
3. With `timezone_auto=true`, a fresh GPS position is sent to `timeapi.io` while
   the vehicle is stopped. The returned timezone is applied and saved in NVS.
4. Offline, the last successfully resolved timezone is reused.
5. With no saved result, the fallback is Australia/Sydney, including DST:

```cpp
constexpr char kPosixTimezone[] = "AEST-10AEDT,M10.1.0,M4.1.0/3";
```

For Australian zones, the firmware maps the returned IANA zone to full POSIX
DST rules. Elsewhere it uses the service's current UTC offset and refreshes it
every six hours when stopped and online. If automatic location lookup is not
wanted, set `timezone_auto=false` and provide a POSIX `timezone=` rule.

### Optional Starlink/Wi-Fi configuration

Copy `logger.cfg.example` to `/telemetry/logger.cfg` on the logging microSD card,
then edit it:

```ini
wifi_ssid=Your Starlink WiFi name
wifi_password=Your Starlink WiFi password
timezone_auto=true
# timezone=AEST-10AEDT,M10.1.0,M4.1.0/3
```

The logger never requires Wi-Fi to record. Wi-Fi is used only for NTP and the
optional coordinate-to-timezone lookup. The configuration file stores the
Wi-Fi password as plain text on the SD card, so keep the card private. Automatic
timezone lookup sends the current GPS coordinates to `timeapi.io`; set
`timezone_auto=false` to disable this.

On the ADV, M5Unified restores system UTC from the onboard RTC at boot. A fresh
GPS or NTP time periodically corrects that RTC. The original Cardputer has no
RTC, so after a cold restart it waits for valid GPS or NTP time before creating
a file; this prevents wrongly dated files.

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
| `kGroveGpsBaudCandidates` | 9600, 115200, 38400, 4800, 19200, 57600 | Grove NMEA auto-detection order |
| `kCapGpsBaud` | 115200 | Cap LoRa-1262 ATGM336H baud |
| `kGpsBaudScanIntervalMs` | 2.5 s | Time spent testing each Grove baud |
| `kGpsSourceStaleMs` | 5 s | Source failover/detection timeout |
| `kTimezoneLookupIntervalMs` | 6 h | Online location-timezone refresh interval |
| `kTimezoneLookupRetryMs` | 15 min | Retry delay after an unavailable lookup |
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
include/LocationTime.h    Wi-Fi/NTP/location-timezone interface
include/LogSchedule.h     pure stopped-schedule interface
src/LocationTime.cpp      optional network time and timezone implementation
src/LogSchedule.cpp       15-minute/hourly boundary calculations
src/main.cpp              GPS, IMU, clock, NVS, SD, CSV and display logic
logger.cfg.example        optional Starlink/Wi-Fi and timezone configuration
test/test_schedule/       boundary and persisted-state unit tests
dist/                     verified Launcher and factory binaries
```

`pio test -e native` runs the schedule boundary tests on systems with a desktop
C/C++ compiler installed. The firmware build itself compiles the same scheduling
source for ESP32-S3.
