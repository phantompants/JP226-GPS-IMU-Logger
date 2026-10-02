# JP226-GPS-IMU-Logger

An Arduino/PlatformIO telemetry logger for the **M5Stack Cardputer ADV**, the
**original Cardputer** and the **M5Stack Core2**. Both Cardputers run the same
compiled app binary, and M5Unified detects the board at runtime; the Core2 has
its own build with a touch interface:

- Cardputer ADV: logs the onboard BMI270 accelerometer and gyroscope.
- Original Cardputer: has no IMU of its own; leave the IMU fields empty
  (`imu_available=0`) or take motion data from an AtomS3.
- Core2: touch-screen logger with a Grove GPS on a DIN base's Port B. Uses its
  own IMU where fitted, or an AtomS3's (see [M5Stack Core2
  version](#m5stack-core2-version)).
- Any logger can take GPS plus MPU6886 IMU telemetry from an AtomS3 fitted to
  an Atomic GPS Base over ESP-NOW, and pair with an M5Dial and an Atom Echo.

The project builds successfully against Arduino-ESP32 3.3.9, M5Cardputer tag
1.2.0, M5Unified 0.2.22 and TinyGPSPlus 1.1.0. The included app binary is ready
for Cardputer Launcher.

> **Why log with a dinosaur?** Because every trip deserves a good *tracks*
> record. Our resident brontosaurus has been doing fieldwork since the
> Jurassic, and it has never once lost a fossil, sorry, a *file*.

## Acknowledgements

A sincere thank you to
[geo-tp/M5Cardputer-GPS-Logger](https://github.com/geo-tp/M5Cardputer-GPS-Logger)
for creating the original GPS Logger and inspiring us to write this newer GPS +
IMU version. JP226-GPS-IMU-Logger retains its familiar live GPS display and
display controls while adding IMU telemetry, daily rollover, and an always-on
movement-aware logging schedule.

## Dino-mite extras

Every device with a screen (Cardputer, Core2, M5Dial and AtomS3) ends with two
dinosaur pages.

**JP226PRINTS (last page): Dino Dash.** A little pixel-art brontosaurus above
**"Brought to you by JP226Prints"**. While you are driving (over 5 km/h) it
just runs along by itself. Park up (under 3 km/h) and it becomes a game: rocks
and cacti scroll in and you jump. You can land on them and ride along, so only
running into the side of one ends the game. Your score is the number of
seconds you survive, and each device saves your personal best ("NEW PB 41s!").
Think of it as a *Jurassic lark*.

| Device | Jump | Leave the page |
|---|---|---|
| Cardputer | Space, Enter or `;` | Tab / `[` `]` |
| Core2 | Tap the screen or the JUMP button | PREV / NEXT |
| M5Dial | Press or turn the knob | Hold the button |
| AtomS3 | Tap the screen | Hold the screen |

**FOSSIL RECORD (second last): the version page.** Shows the firmware's
*species* (version number), *DNA* (git commit; a `+` means it was built with
uncommitted changes) and *hatched* date, under a rotating dino one-liner. Check
it on each device to see exactly which code it is running. Test builds count
up from `v0.10` (`v0.11`, `v0.12`, ...); `v1.0` will be the first public
release, once it has been tested on every logger including the Cardputer ADV. The number lives in
`shared/Version.h`; the commit and date are stamped in at build time by
`shared/version.py`.

The Atom Echo cannot show the dinosaur, so it speaks for it instead: "T-Rex
says, rawr! Is logging" or "T-Rex says, rawr! Not logging". Short arms, big
opinions.

## Support the project

JP226-GPS-IMU-Logger was built for a charity drive. We drive a Jurassic Park
tribute Ford Ranger and raise funds for **Variety – the Children's Charity**,
which is also why there is a dinosaur in every corner of this firmware. You can
donate to our team, **The Fast and the Fossilized**, on our
[Variety 4WD Queensland fundraising page](https://www.variety4wdqld.com.au/t/thefastandthefossilized).

If you like JP226-GPS-IMU-Logger and would like to support its continued
development, donations are welcome through
[JP226 Prints](https://jp226prints.au/).

**Coming next:** a port to the **M5Stack CardputerZero** as soon as ours
arrives.

Want it on another device? Ports to other M5Stack (or ESP32) hardware are very
possible, and a sufficiently generous donation is the fastest way to get a
board to the top of the list. The dinosaur will happily learn a new screen
size; it just needs feeding.

## Not sure what to build? Ask your AI

This project supports a lot of M5Stack combinations. Copy the prompt below into
ChatGPT, Claude, Copilot, Gemini or another AI assistant, fill in the last
section, and it will suggest the best setup for the devices you already own, or
a shopping list for your budget. Check current prices and stock yourself before
buying; AI assistants can be out of date.

```text
I want to build a GPS + IMU (motion) logger using the open-source
JP226-GPS-IMU-Logger firmware for M5Stack devices. Using only the facts below,
recommend the best configuration for me.

WHAT THE FIRMWARE DOES
- A "logger" records GPS position, speed and IMU motion to CSV files on a
  microSD card, makes daily Google Earth KML files, saves waypoints, and can
  sync its clock from GPS or Wi-Fi internet time.
- Accessories talk to the logger by ESP-NOW radio (no cables for data). Several
  loggers can run side by side; each accessory pairs with one logger type.

LOGGERS (pick one per kit; each needs a microSD card)
- M5Stack Cardputer ADV: keyboard + screen, built-in BMI270 IMU. GPS from a
  Grove GPS unit, or the Cap LoRa-1262 (ADV only), which has a built-in GPS.
- M5Stack Cardputer (original v1): keyboard + screen, NO IMU and no GPS. Needs
  an AtomS3 for IMU (and its GPS base for GPS), or a Grove GPS plus an AtomS3
  for the IMU.
- M5Stack Core2 (touch screen): GPS from a Grove GPS unit. A Core2 DIN base
  adds a battery and Grove Port B (GPS goes there) and Port C. Some Core2 units
  have no IMU; then an AtomS3 provides it.

GPS AND IMU OPTIONS
- M5Stack Grove GPS unit (AT6558 or other NMEA receiver): plugs into a
  logger's Grove port. Gives GPS only.
- M5Stack AtomS3 (NOT AtomS3 Lite, which has no IMU or screen): sends its IMU
  by radio. Fitted to an Atomic GPS Base it sends GPS and IMU. It needs 5 V
  power, e.g. from the logger's Grove port or USB-C.
- Source modes on the logger: own GPS + own IMU; GPS + IMU from an AtomS3;
  own GPS + IMU from an AtomS3.

OPTIONAL ACCESSORIES (one logger can take two at once)
- M5Stack M5Dial: rotary knob + round screen. Saves categorised waypoints,
  sets road type / tyre pressures / suspension / load, shows speed and link
  status. Tap its CONNECTION page to choose which logger type it pairs with.
- M5Stack Atom Echo: spoken alerts (fix lost, logger lost, SD error, low
  battery, logging on/off) and a one-button waypoint marker. It can sit on an
  Atomic SPK base for a louder speaker, but the base's SD slot must stay
  EMPTY: its SD pins clash with the Echo's audio.
- Not useful here: Atom Lite / Atom Matrix as a relay (adds nothing), and the
  Atomic SPK base's SD card with an Atom Echo.

THINGS EVERY KIT NEEDS
- One logger, one GPS source, a microSD card (FAT32, 32 GB or smaller is the
  safe choice), and an IMU source if you want motion data.

PLEASE GIVE ME
1. The best kit (or kits) I can build, with what plugs into what.
2. The source mode to select on each logger.
3. Anything I am missing, with a rough price, and the cheapest way to fill
   the gap.
4. Any combinations to avoid.

MY SITUATION (fill in one or both)
- Devices I already have: [e.g. Cardputer v1, AtomS3 with GPS base, M5Dial]
- Or my budget and country: [e.g. AUD 150, Australia]
- What I want to log: [e.g. 4WD trips, motorbike rides, cycling, testing]
```

## Hardware

- M5Stack Cardputer ADV or original Cardputer
- One supported UART NMEA GPS source:
  - M5Stack Unit GPS with AT6558 (or another Grove NMEA receiver), or
  - M5Stack Cap LoRa-1262 onboard ATGM336H-6N GPS on Cardputer ADV
- Optional remote source: M5Stack AtomS3 (not AtomS3 Lite) plus Atomic GPS Base
  A134 or A134-V2. The AtomS3 needs USB/5 V power but no microSD card.
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
- [AtomS3 product and MPU6886 details](https://docs.m5stack.com/en/core/AtomS3)
- [Atomic GPS Base](https://docs.m5stack.com/en/atom/Atomic%20GPS%20Base)
- [Atomic GPS Base v2.0](https://docs.m5stack.com/en/atom/Atomic_GPS_Base_v2.0)

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

### Build and flash the AtomS3 remote

The remote is a separate PlatformIO project so it cannot accidentally be
flashed to a Cardputer:

```powershell
platformio run -d atoms3_gps_imu
platformio run -d atoms3_gps_imu -t upload --upload-port COMx
```

Fit the AtomS3 to the powered-off Atomic GPS Base, connect USB-C, replace
`COMx` with its serial port and upload. The display and serial console show its
Wi-Fi station MAC, GPS/IMU state, radio channel and link status. Both A134
(9600 baud) and A134-V2 (115200 baud) are auto-detected. The firmware consumes
NMEA as quickly as the installed receiver produces it and publishes at up to
10 Hz; it does not force a receiver-specific update-rate command.

See [Remote node setup](docs/remote-node.md) for mounting, zeroing, pairing,
flashing and troubleshooting, and [Telemetry protocol](docs/telemetry-protocol.md)
for the packed wire format.

## M5Stack Core2 version

The same logger also builds for the **M5Stack Core2** (v1.0 with MPU6886 or
v1.1 with BMI270). It keeps everything the Cardputer does (CSV logging, KML
export, waypoints, web portal, AtomS3 remote and M5Dial remote) and replaces
the keyboard with a touch UI sized for the 320x240 screen.

**Hardware:** fit the Core2 to a DIN base and plug an M5Stack GPS Unit (AT6558
or another NMEA receiver) into the black **Port B** Grove socket. The firmware
listens on G36, scans the common baud rates, then tries G26 in case the lines
are the other way round, so no wiring choice is needed. It never transmits to
the GPS, because G36 is input-only. An AtomS3 IMU node can take its power from
**Port C**; its data travels by radio, not over the cable. Logs go to the Core2's own microSD slot. The Core2's
battery-backed clock keeps UTC across power-off, so logging resumes before the
GPS has a fix.

**Build and flash** over USB (the Core2 resets itself into download mode):

```powershell
platformio run -e m5stack-core2
platformio run -e m5stack-core2 -t upload --upload-port COMx
```

**Controls:** the three touch buttons under the screen are **< PREV**, the
page's action (shown in yellow in the footer) and **NEXT >**. Hold the middle
button to turn the screen off; touch anywhere to turn it back on.

| Page | Shows | Middle button / touch |
|---|---|---|
| DRIVE | Large speed, fix, log state, current place, trip time | MARK: save a GENERIC waypoint |
| HUD | Mirrored speed for a windscreen reflection | MARK |
| GPS | Fix, position, altitude, heading, HDOP, receiver and baud | SOURCE: cycles LOCAL GPS, ATOM REMOTE and GPS+ATOM IMU |
| IMU | Acceleration and rotation per axis (X red, Y green, Z blue), pitch, roll, vibration | - |
| WAYPOINT | Tap one of nine types to save a waypoint here | LAST: details of the last waypoint |
| LOGGER | SD, log state, rows, current file, KML export progress | KML SCAN |
| TIME | Local time and date, time zone, Wi-Fi and web address | - |
| SETUP | Source, AtomS3 and M5Dial link, GPS receiver, IMU type, brightness | Touch the on-screen buttons |

**No IMU?** Some Core2 units have no IMU in the main unit (the serial report
lists no device at 0x68/0x69). Select **GPS+ATOM IMU** to keep the Grove GPS
for position and take motion data from an AtomS3 running the remote firmware;
the AtomS3 needs no GPS base for this. The original Cardputer, which also has
no IMU, has the same option on its `R` key. CSV rows record the source as
`LOCAL_GPS_ATOM_IMU` and the IMU type as `MPU6886_REMOTE`.

**M5Dial:** the Core2 accepts a Dial just as the Cardputer does, but the Dial
firmware as shipped pairs only with the Cardputer ADV (see the kit layout
below). Change `kLoggerBoard` in `m5dial_waypoint/src/main.cpp` to
`LoggerBoard::Core2` to use it here instead.

**Wi-Fi:** there is no keyboard for the Wi-Fi setup page, so put the network
name and password in `/telemetry/logger.cfg` on the SD card (see
`logger.cfg.example`). Waypoint names and notes cannot be typed on the Core2;
the waypoint type is recorded instead.

## Running several loggers

Each logger announces its board type in its discovery beacon, and every
accessory pairs only with its own type, so three kits can run side by side, a
small herd that never wanders into the wrong *Rex-tangle*:

| Kit | Logger | GPS | IMU | Accessories |
|---|---|---|---|---|
| 1 | Cardputer ADV | Grove GPS Unit | Built in | M5Dial, Atom Echo |
| 2 | Core2 on a DIN base | Grove GPS Unit on Port B | AtomS3 on SPK base, by radio | AtomS3 powered from Port C |
| 3 | Cardputer v1 | AtomS3 on Atomic GPS Base, by radio | Same AtomS3 | AtomS3 powered from the Grove port |

Set the source to `OWN GPS` / `LOCAL GPS` on kit 1, `GPS+ATOM IMU` on kit 2
and `ATOM REMOTE` on kit 3. The Grove cables to the AtomS3s carry power only.

- **AtomS3:** each one's logger is listed by its own MAC in
  `kLoggerAssignments` in `atoms3_gps_imu/include/AtomConfig.h`. An unlisted
  AtomS3 pairs with the first logger it hears.
- **M5Dial:** tap on its CONNECTION page to choose CARDPUTER ADV, CARDPUTER,
  CORE2 or ANY LOGGER. The choice is saved, so moving it to another kit needs
  no reflash.
- **Atom Echo:** hold its main button and press the small reset button on the
  side. It restarts, steps to the next logger and says which one ("Pairs with
  Cardputer ADV", "Cardputer", "Core 2" or "any logger"). The choice is saved.
- **Logger:** accepts up to two controllers at once (Dial and Echo).

Loggers with firmware older than this report no board type, and accessories
pair with them as before.

## Atom Echo alerts

`atom_echo/` turns an M5Stack Atom Echo into a speaking companion for the
Cardputer ADV. It works on its own or on an Atomic SPK base: at start-up it
detects the base and plays through the base's louder speaker instead. Leave the
base's SD slot empty. Its SD card is wired to the Echo's microphone and speaker
pins, so the firmware never uses it.

| Button | Action |
|---|---|
| Tap | Save a `MARK` waypoint at the logger's position; it says "Waypoint saved" or "Waypoint not saved" |
| Hold 1 s (tick), release | Spoken status: fix, logging, SD card and logger battery |
| Hold 4 s (second tick), release | Mute or unmute the automatic alerts; remembered across restarts |
| Hold, then press the side reset button | Choose the next logger to pair with, spoken aloud |

Automatic alerts: logger connected or lost, logging started or stopped ("T-Rex
says, rawr! Is logging" / "Not logging"), GPS fix gained or lost, AtomS3 link
(only when the logger uses one), SD card error or recovery, and logger battery
under 20% and 10%. A change must last 3 seconds before it is spoken, so a brief
dropout will not make it *dino-sore*.

Light: blinking blue while searching, amber when linked without a fix, green
with a fix, white while waiting for a waypoint confirmation, and a short purple
blink every 3 seconds when muted.

```powershell
platformio run -d atom_echo -t upload --upload-port COMx
```

The clips are generated with Windows text-to-speech. To change the wording,
edit the phrases in `atom_echo/tools/make_clips.ps1` and run it from the
`atom_echo` folder; it rewrites `src/Clips.h`.

## Controls

| Key | Action |
|---|---|
| `Tab`, `]` or Fn+right | Next large-text dashboard page |
| `[` or Fn+left | Previous dashboard page |
| `1`–`9` | Open dashboard pages 1–9 directly |
| `0` | Open the KML export page (page 10) |
| `G` | Cycle GPS source (`AUTO`/`GROVE`/`CAP` on ADV) |
| `R` | On GPS SOURCE: GPS from this unit or the AtomS3. On IMU SOURCE: IMU from this unit or the AtomS3 (GPS from the AtomS3 always brings its IMU). Elsewhere: cycle `LOCAL GPS`, `ATOM REMOTE` and `GPS+ATOM IMU` |
| `W` | Open the Wi-Fi setup page and scan for nearby networks |
| `K` | Open the KML page and rescan all finished days for missing exports |
| `P` | Open the WAYPOINT page |
| `A` on WAYPOINT | Save a waypoint immediately from the selected GPS source |
| `N`, `T`, `F` on WAYPOINT | Edit the last waypoint's name, note or photo reference |
| `V` on WAYPOINT | Toggle last-waypoint details |
| `S` | Turn the display off/on without stopping GPS monitoring |
| `-` / `=` | Decrease/increase display brightness |

The selected page is remembered across restarts. The Cardputer battery
percentage appears at the top right of each titled page, and the Logger page
also shows its voltage. M5Unified reads the battery through an ADC divider and
cannot detect charging. The eleven pages are:

1. Combined GPS/IMU summary
2. Large current speed
3. Horizontally mirrored HUD speed for windscreen reflection
4. GPS fix, coordinates, satellites, HDOP, receiver and timezone status
5. ADV IMU acceleration, orientation and gyroscope status
6. GPS source setup and baud/signal status
7. Wi-Fi connection status and setup shortcut
8. Logger, SD card, row count, file and parked-schedule status
9. Large local clock, date and timezone status
10. Daily CSV-to-KML export status and manual rescan
11. Waypoint creation, editing and last-location details

Waypoints are saved independently of the moving/parked logging schedule.
Their original ID, timestamp and coordinates survive later text edits.
An optional M5Dial remote can create categorized waypoints and set the vehicle
context that is captured with each waypoint. See
[Waypoint and Dial setup](docs/waypoints.md).

Dashboard and Wi-Fi setup frames are drawn off-screen and transferred to the
LCD in one operation to prevent visible clearing/flicker. Wi-Fi results use
large `ENTER/Q: QUIT` and `R: START AGAIN` actions. The HUD digits are rendered
directly as mirrored seven-segment shapes, which works on both the original
Cardputer and Cardputer ADV without relying on display scaling support.

The HUD page is intentionally mirrored on the Cardputer screen. Position it
flat near the windscreen only where it is legal and cannot obstruct the
driver's view. It is a convenience display, not a calibrated vehicle
speedometer. GPS speed may lag or be unavailable in tunnels and poor reception.

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

### Starlink/Wi-Fi setup

Press `W` on the logger screen to open the built-in Wi-Fi setup page. It scans
for nearby networks without stopping GPS or SD logging. Use Fn+up/down or the
`,`/`.` keys to move, press Enter to select a network, type its password, then
press Enter again to save and connect. Use `R` to rescan and `Q` to close the
page. Fn+backtick acts as Escape/back while entering a password.

Credentials entered on the device are stored in ESP32 non-volatile storage and
reused after restarts. The password is masked on screen. Like most ESP32
applications, NVS is not encrypted unless flash encryption has separately been
enabled, so physical access to the device should be treated as access to its
saved credentials.

The SD configuration method remains available as an optional first-boot or
recovery method. On-device saved credentials take priority. Copy
`logger.cfg.example` to `/telemetry/logger.cfg` on the logging microSD card,
then edit it:

```ini
wifi_ssid=Your Starlink WiFi name
wifi_password=Your Starlink WiFi password
timezone_auto=true
# timezone=AEST-10AEDT,M10.1.0,M4.1.0/3
```

The logger never requires Wi-Fi to record. Wi-Fi is used for NTP, the optional
coordinate-to-timezone lookup, and the local file portal described below. The
configuration file stores the Wi-Fi password as plain text on the SD card, so
keep the card private. Automatic timezone lookup sends the current GPS
coordinates to `timeapi.io`; set `timezone_auto=false` to disable this.

On the ADV, M5Unified restores system UTC from the onboard RTC at boot. A fresh
GPS or NTP time periodically corrects that RTC. The original Cardputer has no
RTC, so after a cold restart it waits for valid GPS or NTP time before creating
a file; this prevents wrongly dated files.

Each append is flushed immediately. If the SD card is missing or an append
fails, the logger closes the file and retries the card every 10 seconds. An
existing daily file is appended to and does not receive a second header.

## iPhone and iPad file portal

When the Cardputer is connected to Wi-Fi, it serves a responsive local web page
designed for an iPhone 16 Pro Max and a 12.9-inch iPad Pro. Join the phone or
tablet to the same travel-router network, then open:

```text
http://jp226-logger.local
```

The Wi-Fi dashboard page explicitly shows `WEB: HTTP PORT 80`, the Cardputer's
numeric IP address, and `http://jp226-logger.local`. If the router blocks mDNS
or multicast discovery, use the numeric IP instead, for example
`http://192.168.1.42/`. Do not use `https://`.

The portal provides:

- large, touch-friendly GPS, logging, SD and KML status cards;
- the latest speed, local clock, timezone and connected Wi-Fi network;
- a newest-first list of daily CSV and KML files on the microSD card;
- direct CSV/KML download buttons; and
- a button to rescan completed CSV days for missing KML files.

The file portal is deliberately read-only: it cannot upload, rename or delete
files. Downloads are accepted only after the logger has positively entered
`STOPPED_15M` or `STOPPED_HOURLY`. This prevents a large synchronous SD/network
transfer from delaying NMEA processing while driving. If KML conversion is
requested while moving, the request remains queued and starts automatically
after the vehicle is confirmed stopped. Movement resuming pauses conversion
immediately and it resumes after the vehicle is stopped again. If power is lost,
an incomplete `.tmp` export is discarded and rebuilt on a later stopped scan.

This is a local-network interface, not a cloud service, and it does not need
working Starlink Internet once the devices are connected to the same router.
It has no login, so use it only on a trusted travel-router network. Do not add a
public Internet port-forward to the Cardputer. A properly configured VPN is the
safer option if access is ever needed from outside the vehicle network.

## Google Earth and Google Maps KML export

Dinosaurs left tracks for palaeontologists; this logger leaves them for Google
Earth. Every finished local-calendar-day CSV is automatically converted to:

```text
/telemetry/kml/telemetry_YYYY-MM-DD.kml
```

The current day's CSV is deliberately skipped because it is still being
written. At local midnight the logger closes the old file, queues it for
conversion, and continues logging to the new date. On every boot it also scans
for older CSV files that do not yet have a KML. Press `K` at any time to open
page 10 and run that scan manually.

Conversion runs only while the vehicle is confirmed stopped. It is streamed a
few CSV rows at a time so a large file is never loaded into RAM, and it pauses
as soon as movement resumes. Only valid latitude and longitude rows are
included. Current 26-column logs require `fix_valid=1`; older compatible logs
without that column are accepted when their `lat` and `lon` values are valid.
The route is stored as a ground-clamped KML `LineString`, using the required
`longitude,latitude,altitude` coordinate order.

The exporter writes a `.tmp` file first and renames it only after the complete
KML is safely closed. A restart or power loss therefore cannot leave a partial
file with a finished `.kml` name. If travel across timezones causes a date to be
reopened, its previous KML is removed and regenerated after that date is
finished again.

Open the `.kml` directly in Google Earth. For Google Maps, import it into
[Google My Maps](https://support.google.com/mymaps/answer/3024836), then open the
saved My Map from Google Maps. The generated structure follows Google's
[KML `LineString` reference](https://developers.google.com/kml/documentation/kmlreference#linestring).

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
| `kWebHostname` | `jp226-logger` | Local mDNS hostname for the file portal |
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
| `kKmlLinesPerUpdate` | 12 | CSV rows converted during each non-blocking pass |
| `kKmlFlushEveryPoints` | 128 | KML points written between SD flushes |

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
| `roughness_index` | Local ADV: legacy RMS high-pass acceleration magnitude; AtomS3: vertical vibration RMS, both in g |
| `local_timestamp` | ISO-8601 local timestamp with UTC offset |
| `fix_valid` | `1` only when all configured GPS quality gates pass |
| `fix_age_ms` | Age of the last parsed position |
| `imu_available` | `1` on a detected ADV IMU, otherwise `0` |
| `imu_type` | Detected M5Unified IMU type (`BMI270`, `none`, etc.) |
| `log_state` | State-machine mode that caused the row |
| `uptime_ms` | 64-bit boot uptime for diagnostics |
| `telemetry_source` | `LOCAL_GPS` or `ATOMS3_REMOTE` |
| `remote_sequence` | AtomS3 packet sequence; empty for local telemetry |
| `accel_x_mps2`, `accel_y_mps2`, `accel_z_mps2` | Installed-frame acceleration in m/s^2 |
| `accel_rms_mps2` | One-second dynamic acceleration RMS |
| `vertical_accel_rms_mps2` | One-second vertical dynamic acceleration RMS |
| `vertical_accel_peak_pos_mps2`, `vertical_accel_peak_neg_mps2` | One-second signed vertical peaks |
| `lateral_accel_peak_mps2`, `longitudinal_accel_peak_mps2` | One-second absolute vehicle-axis peaks |
| `vibration_rms_mps2` | One-second vertical vibration/roughness RMS |
| `imu_samples` | Samples contributing to the statistics interval |
| `gps_age_ms`, `packet_age_ms` | Source fix age and Cardputer receive age |
| `packets_lost`, `duplicate_packets`, `crc_errors` | Cumulative link diagnostics |
| `remote_tx_failures` | AtomS3 cumulative ESP-NOW send failures |
| `poi`, `poi_source` | Current user label/waypoint or automatic place, and its origin |
| `auto_place` | Independently retained automatic locality name |
| `waypoint_id` | ID of the nearby user waypoint, if one has priority |

On the original Cardputer in `LOCAL_GPS`, IMU numeric columns remain empty. In
`ATOMS3_REMOTE`, both original Cardputer revisions receive the AtomS3 IMU data.
The normal CSV still follows the existing moving/15-minute/hourly schedule; it
does not write at the 100 Hz IMU sample rate.

### Optional raw IMU file

Set `-DENABLE_RAW_IMU_LOGGING=1` in both PlatformIO projects to transmit
16-sample batches and create `/telemetry/imu_YYYY-MM-DD.csv`. Cardputer writes
32 rows per buffered flush. This is off by default because 100 Hz data grows
quickly and increases SD wear. The normal telemetry CSV is unchanged when the
option is disabled.

## Architecture and future Cardputer ADV mode

Acquisition is separated from the logger as:

```text
local UART GPS + local IMU ---\
                                > NormalizedTelemetry -> movement rules -> CSV/UI
AtomS3 ESP-NOW receiver -------/
```

`TelemetrySource::CardputerAdv` is reserved so a future ADV implementation can
combine a directly connected AT6558 with its internal BMI270 without changing
the CSV, movement scheduling or display consumers. The current ADV local mode
continues to work as before; no ADV-only dependency was added to original
Cardputer builds.

Known limitations: ESP-NOW is unencrypted in this first version; set the MAC
allow-lists to reject unrelated nodes. Discovery follows the Cardputer's
current 2.4 GHz channel, including after it joins Wi-Fi, but some access points
with aggressive channel steering can extend reconnection time. GPS update rate
is limited by the receiver's saved NMEA configuration. Hardware-in-loop GPS,
radio-range and axis-sign validation should be completed in the installed
vehicle before relying on derived suspension metrics.

## Project layout

```text
include/Config.h          hardware pins, thresholds and intervals
include/KmlExporter.h     restart-safe streaming KML export interface
include/LocationTime.h    Wi-Fi/NTP/location-timezone interface
include/LogSchedule.h     pure stopped-schedule interface
include/TelemetryData.h   source-neutral GPS, IMU and link data model
include/RemoteTelemetryReceiver.h  ESP-NOW receive/discovery interface
include/WebPortal.h       responsive read-only CSV/KML portal interface
src/LocationTime.cpp      optional network time and timezone implementation
src/KmlExporter.cpp       daily CSV discovery and Google KML generation
src/LogSchedule.cpp       15-minute/hourly boundary calculations
src/RemoteTelemetryReceiver.cpp  packet validation and normalized reception
src/WebPortal.cpp         local status, file listing and download web server
src/main.cpp              GPS, IMU, clock, NVS, SD, CSV and display logic
shared/TelemetryProtocol.h  versioned 136-byte packet, CRC and raw batch format
atoms3_gps_imu/           independent AtomS3 + Atomic GPS Base firmware
m5dial_waypoint/           independent M5Dial control firmware
atom_echo/                Atom Echo spoken alerts and waypoint button
docs/                     remote setup and telemetry protocol reference
logger.cfg.example        optional Starlink/Wi-Fi and timezone configuration
test/test_schedule/       boundary and persisted-state unit tests
dist/                     verified Launcher and factory binaries
```

`pio test -e native` runs the schedule boundary tests on systems with a desktop
C/C++ compiler installed. The firmware build itself compiles the same scheduling
source for ESP32-S3.
