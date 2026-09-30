# AtomS3 remote GPS + IMU node

## Hardware and firmware

Use an original M5Stack AtomS3 with its MPU6886 and display, fitted to an
Atomic GPS Base A134 or A134-V2. Power the units down before fitting or removing
the base. The A134 GPS TX is on AtomS3 GPIO5 at 9600/8-N-1. A134-V2 uses GPIO5
for GPS TX, GPIO6 for GPS RX and defaults to 115200/8-N-1. The firmware scans
both rates. The base's microSD slot is not used; the Cardputer remains the
authoritative logger.

Build and flash:

```powershell
platformio run -d atoms3_gps_imu
platformio run -d atoms3_gps_imu -t upload --upload-port COMx
platformio device monitor -d atoms3_gps_imu --port COMx --baud 115200
```

The Cardputer firmware remains the root project:

```powershell
platformio run -e m5stack-stamps3
```

Use `.pio/build/m5stack-stamps3/firmware.bin` with Cardputer Launcher, or its
factory image for a direct full flash. The same binary supports the original
Cardputer, Cardputer v1.1 and Cardputer ADV through M5Unified runtime detection.

## Pairing and MAC configuration

The default is discovery without recompilation:

1. Flash and start the Cardputer and AtomS3.
2. Press `R` on the Cardputer until `ATOM REMOTE` is shown.
3. The Cardputer broadcasts a small discovery packet on its current Wi-Fi
   channel. The AtomS3 scans channels 1-13 until it receives that packet.
4. `LINK OK` appears on the AtomS3 and `Remote: CONNECTED` on the Cardputer.
5. A lost node is declared stale after 3 seconds. Both devices automatically
   resume after it returns; the AtomS3 resumes channel scanning after 5 seconds.

For an allow-list, copy the AtomS3 station MAC shown on its display/serial port
to `config::kAtomEspNowMac` in `include/Config.h`. The reciprocal optional
Cardputer address is `atom_config::kCardputerEspNowMac` in
`atoms3_gps_imu/include/AtomConfig.h`. Zero addresses enable discovery. ESP-NOW
uses station MAC addresses, not Bluetooth MAC addresses.

ESP-NOW and an infrastructure Wi-Fi connection must share a radio channel.
Discovery handles channel changes by having the AtomS3 scan for the
Cardputer's current beacon. No SSID or password is stored on the AtomS3.

## Mounting convention and zeroing

Install the AtomS3 rigidly so its axes represent:

- X: vehicle longitudinal; positive acceleration is forward.
- Y: vehicle lateral; positive acceleration is left for the documented mount.
- Z: vehicle vertical; positive acceleration is upward.

Confirm signs in the vehicle because rotating the enclosure changes them. With
the vehicle stationary on level ground, hold the AtomS3 screen/button for 1.5
seconds, then keep it still for about 2 seconds. The firmware averages 200 IMU
samples, stores installed pitch, roll and local gravity in NVS, rotates future
accelerometer and gyro measurements into that leveled installed frame, and
reports pitch/roll relative to it. `IMU 100Hz ZERO` confirms a stored zero.

The acceleration fields retain gravity on Z. Derived vertical acceleration and
one-second statistics subtract the stored gravity reference. Pitch and roll
remain acceleration-derived, so strong linear acceleration temporarily affects
them.

## Sampling and status

The IMU target is 100 Hz. The radio packet includes the latest six-axis sample
and the most recently completed one-second statistics window. GPS is parsed
continuously and packets are sent every 100 ms, preserving useful receiver
rates up to 10 Hz without sending one ESP-NOW packet per IMU sample. GPS loss
does not stop IMU sampling or transmission; validity flags and GPS age identify
the condition.

## Display pages and buttons

A short press of the AtomS3 screen cycles five pages; the choice is
remembered across restarts. Holding the screen for 1.5 seconds zeroes the
mount from any page. The small side button is the hardware reset: a short
press restarts the AtomS3 and a 2-second hold enters download mode (green
LED, screen off).

1. **Combined** - GPS fix/satellites, link state and channel, IMU zero
   state, pitch, roll and vibration, plus sequence, GPS age and station MAC.
   `SCAN` is normal until the Cardputer beacon is found.
2. **IMU** - installed-frame acceleration (m/s^2) and gyro (degrees/second),
   with X red, Y green and Z blue.
3. **GPS** - fix/satellites, latitude, longitude, speed, altitude, HDOP and
   the detected baud rate.
4. **Time** - large local time and date. UTC comes from the GPS, or from the
   paired Cardputer's clock when the GPS has no time yet; the local offset
   (including daylight saving) comes from the Cardputer. Without a
   Cardputer link the page shows UTC.
5. **Power** - the AtomS3 and Atomic GPS Base have no battery, so this shows
   `USB POWER` for the node and the Cardputer's battery percentage and
   voltage relayed over ESP-NOW.

## Optional raw IMU capture

Add `-DENABLE_RAW_IMU_LOGGING=1` to both projects. The AtomS3 then transmits
16-sample quantized batches (milli-g and 0.1 degree/second); the Cardputer
buffers 32 rows before each SD flush to `imu_YYYY-MM-DD.csv`. This is intended
for short suspension/road-analysis sessions and is deliberately disabled by
default.
