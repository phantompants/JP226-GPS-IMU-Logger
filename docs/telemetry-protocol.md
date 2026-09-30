# ESP-NOW telemetry protocol v2

Version 2 added the Cardputer clock, UTC offset and battery to the discovery
beacon and the Cardputer battery to the Dial status packet. Devices running
v1 and v2 ignore each other, so reflash the Cardputer, AtomS3 and Dial
together.

`shared/TelemetryProtocol.h` is the authoritative definition. Every structure
is packed, uses explicit-width integer types, has a `JP22` magic value,
protocol version, packet type, byte size and CRC-32. CRC is IEEE polynomial
`0xEDB88320`, calculated over every byte except the final CRC field. Multi-byte
values use the native little-endian representation shared by the ESP32-S3
peers. Version or size mismatches are rejected rather than partially decoded.

## Main telemetry packet

`TelemetryPacket` is 136 bytes, comfortably below the ESP-NOW v1 250-byte
limit. It contains:

- sequence, AtomS3 uptime and sample time;
- GPS UTC epoch milliseconds, age and per-field validity flags;
- latitude/longitude (`double`), altitude m, speed km/h, course degrees,
  satellites and HDOP;
- current installed-frame acceleration m/s^2 and gyro degrees/second;
- zero-relative pitch/roll degrees;
- one-second acceleration RMS, vertical RMS, positive/negative vertical peak,
  lateral/longitudinal absolute peak and vibration RMS;
- aggregation sample count and cumulative AtomS3 transmit failures.

Sequence numbers increment for each transmitted telemetry packet. The
Cardputer drops duplicates and out-of-order packets, counts positive sequence
gaps as packet loss, adds local packet age to GPS age, and marks all remote data
stale after `kRemoteStaleMs`. A packet with a lower AtomS3 uptime (node
restart), a different source MAC after the link went stale (replacement node),
or any packet after a stale gap starts a new sequence stream instead of being
discarded as out-of-order. CRC and protocol-version errors have independent
counters.

## Discovery and recovery

The Cardputer sends a 30-byte broadcast `DiscoveryPacket` once per second. It
carries the Cardputer's UTC time, current local UTC offset in minutes
(including daylight saving), validity flags and battery percentage/millivolts
(`-1` percent when unmeasured). An
unpaired AtomS3 scans Wi-Fi channels 1-13, validates the beacon and adds its
source as an unencrypted peer. This also lets it follow a Cardputer that changes
channel when joining Wi-Fi. The AtomS3 removes an unresponsive peer and returns
to scanning. MAC allow-lists can constrain either side.

Encryption is off in v2. Peer creation and packet validation are isolated from
the telemetry model so a later LMK/PMK or application authentication tag can be
added without coupling logging to the pairing method.

## Optional raw batch

`RawImuBatchPacket` is 248 bytes and carries 16 samples. Each sample contains
signed milli-g accelerometer axes, signed 0.1 degree/second gyro axes and a
100-microsecond offset from the batch start. It is only transmitted when
`ENABLE_RAW_IMU_LOGGING=1`; normal telemetry never depends on it.

## Dial control packets

The separate M5Dial firmware uses the same discovery beacon and CRC rules,
but not the AtomS3 telemetry peer allow-list. A `DialCommandPacket` (56 bytes)
contains a sequence, action, optional 32-byte text and optional numeric
value. Actions include heartbeat, waypoint creation and vehicle-context
changes. The Cardputer sends a `DialStatusPacket` (152 bytes) every 500 ms
with speed, fix, satellite count, log state/elapsed time, place, last waypoint
ID, context and Cardputer battery. A `DialAckPacket` (44 bytes) confirms a command and returns
the waypoint ID when creation succeeds.

The Dial retries unacknowledged commands with the same sequence. The
Cardputer suppresses duplicate execution and repeats the previous
acknowledgement. The optional `kDialEspNowMac` restricts which Dial may issue
commands. CRC detects accidental corruption, not forgery: this v2 protocol
is not encrypted or authenticated.
