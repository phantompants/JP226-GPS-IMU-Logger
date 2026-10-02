#pragma once

#include <Arduino.h>

// Set to 1 by the m5stack-core2 build; the Cardputer build leaves it unset.
#ifndef JP226_CORE2
#define JP226_CORE2 0
#endif

namespace config {

// UART NMEA GPS inputs. The Grove port accepts the AT6558 and other standard
// NMEA 0183 receivers. Its common baud rates are scanned automatically.
#if JP226_CORE2
// Core2 DIN base black Port B (G36/G26). The logger only listens to the GPS,
// so no TX pin is assigned: G36 is input-only and must never be driven. GPS
// TX may be wired to either line, so after a full baud scan the receive pin
// swaps and the scan repeats.
constexpr int kGroveGpsRxPin = 36;
constexpr int kGroveGpsTxPin = -1;
constexpr int kGroveGpsAltRxPin = 26;
constexpr int kGroveGpsAltTxPin = -1;
#else
constexpr int kGroveGpsRxPin = 1;
constexpr int kGroveGpsTxPin = 2;
constexpr int kGroveGpsAltRxPin = -1;
constexpr int kGroveGpsAltTxPin = -1;
#endif
constexpr uint32_t kGroveGpsBaudCandidates[] = {9600, 115200, 38400,
                                                4800, 19200,  57600};
constexpr size_t kGroveGpsBaudCandidateCount =
    sizeof(kGroveGpsBaudCandidates) / sizeof(kGroveGpsBaudCandidates[0]);

// Cardputer ADV Cap LoRa-1262 onboard ATGM336H-6N GPS. The Cap is only
// initialized when M5Unified identifies a Cardputer ADV.
constexpr int kCapGpsRxPin = 15;  // Cap GPS_TX -> Cardputer ADV GPIO15
constexpr int kCapGpsTxPin = 13;  // Cardputer ADV GPIO13 -> Cap GPS_RX
constexpr uint32_t kCapGpsBaud = 115200;
constexpr int kCapLoraCsPin = 5;  // Keep SX1262 deselected while SD uses SPI.

constexpr uint32_t kGpsBaudScanIntervalMs = 2'500;
constexpr uint32_t kGpsSourceStaleMs = 5'000;
// R switches between LOCAL_GPS and ATOMS3_REMOTE at runtime and persists the
// selection. The local mode remains the default to preserve existing installs.
constexpr bool kDefaultToAtomS3Remote = false;
constexpr uint32_t kRemoteStaleMs = 3'000;
constexpr uint32_t kRemoteDiscoveryIntervalMs = 1'000;
// Leave zero for discovery, or paste the AtomS3 station MAC shown on its
// display/serial output to restrict reception to that node.
constexpr uint8_t kAtomEspNowMac[6] = {0, 0, 0, 0, 0, 0};
// Optionally restrict Dial commands to a known station MAC. Leaving zero
// accepts the first Dial seen until reboot; ESP-NOW packets are not encrypted.
constexpr uint8_t kDialEspNowMac[6] = {0, 0, 0, 0, 0, 0};

#ifndef ENABLE_RAW_IMU_LOGGING
#define ENABLE_RAW_IMU_LOGGING 0
#endif

// Optional Wi-Fi/NTP and coordinate-based timezone discovery. Credentials and
// overrides are read from kLoggerConfigPath on the SD card; they are never
// compiled into the firmware.
constexpr char kLoggerConfigPath[] = "/telemetry/logger.cfg";
constexpr char kNtpServer1[] = "pool.ntp.org";
constexpr char kNtpServer2[] = "time.google.com";
constexpr char kNtpServer3[] = "time.cloudflare.com";
constexpr char kTimezoneLookupUrl[] =
    "https://timeapi.io/api/timezone/coordinate";
constexpr uint32_t kWifiRetryIntervalMs = 60'000;
constexpr uint32_t kTimezoneLookupRetryMs = 15 * 60 * 1000UL;
constexpr uint32_t kTimezoneLookupIntervalMs = 6 * 60 * 60 * 1000UL;
constexpr uint32_t kTimezoneHttpTimeoutMs = 5'000;

// Australia/Sydney, including current daylight-saving transitions. Change this
// POSIX TZ string if the logger is used in another local time zone.
constexpr char kPosixTimezone[] = "AEST-10AEDT,M10.1.0,M4.1.0/3";

// Motion has hysteresis: movement starts immediately at/above the start value;
// stopping needs speed at/below the stop value for kStopConfirmMs.
constexpr float kMoveStartKmh = 3.0f;
constexpr float kMoveStopKmh = 1.5f;
constexpr uint32_t kStopConfirmMs = 10'000;

// Logging rates.
constexpr uint32_t kMovingLogIntervalMs = 1'000;
constexpr uint32_t kStoppedFirstHourIntervalSec = 15 * 60;
constexpr uint32_t kStoppedHourlyIntervalSec = 60 * 60;
constexpr uint32_t kFixLostLogIntervalMs = 60'000;

// GPS quality gates. A fix that fails one of these gates is marked invalid and
// is never used to make a new movement/stopped decision.
constexpr uint32_t kMaxFixAgeMs = 3'000;
constexpr uint32_t kMaxDateTimeAgeMs = 5'000;
constexpr uint32_t kFixLossTimeoutMs = 10'000;
constexpr uint8_t kMinimumSatellites = 4;
constexpr float kMaximumHdop = 5.0f;

constexpr uint32_t kImuSampleIntervalMs = 10;  // 100 Hz target
constexpr uint32_t kDisplayIntervalMs = 500;
constexpr uint32_t kSdRetryIntervalMs = 10'000;
constexpr uint32_t kClockResyncIntervalMs = 60'000;
constexpr uint32_t kPersistedStopMaxAgeSec = 30UL * 24UL * 60UL * 60UL;

#if JP226_CORE2
constexpr int kSdSckPin = 18;
constexpr int kSdMisoPin = 38;
constexpr int kSdMosiPin = 23;
constexpr int kSdCsPin = 4;
#else
constexpr int kSdSckPin = 40;
constexpr int kSdMisoPin = 39;
constexpr int kSdMosiPin = 14;
constexpr int kSdCsPin = 12;
#endif
constexpr uint32_t kSdFrequencyHz = 25'000'000;

constexpr char kLogDirectory[] = "/telemetry";
constexpr char kFilePrefix[] = "telemetry_";
constexpr char kKmlDirectory[] = "/telemetry/kml";
constexpr uint16_t kKmlLinesPerUpdate = 12;
constexpr uint16_t kKmlFlushEveryPoints = 128;
constexpr char kWebHostname[] = "jp226-logger";
// UDP port for the status broadcast read by the ESPHome car display
// (esphome/jp226-car-display.yaml). Sent every two seconds on Wi-Fi.
constexpr uint16_t kStatusBroadcastPort = 47226;
constexpr uint32_t kStatusBroadcastIntervalMs = 2000;

}  // namespace config

