#pragma once

#include <Arduino.h>

namespace config {

// UART NMEA GPS inputs. The Grove port accepts the AT6558 and other standard
// NMEA 0183 receivers. Its common baud rates are scanned automatically.
constexpr int kGroveGpsRxPin = 1;
constexpr int kGroveGpsTxPin = 2;
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

constexpr int kSdSckPin = 40;
constexpr int kSdMisoPin = 39;
constexpr int kSdMosiPin = 14;
constexpr int kSdCsPin = 12;
constexpr uint32_t kSdFrequencyHz = 25'000'000;

constexpr char kLogDirectory[] = "/telemetry";
constexpr char kFilePrefix[] = "telemetry_";
constexpr char kKmlDirectory[] = "/telemetry/kml";
constexpr uint16_t kKmlLinesPerUpdate = 12;
constexpr uint16_t kKmlFlushEveryPoints = 128;

}  // namespace config

