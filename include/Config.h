#pragma once

#include <Arduino.h>

namespace config {

// AT6558 on the Cardputer HY2.0-4P/Grove connector:
// module TX (white) -> Cardputer GPIO1 (RX)
// module RX (yellow) -> Cardputer GPIO2 (TX)
constexpr int kGpsRxPin = 1;
constexpr int kGpsTxPin = 2;
constexpr uint32_t kGpsBaud = 9600;

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

}  // namespace config

