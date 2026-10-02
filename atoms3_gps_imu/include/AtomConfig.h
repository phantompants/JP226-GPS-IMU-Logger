#pragma once

#include <Arduino.h>

#include "TelemetryProtocol.h"

namespace atom_config {

// Atomic GPS Base (A134) GPS TX is connected to AtomS3 GPIO5. The original
// base is receive-only from the Atom's perspective; v2 also exposes RX on G6.
constexpr int kGpsRxPin = 5;
constexpr int kGpsTxPin = 6;
constexpr uint32_t kGpsBaudCandidates[] = {9600, 115200};
constexpr size_t kGpsBaudCandidateCount =
    sizeof(kGpsBaudCandidates) / sizeof(kGpsBaudCandidates[0]);
constexpr uint32_t kGpsBaudScanIntervalMs = 5'000;
constexpr uint32_t kGpsFreshMs = 3'000;

constexpr uint32_t kImuSampleIntervalUs = 10'000;  // 100 Hz target
constexpr uint32_t kStatisticsIntervalMs = 1'000;
constexpr uint32_t kTelemetryIntervalMs = 100;  // Preserve GPS rates up to 10 Hz
constexpr uint32_t kDisplayIntervalMs = 500;
constexpr uint32_t kLinkTimeoutMs = 5'000;
constexpr uint32_t kChannelHopIntervalMs = 250;

// Optional Cardputer allow-list. Zero uses discovery beacons. Set this to the
// Cardputer station MAC to reject beacons from any other Cardputer.
constexpr uint8_t kCardputerEspNowMac[6] = {0, 0, 0, 0, 0, 0};

// Which logger each AtomS3 belongs to, by its own station MAC. With several
// loggers in range, an AtomS3 listed here only pairs with that board type; an
// unlisted AtomS3 pairs with the first logger it hears.
struct LoggerAssignment {
  uint8_t atomMac[6];
  telemetry::LoggerBoard board;
};
constexpr LoggerAssignment kLoggerAssignments[] = {
    // GPS base, powered from the Cardputer v1 Grove port.
    {{0xF4, 0x12, 0xFA, 0xA0, 0x01, 0x18}, telemetry::LoggerBoard::Cardputer},
    // IMU only (SPK base), powered from the Core2 DIN base Port C.
    {{0x48, 0x27, 0xE2, 0xE4, 0x04, 0x60}, telemetry::LoggerBoard::Core2},
};

#ifndef ENABLE_RAW_IMU_LOGGING
#define ENABLE_RAW_IMU_LOGGING 0
#endif

}  // namespace atom_config
