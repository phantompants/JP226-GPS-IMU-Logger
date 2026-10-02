#pragma once

#include <Arduino.h>

enum class TelemetrySource : uint8_t {
  LocalGps = 0,
  AtomS3Remote = 1,
  CardputerAdv = 2,  // Reserved for a future fully-normalized ADV source.
  // Position from the local GPS, motion from an AtomS3 over ESP-NOW. For
  // boards without their own IMU.
  LocalGpsAtomImu = 3,
};

inline bool usesRemoteImu(TelemetrySource source) {
  return source == TelemetrySource::AtomS3Remote ||
         source == TelemetrySource::LocalGpsAtomImu;
}

struct GpsSnapshot {
  bool fixValid = false;
  bool positionFresh = false;
  bool speedFresh = false;
  bool altitudeFresh = false;
  bool courseFresh = false;
  bool satellitesValid = false;
  bool hdopValid = false;
  bool vdopValid = false;
  bool utcValid = false;
  uint32_t fixAgeMs = UINT32_MAX;
  uint64_t utcEpochMs = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitudeM = 0.0;
  double speedKmh = 0.0;
  double courseDeg = 0.0;
  uint32_t satellites = 0;
  double hdop = 0.0;
  double vdop = 0.0;
};

struct ImuSample {
  bool available = false;
  bool valid = false;
  bool calibrated = false;
  float axMps2 = 0.0f;
  float ayMps2 = 0.0f;
  float azMps2 = 0.0f;
  float gxDps = 0.0f;
  float gyDps = 0.0f;
  float gzDps = 0.0f;
  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
  float gTotalMps2 = 0.0f;
  float accelRmsMps2 = 0.0f;
  float verticalAccelRmsMps2 = 0.0f;
  float verticalAccelPeakPosMps2 = 0.0f;
  float verticalAccelPeakNegMps2 = 0.0f;
  float lateralAccelPeakMps2 = 0.0f;
  float longitudinalAccelPeakMps2 = 0.0f;
  float vibrationRmsMps2 = 0.0f;
  float legacyRoughnessMps2 = 0.0f;
  uint16_t sampleCount = 0;
};

struct TelemetryMetadata {
  TelemetrySource source = TelemetrySource::LocalGps;
  bool remoteConnected = false;
  uint32_t remoteSequence = 0;
  uint32_t packetAgeMs = UINT32_MAX;
  uint32_t packetsLost = 0;
  uint32_t duplicates = 0;
  uint32_t crcErrors = 0;
  uint32_t versionErrors = 0;
  uint16_t remoteTransmitFailures = 0;
};

struct NormalizedTelemetry {
  GpsSnapshot gps;
  ImuSample imu;
  TelemetryMetadata metadata;
};
