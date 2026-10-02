#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

#include "AtomConfig.h"
#include "DinoSprite.h"
#include "Version.h"
#include "TelemetryProtocol.h"

namespace {

constexpr float kGravityMps2 = 9.80665f;

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
Preferences preferences;

struct MountCalibration {
  bool valid = false;
  float pitchRad = 0.0f;
  float rollRad = 0.0f;
  float gravityMps2 = kGravityMps2;
};

struct ImuState {
  bool valid = false;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
};

struct ImuStatistics {
  bool valid = false;
  float accelRms = 0.0f;
  float verticalRms = 0.0f;
  float verticalPeakPos = 0.0f;
  float verticalPeakNeg = 0.0f;
  float lateralPeak = 0.0f;
  float longitudinalPeak = 0.0f;
  float vibrationRms = 0.0f;
  uint16_t samples = 0;
};

MountCalibration calibration;
ImuState latestImu;
ImuStatistics publishedStatistics;

double sumAccelSquares = 0.0;
double sumVerticalSquares = 0.0;
float verticalPeakPos = 0.0f;
float verticalPeakNeg = 0.0f;
float lateralPeak = 0.0f;
float longitudinalPeak = 0.0f;
uint32_t statisticsSamples = 0;
uint32_t statisticsStartedMs = 0;

bool calibrating = false;
bool calibrationButtonLatched = false;
uint32_t calibrationSamples = 0;
double calibrationAx = 0.0;
double calibrationAy = 0.0;
double calibrationAz = 0.0;

size_t gpsBaudIndex = 0;
uint32_t gpsBaudStartedMs = 0;
uint32_t lastGpsSentenceMs = 0;
uint32_t lastImuSampleUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastChannelHopMs = 0;
uint32_t lastPeerSeenMs = 0;
uint32_t lastSendSuccessMs = 0;
uint32_t telemetrySequence = 0;
uint32_t rawSequence = 0;
std::atomic<uint32_t> transmitFailures{0};
uint8_t scanChannel = 1;
uint8_t peerChannel = 0;
uint8_t peerMac[6]{};
bool havePeer = false;
bool espNowReady = false;
portMUX_TYPE beaconMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t beaconMac[6]{};
uint8_t beaconChannel = 0;
bool beaconPending = false;
telemetry::DiscoveryPacket beaconPacket{};
// Latest beacon from the paired Cardputer: its clock, zone and battery.
telemetry::DiscoveryPacket hostBeacon{};
bool haveHostBeacon = false;
uint32_t hostBeaconMs = 0;

enum class Page : uint8_t {
  Combined, Imu, Gps, Time, Power, Version, About, Count
};
Page page = Page::Combined;
M5Canvas canvas(&M5.Display);
bool canvasReady = false;

#if ENABLE_RAW_IMU_LOGGING
telemetry::RawImuBatchPacket rawBatch{};
#endif

int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
  const unsigned dayOfYear =
      (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 -
                            yearOfEra / 100 + dayOfYear;
  return era * 146097LL + static_cast<int64_t>(dayOfEra) - 719468LL;
}

uint64_t gpsUtcEpochMs() {
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;
  const int year = gps.date.year();
  const unsigned month = gps.date.month();
  const unsigned day = gps.date.day();
  if (year < 2024 || year > 2099 || month < 1 || month > 12 || day < 1 ||
      day > 31) {
    return 0;
  }
  const int64_t seconds =
      daysFromCivil(year, month, day) * 86400LL + gps.time.hour() * 3600LL +
      gps.time.minute() * 60LL + gps.time.second();
  return static_cast<uint64_t>(seconds) * 1000ULL +
         static_cast<uint64_t>(gps.time.centisecond()) * 10ULL;
}

void startGps(uint32_t baud) {
  gpsSerial.end();
  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(baud, SERIAL_8N1, atom_config::kGpsRxPin,
                  atom_config::kGpsTxPin);
  gpsBaudStartedMs = millis();
  lastGpsSentenceMs = 0;
  Serial.printf("GPS baud: %lu\n", static_cast<unsigned long>(baud));
}

void pollGps(uint32_t nowMs) {
  const uint32_t previousChecksums = gps.passedChecksum();
  while (gpsSerial.available() > 0) {
    gps.encode(static_cast<char>(gpsSerial.read()));
  }
  if (gps.passedChecksum() != previousChecksums) lastGpsSentenceMs = nowMs;
  if (lastGpsSentenceMs == 0 &&
      nowMs - gpsBaudStartedMs >= atom_config::kGpsBaudScanIntervalMs) {
    gpsBaudIndex = (gpsBaudIndex + 1U) % atom_config::kGpsBaudCandidateCount;
    startGps(atom_config::kGpsBaudCandidates[gpsBaudIndex]);
  }
}

void rotateToInstalledFrame(float x, float y, float z, float& outX,
                            float& outY, float& outZ) {
  const float cr = std::cos(calibration.rollRad);
  const float sr = std::sin(calibration.rollRad);
  const float cp = std::cos(calibration.pitchRad);
  const float sp = std::sin(calibration.pitchRad);
  const float y1 = cr * y - sr * z;
  const float z1 = sr * y + cr * z;
  outX = cp * x + sp * z1;
  outY = y1;
  outZ = -sp * x + cp * z1;
}

void finishCalibration() {
  if (calibrationSamples < 50) {
    calibrating = false;
    return;
  }
  const float ax = calibrationAx / calibrationSamples;
  const float ay = calibrationAy / calibrationSamples;
  const float az = calibrationAz / calibrationSamples;
  calibration.pitchRad = std::atan2(-ax, std::sqrt(ay * ay + az * az));
  calibration.rollRad = std::atan2(ay, az);
  calibration.gravityMps2 = std::sqrt(ax * ax + ay * ay + az * az);
  calibration.valid = true;
  preferences.putBool("valid", true);
  preferences.putFloat("pitch", calibration.pitchRad);
  preferences.putFloat("roll", calibration.rollRad);
  preferences.putFloat("gravity", calibration.gravityMps2);
  calibrating = false;
  Serial.printf("Mount zero saved: pitch %.2f roll %.2f gravity %.4f\n",
                calibration.pitchRad * 180.0f / PI,
                calibration.rollRad * 180.0f / PI,
                calibration.gravityMps2);
}

void beginCalibration() {
  calibrating = true;
  calibrationSamples = 0;
  calibrationAx = calibrationAy = calibrationAz = 0.0;
}

int16_t clampInt16(float value) {
  return static_cast<int16_t>(
      std::max(-32768.0f, std::min(32767.0f, std::round(value))));
}

void queueRawSample(uint32_t nowUs) {
#if ENABLE_RAW_IMU_LOGGING
  if (rawBatch.sample_count == 0) {
    telemetry::preparePacket(rawBatch, telemetry::PacketType::RawImuBatch);
    rawBatch.sequence = rawSequence++;
    rawBatch.batch_start_time_ms = nowUs / 1000U;
  }
  auto& sample = rawBatch.samples[rawBatch.sample_count++];
  sample.accel_x_mg = clampInt16(latestImu.ax / kGravityMps2 * 1000.0f);
  sample.accel_y_mg = clampInt16(latestImu.ay / kGravityMps2 * 1000.0f);
  sample.accel_z_mg = clampInt16(latestImu.az / kGravityMps2 * 1000.0f);
  sample.gyro_x_deci_dps = clampInt16(latestImu.gx * 10.0f);
  sample.gyro_y_deci_dps = clampInt16(latestImu.gy * 10.0f);
  sample.gyro_z_deci_dps = clampInt16(latestImu.gz * 10.0f);
  const uint32_t startUs = rawBatch.batch_start_time_ms * 1000U;
  sample.time_offset_100us = static_cast<uint16_t>((nowUs - startUs) / 100U);
  if (rawBatch.sample_count == telemetry::kRawImuSamplesPerPacket) {
    telemetry::sealPacket(rawBatch);
    if (havePeer) {
      esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&rawBatch),
                   sizeof(rawBatch));
    }
    rawBatch.sample_count = 0;
  }
#else
  (void)nowUs;
#endif
}

void updateImu(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  if (nowUs - lastImuSampleUs < atom_config::kImuSampleIntervalUs) return;
  lastImuSampleUs = nowUs;
  if (!M5.Imu.update()) return;
  const auto data = M5.Imu.getImuData();
  const float rawAx = data.accel.x * kGravityMps2;
  const float rawAy = data.accel.y * kGravityMps2;
  const float rawAz = data.accel.z * kGravityMps2;

  if (calibrating) {
    calibrationAx += rawAx;
    calibrationAy += rawAy;
    calibrationAz += rawAz;
    if (++calibrationSamples >= 200) finishCalibration();
  }

  rotateToInstalledFrame(rawAx, rawAy, rawAz, latestImu.ax, latestImu.ay,
                         latestImu.az);
  rotateToInstalledFrame(data.gyro.x, data.gyro.y, data.gyro.z, latestImu.gx,
                         latestImu.gy, latestImu.gz);
  const float rawPitch =
      std::atan2(-rawAx, std::sqrt(rawAy * rawAy + rawAz * rawAz));
  const float rawRoll = std::atan2(rawAy, rawAz);
  latestImu.pitchDeg = (rawPitch - calibration.pitchRad) * 180.0f / PI;
  latestImu.rollDeg = (rawRoll - calibration.rollRad) * 180.0f / PI;
  latestImu.valid = true;

  const float vertical = latestImu.az - calibration.gravityMps2;
  const float dynamicMagnitude =
      std::sqrt(latestImu.ax * latestImu.ax + latestImu.ay * latestImu.ay +
                vertical * vertical);
  sumAccelSquares += static_cast<double>(dynamicMagnitude) * dynamicMagnitude;
  sumVerticalSquares += static_cast<double>(vertical) * vertical;
  verticalPeakPos = std::max(verticalPeakPos, vertical);
  verticalPeakNeg = std::min(verticalPeakNeg, vertical);
  lateralPeak = std::max(lateralPeak, std::fabs(latestImu.ay));
  longitudinalPeak = std::max(longitudinalPeak, std::fabs(latestImu.ax));
  ++statisticsSamples;
  queueRawSample(nowUs);

  if (statisticsStartedMs == 0) statisticsStartedMs = nowMs;
  if (nowMs - statisticsStartedMs >= atom_config::kStatisticsIntervalMs &&
      statisticsSamples > 0) {
    publishedStatistics.valid = true;
    publishedStatistics.accelRms =
        std::sqrt(sumAccelSquares / statisticsSamples);
    publishedStatistics.verticalRms =
        std::sqrt(sumVerticalSquares / statisticsSamples);
    publishedStatistics.verticalPeakPos = verticalPeakPos;
    publishedStatistics.verticalPeakNeg = verticalPeakNeg;
    publishedStatistics.lateralPeak = lateralPeak;
    publishedStatistics.longitudinalPeak = longitudinalPeak;
    publishedStatistics.vibrationRms = publishedStatistics.verticalRms;
    publishedStatistics.samples = static_cast<uint16_t>(
        std::min<uint32_t>(statisticsSamples, UINT16_MAX));
    sumAccelSquares = sumVerticalSquares = 0.0;
    verticalPeakPos = verticalPeakNeg = 0.0f;
    lateralPeak = longitudinalPeak = 0.0f;
    statisticsSamples = 0;
    statisticsStartedMs = nowMs;
  }
}

telemetry::LoggerBoard assignedBoard = telemetry::LoggerBoard::Unknown;

void loadLoggerAssignment() {
  uint8_t ownMac[6]{};
  esp_read_mac(ownMac, ESP_MAC_WIFI_STA);
  for (const auto& assignment : atom_config::kLoggerAssignments) {
    if (std::memcmp(ownMac, assignment.atomMac, 6) == 0) {
      assignedBoard = assignment.board;
    }
  }
}

const char* assignedBoardName() {
  switch (assignedBoard) {
    case telemetry::LoggerBoard::Cardputer: return "CARDPUTER";
    case telemetry::LoggerBoard::CardputerAdv: return "CARDPUTER ADV";
    case telemetry::LoggerBoard::Core2: return "CORE2";
    default: return "ANY LOGGER";
  }
}

bool configuredPeerAllowed(const uint8_t* mac) {
  return telemetry::macIsUnset(atom_config::kCardputerEspNowMac) ||
         std::memcmp(mac, atom_config::kCardputerEspNowMac, 6) == 0;
}

// The receive callback runs in the Wi-Fi task, so it only records the beacon.
// Peer changes happen in loop() where they cannot race esp_now_send().
void processBeacon(uint32_t nowMs) {
  uint8_t mac[6]{};
  uint8_t channel = 0;
  bool pending = false;
  telemetry::DiscoveryPacket beacon{};
  portENTER_CRITICAL(&beaconMux);
  if (beaconPending) {
    std::memcpy(mac, beaconMac, 6);
    channel = beaconChannel;
    beacon = beaconPacket;
    beaconPending = false;
    pending = true;
  }
  portEXIT_CRITICAL(&beaconMux);
  if (!pending) return;
  if (havePeer) {
    if (std::memcmp(mac, peerMac, 6) != 0) return;
    peerChannel = channel;
    lastPeerSeenMs = nowMs;
    hostBeacon = beacon;
    haveHostBeacon = true;
    hostBeaconMs = nowMs;
    return;
  }
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_now_peer_info_t peer{};
  std::memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) return;
  std::memcpy(peerMac, mac, 6);
  peerChannel = channel;
  scanChannel = channel;
  havePeer = true;
  lastPeerSeenMs = nowMs;
  hostBeacon = beacon;
  haveHostBeacon = true;
  hostBeaconMs = nowMs;
}

void receiveCallback(const esp_now_recv_info_t* info, const uint8_t* data,
                     int length) {
  if (info == nullptr || info->src_addr == nullptr || data == nullptr ||
      length != static_cast<int>(sizeof(telemetry::DiscoveryPacket)) ||
      !configuredPeerAllowed(info->src_addr)) {
    return;
  }
  telemetry::DiscoveryPacket packet{};
  std::memcpy(&packet, data, sizeof(packet));
  if (!telemetry::validatePacket(packet, telemetry::PacketType::Discovery) ||
      !telemetry::boardMatches(assignedBoard,
                               telemetry::beaconBoard(packet.time_flags))) {
    return;
  }
  const uint8_t channel = info->rx_ctrl == nullptr ? scanChannel
                                                   : info->rx_ctrl->channel;
  portENTER_CRITICAL(&beaconMux);
  std::memcpy(beaconMac, info->src_addr, 6);
  beaconChannel = channel;
  beaconPacket = packet;
  beaconPending = true;
  portEXIT_CRITICAL(&beaconMux);
}

void sendCallback(const esp_now_send_info_t*, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSendSuccessMs = millis();
  } else {
    transmitFailures.fetch_add(1, std::memory_order_relaxed);
  }
}

void beginEspNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(scanChannel, WIFI_SECOND_CHAN_NONE);
  espNowReady = esp_now_init() == ESP_OK;
  if (espNowReady) {
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sendCallback);
  }
}

void updateLink(uint32_t nowMs) {
  if (!espNowReady) return;
  if (havePeer) {
    // lastSendSuccessMs comes from the send callback and can be later than
    // nowMs; elapsedMs() keeps that from looking like a 49-day silence.
    const uint32_t lastHeardMs = std::max(lastPeerSeenMs, lastSendSuccessMs);
    if (telemetry::elapsedMs(nowMs, lastHeardMs) <= atom_config::kLinkTimeoutMs) {
      return;
    }
    esp_now_del_peer(peerMac);
    havePeer = false;
    std::memset(peerMac, 0, sizeof(peerMac));
  }
  if (nowMs - lastChannelHopMs >= atom_config::kChannelHopIntervalMs) {
    lastChannelHopMs = nowMs;
    scanChannel = scanChannel >= 13 ? 1 : scanChannel + 1;
    esp_wifi_set_channel(scanChannel, WIFI_SECOND_CHAN_NONE);
  }
}

void sendTelemetry(uint32_t nowMs) {
  if (!havePeer || nowMs - lastTelemetryMs < atom_config::kTelemetryIntervalMs) {
    return;
  }
  lastTelemetryMs = nowMs;
  telemetry::TelemetryPacket packet{};
  telemetry::preparePacket(packet, telemetry::PacketType::Telemetry);
  packet.sequence = telemetrySequence++;
  packet.uptime_ms = nowMs;
  packet.sample_time_ms = nowMs;

  const bool positionValid =
      gps.location.isValid() && gps.location.age() <= atom_config::kGpsFreshMs;
  const bool speedValid =
      gps.speed.isValid() && gps.speed.age() <= atom_config::kGpsFreshMs;
  const bool altitudeValid =
      gps.altitude.isValid() && gps.altitude.age() <= atom_config::kGpsFreshMs;
  const bool courseValid =
      gps.course.isValid() && gps.course.age() <= atom_config::kGpsFreshMs;
  const bool utcValid = gps.date.isValid() && gps.time.isValid() &&
                        gps.date.age() <= atom_config::kGpsFreshMs &&
                        gps.time.age() <= atom_config::kGpsFreshMs;
  const bool satellitesValid = gps.satellites.isValid();
  const bool hdopValid = gps.hdop.isValid();
  const bool fixValid = positionValid && speedValid && utcValid &&
                        satellitesValid && gps.satellites.value() >= 4 &&
                        hdopValid && gps.hdop.hdop() <= 5.0;
  if (fixValid) packet.status_flags |= telemetry::GpsFixValid;
  if (positionValid) packet.status_flags |= telemetry::GpsPositionValid;
  if (altitudeValid) packet.status_flags |= telemetry::GpsAltitudeValid;
  if (speedValid) packet.status_flags |= telemetry::GpsSpeedValid;
  if (courseValid) packet.status_flags |= telemetry::GpsCourseValid;
  if (satellitesValid) packet.status_flags |= telemetry::GpsSatellitesValid;
  if (hdopValid) packet.status_flags |= telemetry::GpsHdopValid;
  if (utcValid) packet.status_flags |= telemetry::GpsUtcValid;
  if (latestImu.valid) packet.status_flags |= telemetry::ImuValid;
  if (calibration.valid) packet.status_flags |= telemetry::ImuCalibrated;
  if (publishedStatistics.valid) {
    packet.status_flags |= telemetry::ImuStatisticsValid;
  }

  packet.gps_age_ms = gps.location.isValid()
                          ? static_cast<uint16_t>(
                                std::min<uint32_t>(gps.location.age(), UINT16_MAX))
                          : UINT16_MAX;
  packet.gps_utc_ms = utcValid ? gpsUtcEpochMs() : 0;
  packet.latitude_deg = gps.location.lat();
  packet.longitude_deg = gps.location.lng();
  packet.altitude_m = gps.altitude.meters();
  packet.speed_kmh = gps.speed.kmph();
  packet.course_deg = gps.course.deg();
  packet.hdop = gps.hdop.hdop();
  packet.satellites = static_cast<uint8_t>(
      std::min<uint32_t>(gps.satellites.value(), UINT8_MAX));
  packet.accel_x_mps2 = latestImu.ax;
  packet.accel_y_mps2 = latestImu.ay;
  packet.accel_z_mps2 = latestImu.az;
  packet.gyro_x_dps = latestImu.gx;
  packet.gyro_y_dps = latestImu.gy;
  packet.gyro_z_dps = latestImu.gz;
  packet.pitch_deg = latestImu.pitchDeg;
  packet.roll_deg = latestImu.rollDeg;
  packet.accel_rms_mps2 = publishedStatistics.accelRms;
  packet.vertical_accel_rms_mps2 = publishedStatistics.verticalRms;
  packet.vertical_accel_peak_pos_mps2 = publishedStatistics.verticalPeakPos;
  packet.vertical_accel_peak_neg_mps2 = publishedStatistics.verticalPeakNeg;
  packet.lateral_accel_peak_mps2 = publishedStatistics.lateralPeak;
  packet.longitudinal_accel_peak_mps2 = publishedStatistics.longitudinalPeak;
  packet.vibration_rms_mps2 = publishedStatistics.vibrationRms;
  packet.imu_sample_count = publishedStatistics.samples;
  packet.transmit_failures = static_cast<uint16_t>(
      std::min<uint32_t>(transmitFailures.load(std::memory_order_relaxed),
                         UINT16_MAX));
  telemetry::sealPacket(packet);
  if (esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&packet),
                   sizeof(packet)) != ESP_OK) {
    transmitFailures.fetch_add(1, std::memory_order_relaxed);
  }
}

constexpr uint16_t kAxisXColor = TFT_RED;
constexpr uint16_t kAxisYColor = TFT_GREEN;
constexpr uint16_t kAxisZColor = 0x54BF;  // Light blue; pure blue is dim.
constexpr uint32_t kHostBeaconFreshMs = 10'000;
constexpr uint32_t kHostClockValidMs = 10UL * 60UL * 1000UL;

// Pages use double-size text (12 x 16 px, 10 characters per row). Frames are
// drawn off-screen and pushed in one transfer so the panel never flickers.
class ScreenRows {
 public:
  ScreenRows(M5Canvas& display, int top, int pitch)
      : display_(display), y_(top), pitch_(pitch) {}

  template <typename... Values>
  void line(uint16_t color, const char* format, Values... values) {
    char text[24]{};
    snprintf(text, sizeof(text), format, values...);
    display_.setTextSize(2);
    display_.setTextColor(color, TFT_BLACK);
    display_.drawString(text, 2, y_);
    y_ += pitch_;
  }

 private:
  M5Canvas& display_;
  int y_;
  int pitch_;
};

bool gpsNmeaLive(uint32_t nowMs) {
  return lastGpsSentenceMs != 0 &&
         nowMs - lastGpsSentenceMs <= atom_config::kGpsFreshMs;
}

bool gpsPositionFresh() {
  return gps.location.isValid() &&
         gps.location.age() <= atom_config::kGpsFreshMs;
}

bool hostBeaconFresh(uint32_t nowMs) {
  return haveHostBeacon && nowMs - hostBeaconMs <= kHostBeaconFreshMs;
}

uint16_t batteryColor(int percent) {
  if (percent < 20) return TFT_RED;
  if (percent < 50) return TFT_ORANGE;
  return TFT_GREEN;
}

void drawGpsStatusRow(ScreenRows& rows, uint32_t nowMs) {
  const unsigned long satellites =
      gps.satellites.isValid() ? gps.satellites.value() : 0;
  if (!gpsNmeaLive(nowMs)) {
    rows.line(TFT_RED, "NO GPS");
  } else {
    rows.line(gpsPositionFresh() ? TFT_GREEN : TFT_ORANGE, "%s %lusat",
              gpsPositionFresh() ? "FIX" : "WAIT", satellites);
  }
}

void drawTitle(const char* title, uint16_t color) {
  canvas.setTextSize(1);
  canvas.setTextColor(color, TFT_BLACK);
  canvas.setTextDatum(top_left);
  canvas.drawString(title, 2, 1);
  char index[8]{};
  snprintf(index, sizeof(index), "%u/%u", static_cast<unsigned>(page) + 1,
           static_cast<unsigned>(Page::Count));
  canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
  canvas.setTextDatum(top_right);
  canvas.drawString(index, canvas.width() - 2, 1);
  canvas.setTextDatum(top_left);
  canvas.drawFastHLine(0, 10, canvas.width(), color);
}

void drawCombinedPage(uint32_t nowMs) {
  ScreenRows rows(canvas, 1, 18);
  drawGpsStatusRow(rows, nowMs);
  rows.line(havePeer ? TFT_GREEN : TFT_ORANGE, "%s ch%u",
            havePeer ? "LINK" : "SCAN",
            static_cast<unsigned>(havePeer ? peerChannel : scanChannel));
  rows.line(!latestImu.valid ? TFT_RED
                             : (calibration.valid ? TFT_GREEN : TFT_ORANGE),
            "IMU %s",
            !latestImu.valid ? "WAIT" : (calibration.valid ? "ZERO" : "RAW"));
  rows.line(TFT_WHITE, "P %+6.1f", latestImu.pitchDeg);
  rows.line(TFT_WHITE, "R %+6.1f", latestImu.rollDeg);
  rows.line(TFT_CYAN, "VIB %5.2f", publishedStatistics.vibrationRms);

  canvas.setTextSize(1);
  canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
  char diagnostics[32]{};
  if (gps.location.isValid()) {
    snprintf(diagnostics, sizeof(diagnostics), "SEQ %lu AGE %lums",
             static_cast<unsigned long>(telemetrySequence),
             static_cast<unsigned long>(
                 std::min<uint32_t>(gps.location.age(), 99999)));
  } else {
    snprintf(diagnostics, sizeof(diagnostics), "SEQ %lu AGE --",
             static_cast<unsigned long>(telemetrySequence));
  }
  canvas.drawString(diagnostics, 2, 110);
  canvas.drawString(WiFi.macAddress(), 2, 119);
}

void drawImuPage() {
  drawTitle(calibration.valid ? "IMU  m/s2  dps" : "IMU RAW m/s2 dps",
            calibration.valid ? TFT_GREEN : TFT_ORANGE);
  ScreenRows rows(canvas, 14, 19);
  if (!latestImu.valid) {
    rows.line(TFT_RED, "IMU WAIT");
    return;
  }
  rows.line(kAxisXColor, "AX %+6.2f", latestImu.ax);
  rows.line(kAxisYColor, "AY %+6.2f", latestImu.ay);
  rows.line(kAxisZColor, "AZ %+6.2f", latestImu.az);
  rows.line(kAxisXColor, "GX %+6.1f", latestImu.gx);
  rows.line(kAxisYColor, "GY %+6.1f", latestImu.gy);
  rows.line(kAxisZColor, "GZ %+6.1f", latestImu.gz);
}

void drawGpsPage(uint32_t nowMs) {
  char title[24]{};
  snprintf(title, sizeof(title), "GPS %lu baud",
           static_cast<unsigned long>(
               atom_config::kGpsBaudCandidates[gpsBaudIndex]));
  drawTitle(title, TFT_CYAN);
  ScreenRows rows(canvas, 14, 19);
  drawGpsStatusRow(rows, nowMs);
  if (gpsPositionFresh()) {
    rows.line(TFT_WHITE, "%.5f", gps.location.lat());
    rows.line(TFT_WHITE, "%.5f", gps.location.lng());
  } else {
    rows.line(TFT_DARKGREY, "LAT --");
    rows.line(TFT_DARKGREY, "LON --");
  }
  if (gps.speed.isValid() && gps.speed.age() <= atom_config::kGpsFreshMs) {
    rows.line(TFT_GREEN, "%5.1fkm/h", gps.speed.kmph());
  } else {
    rows.line(TFT_DARKGREY, "  --km/h");
  }
  if (gps.altitude.isValid() &&
      gps.altitude.age() <= atom_config::kGpsFreshMs) {
    rows.line(TFT_WHITE, "ALT %4.0fm", gps.altitude.meters());
  } else {
    rows.line(TFT_DARKGREY, "ALT --");
  }
  if (gps.hdop.isValid()) {
    rows.line(gps.hdop.hdop() <= 5.0 ? TFT_WHITE : TFT_ORANGE, "HDOP %.1f",
              gps.hdop.hdop());
  } else {
    rows.line(TFT_DARKGREY, "HDOP --");
  }
}

// UTC comes from the GPS when it has fresh time, otherwise from the paired
// Cardputer's beacon (its GPS, NTP or RTC clock). Local time uses the
// Cardputer's current UTC offset, which already includes daylight saving.
void drawTimePage(uint32_t nowMs) {
  uint64_t utcMs = 0;
  const char* source = "NO TIME";
  const bool gpsTime = gps.date.isValid() && gps.time.isValid() &&
                       gps.time.age() <= atom_config::kGpsFreshMs;
  if (gpsTime && gpsUtcEpochMs() != 0) {
    utcMs = gpsUtcEpochMs() + gps.time.age();
    source = "GPS";
  } else if (haveHostBeacon && (hostBeacon.time_flags & telemetry::UtcValid) &&
             nowMs - hostBeaconMs <= kHostClockValidMs) {
    utcMs = static_cast<uint64_t>(hostBeacon.utc_epoch_s) * 1000ULL +
            (nowMs - hostBeaconMs);
    source = "CARDPUTER";
  }
  const bool haveOffset =
      haveHostBeacon && (hostBeacon.time_flags & telemetry::UtcOffsetValid);
  const int offsetMin = haveOffset ? hostBeacon.utc_offset_min : 0;

  char title[24]{};
  snprintf(title, sizeof(title), "%s %s", haveOffset ? "LOCAL" : "UTC",
           source);
  drawTitle(title, utcMs != 0 ? TFT_GREEN : TFT_ORANGE);
  canvas.setTextDatum(middle_center);
  if (utcMs == 0) {
    canvas.setTextSize(4);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString("--:--", 64, 44);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
    canvas.drawString("Waiting for GPS time", 64, 80);
    canvas.drawString("or Cardputer link", 64, 92);
    canvas.setTextDatum(top_left);
    return;
  }
  const time_t localSeconds = static_cast<time_t>(utcMs / 1000ULL) +
                              static_cast<time_t>(offsetMin) * 60;
  struct tm local {};
  gmtime_r(&localSeconds, &local);
  char text[24]{};
  canvas.setTextSize(4);
  canvas.setTextColor(TFT_GREEN, TFT_BLACK);
  snprintf(text, sizeof(text), "%02d:%02d", local.tm_hour, local.tm_min);
  canvas.drawString(text, 64, 36);
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(text, sizeof(text), ":%02d", local.tm_sec);
  canvas.drawString(text, 64, 66);
  snprintf(text, sizeof(text), "%04d-%02d-%02d", local.tm_year + 1900,
           local.tm_mon + 1, local.tm_mday);
  canvas.drawString(text, 64, 89);
  static constexpr const char* kDays[] = {"Sun", "Mon", "Tue", "Wed",
                                          "Thu", "Fri", "Sat"};
  const int absoluteOffset = offsetMin < 0 ? -offsetMin : offsetMin;
  if (haveOffset) {
    snprintf(text, sizeof(text), "%s %c%02d:%02d", kDays[local.tm_wday % 7],
             offsetMin < 0 ? '-' : '+', absoluteOffset / 60,
             absoluteOffset % 60);
  } else {
    snprintf(text, sizeof(text), "%s UTC", kDays[local.tm_wday % 7]);
  }
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.drawString(text, 64, 112);
  canvas.setTextDatum(top_left);
}

// The AtomS3 and Atomic GPS Base have no battery; the Cardputer's battery is
// relayed in its discovery beacon.
void drawPowerPage(uint32_t nowMs) {
  drawTitle("POWER", TFT_CYAN);
  ScreenRows rows(canvas, 14, 19);
  rows.line(TFT_WHITE, "ATOMS3");
  const int32_t ownLevel = M5.Power.getBatteryLevel();
  if (ownLevel >= 0 && ownLevel <= 100) {
    rows.line(batteryColor(ownLevel), "BAT %ld%%",
              static_cast<long>(ownLevel));
  } else {
    rows.line(TFT_GREEN, "USB POWER");
  }
  rows.line(TFT_WHITE, "CARDPUTER");
  if (!hostBeaconFresh(nowMs)) {
    rows.line(TFT_ORANGE, "NO LINK");
  } else if (hostBeacon.battery_percent < 0) {
    rows.line(TFT_DARKGREY, "NO GAUGE");
  } else {
    rows.line(batteryColor(hostBeacon.battery_percent), "BAT %d%%",
              hostBeacon.battery_percent);
    rows.line(TFT_WHITE, "%.2f V", hostBeacon.battery_mv / 1000.0f);
  }
}

void drawCalibrationPage() {
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
  canvas.setTextDatum(middle_center);
  canvas.drawString("ZEROING", 64, 28);
  canvas.drawString("KEEP", 64, 56);
  canvas.drawString("STILL", 64, 76);
  char progress[16]{};
  snprintf(progress, sizeof(progress), "%lu/200",
           static_cast<unsigned long>(calibrationSamples));
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(progress, 64, 106);
  canvas.setTextDatum(top_left);
}

dino::Game dinoGame;

// This AtomS3's own GPS speed for the game; unknown counts as parked.
float dinoGameSpeedKmh() {
  return gps.speed.isValid() && gps.speed.age() <= atom_config::kGpsFreshMs
             ? static_cast<float>(gps.speed.kmph())
             : -1.0f;
}

void drawVersionPage(uint32_t nowMs) {
  drawTitle("FOSSIL RECORD", TFT_CYAN);
  dino::draw(canvas, canvas.width() - dino::kWidth * 2 - 2, 12, 2,
             (nowMs / 4000) % dino::kFrameCount);
  canvas.setTextSize(1);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  canvas.drawString("Species", 2, 16);
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.drawString(version::kNumber, 2, 26);
  canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  canvas.drawString("DNA", 2, 50);
  canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
  canvas.drawString(version::kGit, 30, 50);
  canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  canvas.drawString("Hatched", 2, 64);
  canvas.setTextColor(TFT_GREEN, TFT_BLACK);
  canvas.drawString(version::kDate, 2, 76);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
  canvas.drawString(version::jokeAt(nowMs), canvas.width() / 2, 104);
  canvas.setTextDatum(top_left);
}

void drawAboutPage(uint32_t nowMs) {
  drawTitle("JP226PRINTS", TFT_CYAN);
  // Scale 2 leaves enough runway to see the rocks coming on 128 pixels.
  constexpr int kScale = 2;
  constexpr int kGroundY = 16 + dino::kHeight * kScale + 1;
  const int width = canvas.width();
  dinoGame.update(nowMs, dinoGameSpeedKmh(), width / kScale);
  if (dinoGame.takeNewBest()) preferences.putUShort(dino::kBestKey, dinoGame.best());
  dinoGame.draw(canvas, 0, kGroundY, width, kScale, nowMs);
  canvas.setTextSize(1);
  canvas.setTextDatum(middle_center);
  switch (dinoGame.state()) {
    case dino::Game::State::Ready:
      canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
      canvas.drawString("TAP: PLAY HOLD: PAGE", width / 2, 62);
      break;
    case dino::Game::State::Playing:
    case dino::Game::State::Over: {
      char text[32];
      dinoGame.statusText(text, sizeof(text));
      canvas.setTextColor(dinoGame.state() == dino::Game::State::Playing
                              ? TFT_WHITE
                          : dinoGame.beatBest() ? TFT_GREEN
                                                : TFT_RED,
                          TFT_BLACK);
      canvas.drawString(text, width / 2, 62);
      break;
    }
    default:
      break;
  }
  canvas.setTextDatum(middle_center);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(dino::kCredit1, width / 2, 84);
  // Size-2 built-in text is 132 px wide here, wider than the screen.
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.drawString(dino::kCredit2, width / 2, 104);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
}

void drawStatus(uint32_t nowMs) {
  // The dinosaur page animates; everything else refreshes twice a second.
  const uint32_t interval =
      page == Page::About ? 60 : atom_config::kDisplayIntervalMs;
  if (nowMs - lastDisplayMs < interval) return;
  lastDisplayMs = nowMs;
  if (!canvasReady) return;
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextDatum(top_left);
  if (calibrating) {
    drawCalibrationPage();
  } else {
    switch (page) {
      case Page::Imu: drawImuPage(); break;
      case Page::Gps: drawGpsPage(nowMs); break;
      case Page::Time: drawTimePage(nowMs); break;
      case Page::Power: drawPowerPage(nowMs); break;
      case Page::Version: drawVersionPage(nowMs); break;
      case Page::About: drawAboutPage(nowMs); break;
      default: drawCombinedPage(nowMs); break;
    }
  }
  canvas.pushSprite(0, 0);
}

void nextPage() {
  page = static_cast<Page>((static_cast<uint8_t>(page) + 1U) %
                           static_cast<uint8_t>(Page::Count));
  preferences.putUChar("page", static_cast<uint8_t>(page));
  lastDisplayMs = 0;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  auto m5Config = M5.config();
  m5Config.internal_imu = true;
  M5.begin(m5Config);
  M5.Display.setRotation(0);
  M5.Display.setTextWrap(false);
  M5.Display.fillScreen(TFT_BLACK);
  canvas.setColorDepth(16);
  canvasReady = canvas.createSprite(M5.Display.width(),
                                    M5.Display.height()) != nullptr;
  canvas.setTextWrap(false);
  preferences.begin("mountzero", false);
  calibration.valid = preferences.getBool("valid", false);
  calibration.pitchRad = preferences.getFloat("pitch", 0.0f);
  calibration.rollRad = preferences.getFloat("roll", 0.0f);
  calibration.gravityMps2 = preferences.getFloat("gravity", kGravityMps2);
  dinoGame.setBest(preferences.getUShort(dino::kBestKey, 0));
  const uint8_t savedPage = preferences.getUChar("page", 0);
  if (savedPage < static_cast<uint8_t>(Page::Count)) {
    page = static_cast<Page>(savedPage);
  }
  if (M5.Imu.getType() == m5::imu_none) {
    Serial.println("ERROR: AtomS3 IMU not detected");
  }
  gpsBaudIndex = 0;
  startGps(atom_config::kGpsBaudCandidates[gpsBaudIndex]);
  beginEspNow();
  Serial.printf("AtomS3 station MAC: %s\n", WiFi.macAddress().c_str());
}

void loop() {
  M5.update();
  const uint32_t nowMs = millis();
  pollGps(nowMs);
  updateImu(nowMs);
  processBeacon(nowMs);
  updateLink(nowMs);
  sendTelemetry(nowMs);

  static bool pressOnGame = false;
  if (M5.BtnA.wasPressed()) {
    pressOnGame = page == Page::About &&
                  dinoGame.state() != dino::Game::State::Auto;
    if (pressOnGame) dinoGame.press(nowMs);
  }
  if (M5.BtnA.pressedFor(1500) && !calibrationButtonLatched) {
    calibrationButtonLatched = true;
    if (page == Page::About) nextPage();
    else beginCalibration();
  }
  // A short press of the screen cycles pages; the hold above zeroes the mount.
  // On the playable dinosaur page the press was a jump instead.
  if (M5.BtnA.wasReleased()) {
    if (!calibrationButtonLatched && !pressOnGame) nextPage();
    calibrationButtonLatched = false;
  }
  drawStatus(nowMs);
  delay(1);
}
