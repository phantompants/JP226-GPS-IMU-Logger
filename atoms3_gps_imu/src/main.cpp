#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

#include "AtomConfig.h"
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
  portENTER_CRITICAL(&beaconMux);
  if (beaconPending) {
    std::memcpy(mac, beaconMac, 6);
    channel = beaconChannel;
    beaconPending = false;
    pending = true;
  }
  portEXIT_CRITICAL(&beaconMux);
  if (!pending) return;
  if (havePeer) {
    if (std::memcmp(mac, peerMac, 6) != 0) return;
    peerChannel = channel;
    lastPeerSeenMs = nowMs;
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
  if (!telemetry::validatePacket(packet, telemetry::PacketType::Discovery)) {
    return;
  }
  const uint8_t channel = info->rx_ctrl == nullptr ? scanChannel
                                                   : info->rx_ctrl->channel;
  portENTER_CRITICAL(&beaconMux);
  std::memcpy(beaconMac, info->src_addr, 6);
  beaconChannel = channel;
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
    if (nowMs - std::max(lastPeerSeenMs, lastSendSuccessMs) <=
        atom_config::kLinkTimeoutMs) {
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

void drawStatus(uint32_t nowMs) {
  if (nowMs - lastDisplayMs < atom_config::kDisplayIntervalMs) return;
  lastDisplayMs = nowMs;
  auto& display = M5.Display;
  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setTextSize(2);
  display.setCursor(3, 3);
  display.println("GPS+IMU");
  display.setTextSize(1);
  display.setTextColor(gps.location.isValid() &&
                               gps.location.age() <= atom_config::kGpsFreshMs
                           ? TFT_GREEN
                           : TFT_ORANGE,
                       TFT_BLACK);
  display.printf("GPS %s  SAT %lu\n",
                 gps.location.isValid() ? "DATA" : "WAIT",
                 static_cast<unsigned long>(gps.satellites.value()));
  display.setTextColor(latestImu.valid ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  display.printf("IMU %s %s\n", latestImu.valid ? "100Hz" : "WAIT",
                 calibration.valid ? "ZERO" : "RAW");
  display.setTextColor(havePeer ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  display.printf("LINK %s CH %u\n", havePeer ? "OK" : "SEARCH", havePeer
                                                       ? peerChannel
                                                       : scanChannel);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.printf("SEQ %lu AGE %lums\n", static_cast<unsigned long>(
                                         telemetrySequence),
                 static_cast<unsigned long>(gps.location.age()));
  display.printf("P%+.1f R%+.1f V%.2f\n", latestImu.pitchDeg,
                 latestImu.rollDeg, publishedStatistics.vibrationRms);
  display.printf("MAC %s\n", WiFi.macAddress().c_str());
  display.setTextColor(calibrating ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK);
  display.println(calibrating ? "CALIBRATING - KEEP STILL"
                              : "Hold button: mount zero");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  auto m5Config = M5.config();
  m5Config.internal_imu = true;
  M5.begin(m5Config);
  M5.Display.setRotation(0);
  M5.Display.setTextWrap(false);
  preferences.begin("mountzero", false);
  calibration.valid = preferences.getBool("valid", false);
  calibration.pitchRad = preferences.getFloat("pitch", 0.0f);
  calibration.rollRad = preferences.getFloat("roll", 0.0f);
  calibration.gravityMps2 = preferences.getFloat("gravity", kGravityMps2);
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

  if (M5.BtnA.pressedFor(1500) && !calibrationButtonLatched) {
    calibrationButtonLatched = true;
    beginCalibration();
  }
  if (!M5.BtnA.isPressed()) calibrationButtonLatched = false;
  drawStatus(nowMs);
  delay(1);
}
