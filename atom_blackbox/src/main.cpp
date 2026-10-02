// Atom Lite "black box": a backup logger for the Cardputer. It pairs with the
// logger like the M5Dial, receives a position report once a second and writes
// it to daily CSV files on the Atomic SPK base's microSD card, so a trip
// survives even if the Cardputer's own card fails.
//
// Logger choice: hold the button and press the side reset button to toggle
// between the Cardputer v1 (magenta blinks) and the Cardputer ADV (cyan).
// Button: tap to save a waypoint on the logger.
// Light: blue blink = searching, green = recording, amber = linked but not
// recording, red blink = SD card problem, white = waiting for a waypoint ack.

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <cstring>
#include <ctime>

#include "TelemetryProtocol.h"
#include "TempProbe.h"
#include "Version.h"

namespace {

// Atomic SPK base microSD. Its chip select is tied low, hence -1.
constexpr int kSdSckPin = 23;
constexpr int kSdMisoPin = 33;
constexpr int kSdMosiPin = 19;
constexpr uint32_t kSdFrequency = 25'000'000;
constexpr char kDirectory[] = "/blackbox";
constexpr char kWaypointCategory[] = "BLACKBOX";

constexpr uint8_t kChannels = 13;
constexpr uint32_t kScanDwellMs = 450;
constexpr uint32_t kHeartbeatMs = 1000;
constexpr uint32_t kLinkStaleMs = 5000;
constexpr uint32_t kStatusIntervalMs = 2000;
constexpr uint32_t kSdRetryMs = 10'000;
constexpr uint32_t kFlushIntervalMs = 10'000;
constexpr size_t kFlushRows = 10;
constexpr uint32_t kRecordingFreshMs = 5000;
constexpr uint32_t kFreeSpaceIntervalMs = 60'000;
constexpr uint32_t kAckTimeoutMs = 3500;

constexpr telemetry::LoggerBoard kTargets[] = {
    telemetry::LoggerBoard::CardputerAdv, telemetry::LoggerBoard::Cardputer};
// Read by the receive callback; a single byte, so no lock is needed.
volatile uint8_t targetBoard =
    static_cast<uint8_t>(telemetry::LoggerBoard::CardputerAdv);

portMUX_TYPE receiveMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t candidateMac[6]{};
uint8_t candidateChannel = 0;
bool candidateReady = false;
bool paired = false;
uint8_t peerMac[6]{};
uint8_t channel = 1;
uint32_t lastHeardMs = 0;  // Any packet from the logger, stamped in the callback.
telemetry::DialAckPacket receivedAck{};
bool ackReady = false;

// Position reports waiting to be written; the callback only copies them.
constexpr size_t kQueueSize = 8;
telemetry::PositionReportPacket queue[kQueueSize]{};
size_t queueHead = 0;
size_t queueCount = 0;

Preferences preferences;
uint32_t nextSequence = 1;
uint32_t statusSequence = 0;
uint32_t lastScanMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastStatusMs = 0;
telemetry::DialCommandPacket pendingPacket{};
uint32_t pendingSequence = 0;
uint32_t pendingSinceMs = 0;
uint32_t lastPendingSendMs = 0;
uint8_t pendingAttempts = 0;
uint32_t flashColor = 0;
uint32_t flashUntilMs = 0;

bool sdReady = false;
uint32_t lastSdAttemptMs = 0;
String currentPath;
String rowBuffer;
size_t bufferedRows = 0;
uint32_t lastFlushMs = 0;
uint32_t rowsWritten = 0;
uint32_t lastWrittenMs = 0;
uint32_t sdFreeMb = 0;
uint32_t lastFreeSpaceMs = 0;

void onReceive(const esp_now_recv_info_t* info, const uint8_t* bytes,
               int length) {
  if (info == nullptr || info->src_addr == nullptr || bytes == nullptr ||
      length < static_cast<int>(sizeof(telemetry::PacketHeader))) return;
  telemetry::PacketHeader header{};
  std::memcpy(&header, bytes, sizeof(header));
  if (header.magic != telemetry::kMagic ||
      header.version != telemetry::kProtocolVersion ||
      header.size != length) return;
  portENTER_CRITICAL(&receiveMux);
  const bool currentlyPaired = paired;
  const bool samePeer =
      currentlyPaired && std::memcmp(peerMac, info->src_addr, 6) == 0;
  portEXIT_CRITICAL(&receiveMux);
  const auto type = static_cast<telemetry::PacketType>(header.type);
  if (type == telemetry::PacketType::Discovery &&
      length == sizeof(telemetry::DiscoveryPacket)) {
    telemetry::DiscoveryPacket discovery{};
    std::memcpy(&discovery, bytes, sizeof(discovery));
    if (!telemetry::validatePacket(discovery, type) || currentlyPaired ||
        !telemetry::boardMatches(
            static_cast<telemetry::LoggerBoard>(targetBoard),
            telemetry::beaconBoard(discovery.time_flags))) {
      return;
    }
    portENTER_CRITICAL(&receiveMux);
    std::memcpy(candidateMac, info->src_addr, 6);
    candidateChannel =
        info->rx_ctrl != nullptr ? info->rx_ctrl->channel : channel;
    candidateReady = true;
    portEXIT_CRITICAL(&receiveMux);
    return;
  }
  if (!samePeer) return;
  if (type == telemetry::PacketType::PositionReport &&
      length == sizeof(telemetry::PositionReportPacket)) {
    telemetry::PositionReportPacket report{};
    std::memcpy(&report, bytes, sizeof(report));
    if (!telemetry::validatePacket(report, type)) return;
    portENTER_CRITICAL(&receiveMux);
    if (queueCount < kQueueSize) {
      queue[(queueHead + queueCount) % kQueueSize] = report;
      ++queueCount;
    }
    lastHeardMs = millis();
    portEXIT_CRITICAL(&receiveMux);
  } else if (type == telemetry::PacketType::DialStatus &&
             length == sizeof(telemetry::DialStatusPacket)) {
    // The logger's status is only used as proof the link is alive.
    portENTER_CRITICAL(&receiveMux);
    lastHeardMs = millis();
    portEXIT_CRITICAL(&receiveMux);
  } else if (type == telemetry::PacketType::DialAck &&
             length == sizeof(telemetry::DialAckPacket)) {
    telemetry::DialAckPacket ack{};
    std::memcpy(&ack, bytes, sizeof(ack));
    if (!telemetry::validatePacket(ack, type)) return;
    portENTER_CRITICAL(&receiveMux);
    receivedAck = ack;
    ackReady = true;
    portEXIT_CRITICAL(&receiveMux);
  }
}

void flash(uint32_t color, uint32_t durationMs) {
  flashColor = color;
  flashUntilMs = millis() + durationMs;
}

bool sendCommand(telemetry::DialAction action, const char* text = "") {
  if (!paired) return false;
  telemetry::DialCommandPacket packet{};
  telemetry::preparePacket(packet, telemetry::PacketType::DialCommand);
  packet.sequence = nextSequence++;
  packet.action = static_cast<uint8_t>(action);
  std::snprintf(packet.text, sizeof(packet.text), "%s", text);
  telemetry::sealPacket(packet);
  if (esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&packet),
                   sizeof(packet)) != ESP_OK) {
    return false;
  }
  if (action != telemetry::DialAction::Heartbeat) {
    pendingPacket = packet;
    pendingSequence = packet.sequence;
    pendingSinceMs = lastPendingSendMs = millis();
    pendingAttempts = 1;
  }
  return true;
}

void connectCandidate(uint32_t nowMs) {
  uint8_t mac[6]{};
  uint8_t foundChannel = 0;
  portENTER_CRITICAL(&receiveMux);
  if (candidateReady) {
    std::memcpy(mac, candidateMac, 6);
    foundChannel = candidateChannel;
    candidateReady = false;
  }
  portEXIT_CRITICAL(&receiveMux);
  if (foundChannel == 0 || paired) return;
  if (esp_wifi_set_channel(foundChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    return;
  esp_now_peer_info_t peer{};
  std::memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) return;
  portENTER_CRITICAL(&receiveMux);
  std::memcpy(peerMac, mac, 6);
  channel = foundChannel;
  paired = true;
  lastHeardMs = nowMs;
  portEXIT_CRITICAL(&receiveMux);
  lastHeartbeatMs = 0;
  Serial.printf("Paired with %02X:%02X:%02X:%02X:%02X:%02X on channel %u\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel);
}

void sendStatus(uint32_t nowMs) {
  if (!paired || nowMs - lastStatusMs < kStatusIntervalMs) return;
  lastStatusMs = nowMs;
  telemetry::BlackBoxStatusPacket status{};
  telemetry::preparePacket(status, telemetry::PacketType::BlackBoxStatus);
  status.sequence = statusSequence++;
  status.uptime_ms = nowMs;
  status.rows_written = rowsWritten;
  status.sd_free_mb = sdFreeMb;
  if (sdReady) status.flags |= telemetry::BlackBoxSdReady;
  if (sdReady && lastWrittenMs != 0 &&
      nowMs - lastWrittenMs < kRecordingFreshMs) {
    status.flags |= telemetry::BlackBoxRecording;
  }
  const int slash = currentPath.lastIndexOf('/');
  std::snprintf(status.file, sizeof(status.file), "%s",
                currentPath.substring(slash + 1).c_str());
  telemetry::sealPacket(status);
  esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&status),
               sizeof(status));
}

void updateRadio(uint32_t nowMs) {
  portENTER_CRITICAL(&receiveMux);
  const uint32_t heardMs = lastHeardMs;
  portEXIT_CRITICAL(&receiveMux);
  if (paired && telemetry::elapsedMs(nowMs, heardMs) > kLinkStaleMs) {
    esp_now_del_peer(peerMac);
    portENTER_CRITICAL(&receiveMux);
    paired = false;
    portEXIT_CRITICAL(&receiveMux);
    pendingSequence = 0;
    Serial.println("Logger link lost");
  }
  connectCandidate(nowMs);
  if (!paired && nowMs - lastScanMs >= kScanDwellMs) {
    lastScanMs = nowMs;
    const uint8_t nextChannel = channel >= kChannels ? 1 : channel + 1;
    if (esp_wifi_set_channel(nextChannel, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
      portENTER_CRITICAL(&receiveMux);
      channel = nextChannel;
      portEXIT_CRITICAL(&receiveMux);
    }
  }
  if (paired && nowMs - lastHeartbeatMs >= kHeartbeatMs) {
    lastHeartbeatMs = nowMs;
    sendCommand(telemetry::DialAction::Heartbeat);
  }
  sendStatus(nowMs);

  telemetry::DialAckPacket ack{};
  bool gotAck = false;
  portENTER_CRITICAL(&receiveMux);
  if (ackReady) {
    ack = receivedAck;
    ackReady = false;
    gotAck = true;
  }
  portEXIT_CRITICAL(&receiveMux);
  if (gotAck && pendingSequence != 0 &&
      ack.command_sequence == pendingSequence) {
    pendingSequence = 0;
    flash(ack.accepted ? 0x00FF00 : 0xFF0000, 600);
  }
  if (pendingSequence != 0 && nowMs - lastPendingSendMs >= 1000 &&
      pendingAttempts < 3) {
    esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&pendingPacket),
                 sizeof(pendingPacket));
    lastPendingSendMs = nowMs;
    ++pendingAttempts;
  }
  if (pendingSequence != 0 && nowMs - pendingSinceMs > kAckTimeoutMs) {
    pendingSequence = 0;
    flash(0xFF0000, 600);
  }
}


TempProbes probes;

void sendTemperatures(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  static uint32_t sequence = 0;
  if (!paired || nowMs - lastMs < 10'000) return;
  lastMs = nowMs;
  telemetry::TemperatureReportPacket report{};
  if (!makeTemperatureReport(probes, telemetry::ProbeSource::BlackBox, sequence++,
                             report)) {
    return;
  }
  esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&report),
               sizeof(report));
}

// --------------------------------------------------------------- SD card

void mountSd(uint32_t nowMs) {
  if (sdReady || (lastSdAttemptMs != 0 && nowMs - lastSdAttemptMs < kSdRetryMs))
    return;
  lastSdAttemptMs = nowMs;
  SD.end();
  SPI.end();
  SPI.begin(kSdSckPin, kSdMisoPin, kSdMosiPin, -1);
  sdReady = SD.begin(-1, SPI, kSdFrequency) && SD.cardType() != CARD_NONE;
  if (sdReady) {
    if (!SD.exists(kDirectory)) SD.mkdir(kDirectory);
    lastFreeSpaceMs = 0;
    Serial.printf("SD ready, %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  } else {
    Serial.println("SD not found");
  }
}

void sdFailed() {
  sdReady = false;
  currentPath = "";
  Serial.println("SD write failed; will retry");
}

void flushRows(uint32_t nowMs) {
  lastFlushMs = nowMs;
  if (bufferedRows == 0 || !sdReady || currentPath.isEmpty()) return;
  const bool isNew = !SD.exists(currentPath);
  File file = SD.open(currentPath, FILE_APPEND);
  if (!file) {
    sdFailed();
    return;
  }
  if (isNew) {
    file.print(
        "utc,local_time,latitude,longitude,altitude_m,speed_kmh,course_deg,"
        "satellites,hdop,fix,log_mode,accel_x,accel_y,accel_z,gyro_x,gyro_y,"
        "gyro_z,pitch_deg,roll_deg,vibration_rms,logger_board,sequence,"
        "probe_c\n");
  }
  const size_t written = file.print(rowBuffer);
  file.close();
  if (written != rowBuffer.length()) {
    sdFailed();
    return;
  }
  rowsWritten += bufferedRows;
  rowBuffer = "";
  bufferedRows = 0;
}

void updateFreeSpace(uint32_t nowMs) {
  // usedBytes() walks the FAT, so it is only checked once a minute.
  if (!sdReady || (lastFreeSpaceMs != 0 &&
                   nowMs - lastFreeSpaceMs < kFreeSpaceIntervalMs)) {
    return;
  }
  lastFreeSpaceMs = nowMs;
  sdFreeMb = static_cast<uint32_t>((SD.totalBytes() - SD.usedBytes()) /
                                   (1024ULL * 1024ULL));
}

// Daily files by the logger's local date; rows before the logger knows the
// time go to no-clock.csv.
String pathFor(const telemetry::PositionReportPacket& report) {
  if (!(report.flags & telemetry::PositionUtcValid)) {
    return String(kDirectory) + "/no-clock.csv";
  }
  const time_t local = static_cast<time_t>(report.utc_ms / 1000ULL) +
                       report.utc_offset_min * 60;
  struct tm day {};
  gmtime_r(&local, &day);
  char path[40];
  std::snprintf(path, sizeof(path), "%s/%04d-%02d-%02d.csv", kDirectory,
                day.tm_year + 1900, day.tm_mon + 1, day.tm_mday);
  return String(path);
}

void appendRow(const telemetry::PositionReportPacket& report, uint32_t nowMs) {
  const String path = pathFor(report);
  if (path != currentPath) {
    flushRows(nowMs);
    currentPath = path;
  }
  char utc[24] = "";
  char local[24] = "";
  if (report.flags & telemetry::PositionUtcValid) {
    const time_t seconds = static_cast<time_t>(report.utc_ms / 1000ULL);
    struct tm parts {};
    gmtime_r(&seconds, &parts);
    strftime(utc, sizeof(utc), "%Y-%m-%dT%H:%M:%SZ", &parts);
    const time_t localSeconds = seconds + report.utc_offset_min * 60;
    gmtime_r(&localSeconds, &parts);
    strftime(local, sizeof(local), "%Y-%m-%d %H:%M:%S", &parts);
  }
  const bool position = report.flags & telemetry::PositionValid;
  const bool imu = report.flags & telemetry::PositionImuValid;
  char row[320];
  int length = std::snprintf(
      row, sizeof(row), "%s,%s,", utc, local);
  if (position) {
    length += std::snprintf(row + length, sizeof(row) - length,
                            "%.7f,%.7f,%.1f,%.1f,%.1f,", report.latitude_deg,
                            report.longitude_deg, report.altitude_m,
                            report.speed_kmh, report.course_deg);
  } else {
    length += std::snprintf(row + length, sizeof(row) - length, ",,,,,");
  }
  length += std::snprintf(row + length, sizeof(row) - length, "%u,%.1f,%u,%u,",
                          report.satellites, report.hdop,
                          (report.flags & telemetry::PositionFix) ? 1U : 0U,
                          report.log_mode);
  if (imu) {
    length += std::snprintf(
        row + length, sizeof(row) - length,
        "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,", report.accel_x_mps2,
        report.accel_y_mps2, report.accel_z_mps2, report.gyro_x_dps,
        report.gyro_y_dps, report.gyro_z_dps, report.pitch_deg,
        report.roll_deg, report.vibration_rms_mps2);
  } else {
    length += std::snprintf(row + length, sizeof(row) - length, ",,,,,,,,,");
  }
  length += std::snprintf(row + length, sizeof(row) - length, "%u,%lu,",
                          report.logger_board,
                          static_cast<unsigned long>(report.sequence));
  float probe = NAN;
  for (int i = 0; i < probes.count() && std::isnan(probe); ++i) {
    probe = probes.reading(i).celsius;
  }
  if (std::isnan(probe)) {
    std::snprintf(row + length, sizeof(row) - length, "\n");
  } else {
    std::snprintf(row + length, sizeof(row) - length, "%.1f\n", probe);
  }
  rowBuffer += row;
  ++bufferedRows;
  lastWrittenMs = nowMs;
}

void updateRecording(uint32_t nowMs) {
  mountSd(nowMs);
  telemetry::PositionReportPacket report{};
  for (;;) {
    bool have = false;
    portENTER_CRITICAL(&receiveMux);
    if (queueCount > 0) {
      report = queue[queueHead];
      queueHead = (queueHead + 1) % kQueueSize;
      --queueCount;
      have = true;
    }
    portEXIT_CRITICAL(&receiveMux);
    if (!have) break;
    if (sdReady) appendRow(report, nowMs);
  }
  // Rows are written in batches to spare the card.
  if (bufferedRows >= kFlushRows ||
      (bufferedRows > 0 && nowMs - lastFlushMs >= kFlushIntervalMs)) {
    flushRows(nowMs);
  }
  updateFreeSpace(nowMs);
}

// ------------------------------------------------------- button and light

void handleButton() {
  if (!M5.BtnA.wasClicked()) return;
  if (pendingSequence != 0) return;
  if (!paired || !sendCommand(telemetry::DialAction::MarkWaypoint,
                              kWaypointCategory)) {
    flash(0xFF0000, 600);
  }
}

uint32_t targetColor() {
  return targetBoard == static_cast<uint8_t>(telemetry::LoggerBoard::Cardputer)
             ? 0xFF00FF   // magenta: Cardputer v1
             : 0x00FFFF;  // cyan: Cardputer ADV
}

void updateLed(uint32_t nowMs) {
  uint32_t color;
  if (static_cast<int32_t>(flashUntilMs - nowMs) > 0) {
    color = flashColor;
  } else if (pendingSequence != 0) {
    color = 0xFFFFFF;
  } else if (!sdReady) {
    color = (nowMs / 300) % 2 ? 0xFF0000 : 0;
  } else if (!paired) {
    color = (nowMs / 500) % 2 ? 0x0000FF : 0;
  } else if (lastWrittenMs != 0 && nowMs - lastWrittenMs < kRecordingFreshMs) {
    color = 0x00FF00;
  } else {
    color = 0xFF8000;
  }
  static uint32_t shown = 0xFFFFFFFFU;
  if (color == shown) return;
  shown = color;
  M5.Led.setAllColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
}

void blinkTarget(int times) {
  const uint32_t color = targetColor();
  for (int i = 0; i < times; ++i) {
    M5.Led.setAllColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
    delay(250);
    M5.Led.setAllColor(0, 0, 0);
    delay(200);
  }
}

void printReport(uint32_t nowMs) {
  static uint32_t lastReportMs = 0;
  if (nowMs - lastReportMs < 5000) return;
  lastReportMs = nowMs;
  Serial.printf("%s ch %u, SD %s, rows %lu, file %s, free %lu MB\n",
                paired ? "Linked" : "Searching", channel,
                sdReady ? "ready" : "missing",
                static_cast<unsigned long>(rowsWritten), currentPath.c_str(),
                static_cast<unsigned long>(sdFreeMb));
}

}  // namespace

void setup() {
  auto config = M5.config();
  config.internal_mic = false;
  config.internal_spk = false;
  config.internal_imu = false;
  config.internal_rtc = false;
  config.external_imu = false;
  config.external_rtc = false;
  M5.begin(config);
  Serial.begin(115200);
  M5.Led.setBrightness(40);

  preferences.begin("blackbox", false);
  targetBoard = preferences.getUChar(
      "target", static_cast<uint8_t>(telemetry::LoggerBoard::CardputerAdv));
  // Button held through a reset: toggle between the two Cardputers.
  M5.update();
  if (M5.BtnA.isPressed()) {
    targetBoard =
        targetBoard == static_cast<uint8_t>(telemetry::LoggerBoard::Cardputer)
            ? static_cast<uint8_t>(kTargets[0])
            : static_cast<uint8_t>(kTargets[1]);
    preferences.putUChar("target", targetBoard);
    blinkTarget(4);
    while (M5.BtnA.isPressed()) {  // Let go before normal use begins.
      delay(10);
      M5.update();
    }
  } else {
    blinkTarget(1);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  nextSequence = esp_random();
  if (nextSequence == 0) nextSequence = 1;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(onReceive) != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
  }
  uint8_t mac[6]{};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Black box %02X:%02X:%02X:%02X:%02X:%02X, firmware %s %s %s, "
                "pairs with board %u\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                version::kNumber, version::kGit, version::kDate, targetBoard);
  mountSd(millis());
  probes.begin({26, 32});  // DS18B20 probes on the Grove port
}

void loop() {
  M5.update();
  const uint32_t nowMs = millis();
  updateRadio(nowMs);
  probes.update(nowMs);
  sendTemperatures(nowMs);
  updateRecording(nowMs);
  handleButton();
  updateLed(nowMs);
  printReport(nowMs);
  delay(5);
}
