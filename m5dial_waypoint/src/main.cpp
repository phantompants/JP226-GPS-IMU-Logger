#include <Arduino.h>
#include <M5Dial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "TelemetryProtocol.h"

namespace {

constexpr uint8_t kChannels = 13;
constexpr uint32_t kScanDwellMs = 450;
constexpr uint32_t kHeartbeatMs = 1000;
constexpr uint32_t kStatusStaleMs = 5000;
constexpr const char* kCategories[] = {
    "GENERIC", "PHOTO", "CAMP", "FUEL", "LOOKOUT", "TRACK",
    "HAZARD", "INTERESTING", "TEST POINT"};
constexpr const char* kRoads[] = {"PAVED", "GRAVEL", "DIRT", "TRAIL"};
constexpr const char* kSuspension[] = {"SOFT", "NORMAL", "FIRM"};
constexpr const char* kLoads[] = {"SOLO", "LIGHT", "HEAVY"};
constexpr const char* kFieldNames[] = {"ROAD", "FRONT PSI", "REAR PSI",
                                       "FRONT SUSP", "REAR SUSP", "LOAD"};

portMUX_TYPE receiveMux = portMUX_INITIALIZER_UNLOCKED;
telemetry::DialStatusPacket latestStatus{};
telemetry::DialStatusPacket receivedStatus{};
telemetry::DialAckPacket receivedAck{};
uint8_t candidateMac[6]{};
uint8_t candidateChannel = 0;
bool candidateReady = false;
bool statusReady = false;
bool ackReady = false;
bool haveStatus = false;
bool paired = false;
uint8_t peerMac[6]{};
uint8_t channel = 1;
uint32_t lastScanMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t nextSequence = 1;
uint32_t pendingSequence = 0;
uint32_t pendingSinceMs = 0;
uint32_t lastPendingSendMs = 0;
uint8_t pendingAttempts = 0;
telemetry::DialCommandPacket pendingPacket{};
uint32_t messageSinceMs = 0;
long lastEncoderPosition = 0;
bool longPressHandled = false;
enum class DialPage : uint8_t { Drive, Waypoint, Settings };
DialPage page = DialPage::Drive;
bool editing = false;
uint8_t categoryIndex = 0;
uint8_t fieldIndex = 0;
uint8_t roadIndex = 0;
uint8_t frontSuspensionIndex = 1;
uint8_t rearSuspensionIndex = 1;
uint8_t loadIndex = 0;
int frontPsi = 30;
int rearPsi = 30;
String message;

template <size_t N>
void cycleIndex(uint8_t& index, int delta, const char* const (&values)[N]) {
  (void)values;
  int next = static_cast<int>(index) + delta;
  while (next < 0) next += N;
  index = static_cast<uint8_t>(next % N);
}

void onReceive(const esp_now_recv_info_t* info, const uint8_t* bytes,
               int length) {
  if (info == nullptr || info->src_addr == nullptr || bytes == nullptr ||
      length < static_cast<int>(sizeof(telemetry::PacketHeader))) return;
  telemetry::PacketHeader header{};
  std::memcpy(&header, bytes, sizeof(header));
  if (header.magic != telemetry::kMagic ||
      header.version != telemetry::kProtocolVersion ||
      header.size != length) return;
  bool currentlyPaired = false;
  bool samePeer = false;
  portENTER_CRITICAL(&receiveMux);
  currentlyPaired = paired;
  samePeer = currentlyPaired &&
             std::memcmp(peerMac, info->src_addr, 6) == 0;
  portEXIT_CRITICAL(&receiveMux);
  const auto type = static_cast<telemetry::PacketType>(header.type);
  if (type == telemetry::PacketType::Discovery &&
      length == sizeof(telemetry::DiscoveryPacket)) {
    telemetry::DiscoveryPacket discovery{};
    std::memcpy(&discovery, bytes, sizeof(discovery));
    if (!telemetry::validatePacket(discovery, type)) return;
    if (currentlyPaired && !samePeer) return;
    portENTER_CRITICAL(&receiveMux);
    std::memcpy(candidateMac, info->src_addr, 6);
    candidateChannel = info->rx_ctrl != nullptr ? info->rx_ctrl->channel
                                                : channel;
    candidateReady = true;
    portEXIT_CRITICAL(&receiveMux);
  } else if (samePeer &&
             type == telemetry::PacketType::DialStatus &&
             length == sizeof(telemetry::DialStatusPacket)) {
    telemetry::DialStatusPacket status{};
    std::memcpy(&status, bytes, sizeof(status));
    if (!telemetry::validatePacket(status, type)) return;
    portENTER_CRITICAL(&receiveMux);
    receivedStatus = status;
    statusReady = true;
    portEXIT_CRITICAL(&receiveMux);
  } else if (samePeer &&
             type == telemetry::PacketType::DialAck &&
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

void showMessage(const String& value) {
  message = value;
  messageSinceMs = millis();
}

void sendCommand(telemetry::DialAction action, const char* text = "",
                 float value = 0.0f) {
  if (action != telemetry::DialAction::Heartbeat && pendingSequence != 0) {
    showMessage("WAIT FOR CONFIRMATION");
    return;
  }
  if (!paired) {
    showMessage("NO CARDPUTER LINK");
    return;
  }
  telemetry::DialCommandPacket packet{};
  telemetry::preparePacket(packet, telemetry::PacketType::DialCommand);
  packet.sequence = nextSequence++;
  packet.action = static_cast<uint8_t>(action);
  packet.value = value;
  std::snprintf(packet.text, sizeof(packet.text), "%s", text);
  telemetry::sealPacket(packet);
  if (esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&packet),
                   sizeof(packet)) != ESP_OK) {
    showMessage("SEND FAILED");
    return;
  }
  if (action != telemetry::DialAction::Heartbeat) {
    pendingPacket = packet;
    pendingSequence = packet.sequence;
    pendingSinceMs = millis();
    lastPendingSendMs = pendingSinceMs;
    pendingAttempts = 1;
    showMessage("WAITING FOR ACK");
  }
}

void connectCandidate() {
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
  portEXIT_CRITICAL(&receiveMux);
  lastStatusMs = millis();
  lastHeartbeatMs = 0;
  showMessage("CARDPUTER FOUND");
}

void updateRadio(uint32_t nowMs) {
  if (paired && nowMs - lastStatusMs > kStatusStaleMs) {
    esp_now_del_peer(peerMac);
    portENTER_CRITICAL(&receiveMux);
    paired = false;
    portEXIT_CRITICAL(&receiveMux);
    haveStatus = false;
    pendingSequence = 0;
    showMessage("LINK LOST");
  }
  connectCandidate();
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
  telemetry::DialStatusPacket status{};
  telemetry::DialAckPacket ack{};
  bool gotStatus = false;
  bool gotAck = false;
  portENTER_CRITICAL(&receiveMux);
  if (statusReady) {
    status = receivedStatus;
    statusReady = false;
    gotStatus = true;
  }
  if (ackReady) {
    ack = receivedAck;
    ackReady = false;
    gotAck = true;
  }
  portEXIT_CRITICAL(&receiveMux);
  if (gotStatus) {
    latestStatus = status;
    haveStatus = true;
    lastStatusMs = nowMs;
    if (!editing) {
      if (std::isfinite(status.front_psi))
        frontPsi = static_cast<int>(std::lround(status.front_psi));
      if (std::isfinite(status.rear_psi))
        rearPsi = static_cast<int>(std::lround(status.rear_psi));
    }
  }
  if (gotAck && ack.command_sequence == pendingSequence) {
    pendingSequence = 0;
    if (ack.accepted) {
      showMessage(ack.waypoint_id[0] ? String("SAVED ") + ack.waypoint_id
                                     : "SETTING SAVED");
      M5Dial.Speaker.tone(1200, 80);
    } else {
      showMessage("REJECTED: CHECK FIX/SD");
      M5Dial.Speaker.tone(350, 180);
    }
  }
  if (pendingSequence != 0 && nowMs - lastPendingSendMs >= 1000 &&
      pendingAttempts < 3) {
    esp_now_send(peerMac,
                 reinterpret_cast<const uint8_t*>(&pendingPacket),
                 sizeof(pendingPacket));
    lastPendingSendMs = nowMs;
    ++pendingAttempts;
  }
  if (pendingSequence != 0 && nowMs - pendingSinceMs > 3500) {
    pendingSequence = 0;
    showMessage("NO CONFIRMATION");
  }
}

void commitField() {
  switch (fieldIndex) {
    case 0:
      sendCommand(telemetry::DialAction::SetRoad, kRoads[roadIndex]);
      break;
    case 1:
      sendCommand(telemetry::DialAction::SetTyreFront, "", frontPsi);
      break;
    case 2:
      sendCommand(telemetry::DialAction::SetTyreRear, "", rearPsi);
      break;
    case 3:
      sendCommand(telemetry::DialAction::SetSuspensionFront,
                  kSuspension[frontSuspensionIndex]);
      break;
    case 4:
      sendCommand(telemetry::DialAction::SetSuspensionRear,
                  kSuspension[rearSuspensionIndex]);
      break;
    case 5:
      sendCommand(telemetry::DialAction::SetLoad, kLoads[loadIndex]);
      break;
  }
}

String fieldValue() {
  switch (fieldIndex) {
    case 0: return kRoads[roadIndex];
    case 1: return String(frontPsi) + " PSI";
    case 2: return String(rearPsi) + " PSI";
    case 3: return kSuspension[frontSuspensionIndex];
    case 4: return kSuspension[rearSuspensionIndex];
    default: return kLoads[loadIndex];
  }
}

const char* logModeText(uint8_t mode) {
  switch (mode) {
    case 1: return "MOVE";
    case 2: return "PARK15";
    case 3: return "PARK60";
    case 4: return "FIX LOST";
    default: return "WAIT";
  }
}

void handleEncoder(int delta) {
  if (page == DialPage::Waypoint) {
    cycleIndex(categoryIndex, delta, kCategories);
  } else if (page == DialPage::Settings && !editing) {
    int next = static_cast<int>(fieldIndex) + delta;
    while (next < 0) next += 6;
    fieldIndex = static_cast<uint8_t>(next % 6);
  } else if (page == DialPage::Settings) {
    switch (fieldIndex) {
      case 0: cycleIndex(roadIndex, delta, kRoads); break;
      case 1: frontPsi = std::clamp(frontPsi + delta, 0, 100); break;
      case 2: rearPsi = std::clamp(rearPsi + delta, 0, 100); break;
      case 3:
        cycleIndex(frontSuspensionIndex, delta, kSuspension);
        break;
      case 4:
        cycleIndex(rearSuspensionIndex, delta, kSuspension);
        break;
      case 5: cycleIndex(loadIndex, delta, kLoads); break;
    }
  }
}

void handleControls() {
  const long position = M5Dial.Encoder.read() / 4;
  if (position != lastEncoderPosition) {
    const long difference = position - lastEncoderPosition;
    lastEncoderPosition = position;
    handleEncoder(static_cast<int>(std::clamp(difference, -20L, 20L)));
  }
  if (M5Dial.BtnA.pressedFor(900) && !longPressHandled) {
    longPressHandled = true;
    page = page == DialPage::Drive ? DialPage::Waypoint
                                  : page == DialPage::Waypoint
                                        ? DialPage::Settings
                                        : DialPage::Drive;
    editing = false;
    showMessage(page == DialPage::Drive ? "DRIVE"
                : page == DialPage::Waypoint ? "WAYPOINT" : "SETTINGS");
  }
  if (M5Dial.BtnA.wasReleased()) {
    if (!longPressHandled) {
      if (page == DialPage::Waypoint) {
        sendCommand(telemetry::DialAction::MarkWaypoint,
                    kCategories[categoryIndex]);
      } else if (page == DialPage::Drive) {
        page = DialPage::Waypoint;
      } else if (!editing) {
        editing = true;
      } else {
        commitField();
        editing = false;
      }
    }
    longPressHandled = false;
  }
}

void draw() {
  auto& display = M5Dial.Display;
  display.fillScreen(BLACK);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);
  display.setCursor(28, 12);
  display.print(paired && haveStatus ? "CARDPUTER LINK" : "SEARCHING...");
  if (page == DialPage::Settings) {
    display.setCursor(27, 48);
    display.print("CONTEXT SETTINGS");
    display.setCursor(27, 80);
    display.printf("%u/6  %s", fieldIndex + 1, kFieldNames[fieldIndex]);
    display.setTextSize(2);
    display.setCursor(26, 108);
    display.print(fieldValue());
    display.setTextSize(1);
    display.setCursor(27, 153);
    display.print(editing ? "TURN TO CHANGE" : "TURN TO SELECT");
    display.setCursor(27, 169);
    display.print("PRESS TO SAVE");
  } else if (page == DialPage::Waypoint) {
    display.setCursor(30, 49);
    display.print("WAYPOINT");
    display.setTextSize(2);
    display.setCursor(28, 79);
    display.print(kCategories[categoryIndex]);
    display.setTextSize(1);
    display.setCursor(28, 116);
    display.print("TURN: CATEGORY");
    display.setCursor(28, 134);
    display.print("PRESS: SAVE HERE");
    display.setCursor(28, 157);
    display.printf("LAST: %.20s", latestStatus.last_waypoint);
    display.setCursor(28, 175);
    display.printf("PLACE: %.20s", latestStatus.place);
  } else {
    display.setTextSize(4);
    display.setCursor(35, 36);
    if (haveStatus && latestStatus.fix_valid) {
      display.printf("%.0f", latestStatus.speed_kmh);
    } else {
      display.print("--");
    }
    display.setTextSize(1);
    display.setCursor(145, 70);
    display.print("km/h");
    display.setCursor(25, 97);
    if (haveStatus) {
      display.printf("FIX %s  SAT %u  %s",
                     latestStatus.fix_valid ? "YES" : "NO",
                     latestStatus.satellites,
                     logModeText(latestStatus.log_mode));
      display.setCursor(25, 105);
      display.printf("ELAPSED %02lu:%02lu",
                     static_cast<unsigned long>(latestStatus.log_elapsed_s / 3600),
                     static_cast<unsigned long>(
                         (latestStatus.log_elapsed_s / 60) % 60));
      display.setCursor(25, 121);
      display.printf("ROAD %.14s", latestStatus.road);
      display.setCursor(25, 137);
      display.printf("TYRE %.0f / %.0f PSI", latestStatus.front_psi,
                     latestStatus.rear_psi);
      display.setCursor(25, 153);
      display.printf("SUSP %.10s/%.10s", latestStatus.suspension_front,
                     latestStatus.suspension_rear);
      display.setCursor(25, 169);
      display.printf("LOAD %.10s", latestStatus.vehicle_load);
      display.setCursor(25, 185);
      display.printf("POI %.25s", latestStatus.place);
    }
    display.setCursor(25, 201);
    display.print("PRESS: WAYPOINT");
  }
  if (!message.isEmpty() && millis() - messageSinceMs < 3500) {
    display.setTextColor(YELLOW, BLACK);
    display.setCursor(50, 214);
    display.print(message.substring(0, 22));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  M5Dial.begin(M5.config(), true, false);
  M5Dial.Display.setRotation(0);
  M5Dial.Display.setBrightness(120);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  nextSequence = esp_random();
  if (nextSequence == 0) nextSequence = 1;
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(onReceive) != ESP_OK) {
    showMessage("ESP-NOW INIT FAILED");
  }
  lastEncoderPosition = M5Dial.Encoder.read() / 4;
}

void loop() {
  M5Dial.update();
  const uint32_t nowMs = millis();
  updateRadio(nowMs);
  handleControls();
  static uint32_t lastDrawMs = 0;
  if (nowMs - lastDrawMs >= 250) {
    lastDrawMs = nowMs;
    draw();
  }
  delay(5);
}
