#include <Arduino.h>
#include <M5Dial.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstring>

#include "DinoSprite.h"
#include "TempProbe.h"
#include "Version.h"
#include "TelemetryProtocol.h"

namespace {

constexpr uint8_t kChannels = 13;
constexpr uint32_t kScanDwellMs = 450;
constexpr uint32_t kHeartbeatMs = 1000;
constexpr uint32_t kStatusStaleMs = 5000;
// With several loggers in range, the Dial only pairs with the chosen board
// type. Tapping on the CONNECTION page steps through these; Unknown pairs
// with any logger. The choice is saved.
constexpr telemetry::LoggerBoard kTargetBoards[] = {
    telemetry::LoggerBoard::CardputerAdv, telemetry::LoggerBoard::Cardputer,
    telemetry::LoggerBoard::Core2, telemetry::LoggerBoard::Unknown};
constexpr size_t kTargetBoardCount =
    sizeof(kTargetBoards) / sizeof(kTargetBoards[0]);
// Read by the receive callback; a single byte, so no lock is needed.
volatile uint8_t targetBoard =
    static_cast<uint8_t>(telemetry::LoggerBoard::CardputerAdv);
Preferences preferences;
dino::Game dinoGame;
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
int8_t receivedRssi = 0;
int8_t latestRssi = 0;
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
enum class DialPage : uint8_t {
  Drive, Road, Waypoint, Settings, Link, Version, About, Count
};
DialPage page = DialPage::Drive;
bool editing = false;
uint8_t categoryIndex = 0;
// SETTINGS starts at field 1: road type has its own ROAD page now.
uint8_t fieldIndex = 1;
uint8_t roadIndex = 0;
// ROAD page: the surface is sent a second after the knob stops turning.
uint8_t surfaceIndex = 1;
bool surfaceDirty = false;
uint32_t surfaceTurnedMs = 0;
uint8_t frontSuspensionIndex = 1;
uint8_t rearSuspensionIndex = 1;
uint8_t loadIndex = 0;
int frontPsi = 30;
int rearPsi = 30;
String message;
// Frames are drawn off-screen and pushed in one transfer to avoid flicker.
// No parent here: M5Dial.Display is a reference that may not be bound yet
// when this global is constructed, so the target is passed at push time.
M5Canvas canvas;
bool canvasReady = false;

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
    if (!telemetry::boardMatches(
            static_cast<telemetry::LoggerBoard>(targetBoard),
            telemetry::beaconBoard(discovery.time_flags))) return;
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
    receivedRssi = info->rx_ctrl != nullptr
                       ? static_cast<int8_t>(info->rx_ctrl->rssi)
                       : 0;
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
    showMessage("NO LOGGER LINK");
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
  showMessage("LOGGER FOUND");
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
    latestRssi = receivedRssi;
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
    if (!surfaceDirty && status.road_surface > 0 &&
        status.road_surface < telemetry::kRoadSurfaceCount) {
      surfaceIndex = status.road_surface;
    }
    for (uint8_t i = 0; i < sizeof(kRoads) / sizeof(kRoads[0]); ++i) {
      if (std::strncmp(status.road, kRoads[i], sizeof(status.road)) == 0) {
        roadIndex = i;
      }
    }
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


TempProbes probes;

// First probe that answered, or NaN.
float firstProbeCelsius() {
  for (int i = 0; i < probes.count(); ++i) {
    if (!std::isnan(probes.reading(i).celsius)) return probes.reading(i).celsius;
  }
  return NAN;
}

void sendTemperatures(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  static uint32_t sequence = 0;
  if (!paired || nowMs - lastMs < 10'000) return;
  lastMs = nowMs;
  telemetry::TemperatureReportPacket report{};
  if (!makeTemperatureReport(probes, telemetry::ProbeSource::Dial, sequence++,
                             report)) {
    return;
  }
  esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&report),
               sizeof(report));
}

void cycleTargetBoard() {
  size_t index = 0;
  while (index < kTargetBoardCount &&
         static_cast<uint8_t>(kTargetBoards[index]) != targetBoard) {
    ++index;
  }
  targetBoard = static_cast<uint8_t>(kTargetBoards[(index + 1) % kTargetBoardCount]);
  preferences.putUChar("target", targetBoard);
  if (paired) {
    esp_now_del_peer(peerMac);
    portENTER_CRITICAL(&receiveMux);
    paired = false;
    portEXIT_CRITICAL(&receiveMux);
    haveStatus = false;
    pendingSequence = 0;
  }
  showMessage("PAIRING CHANGED");
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
  if (page == DialPage::About) {
    dinoGame.press(millis());
  } else if (page == DialPage::Road) {
    int next = static_cast<int>(surfaceIndex) - 1 + delta;
    const int count = telemetry::kRoadSurfaceCount - 1;
    while (next < 0) next += count;
    surfaceIndex = static_cast<uint8_t>(next % count + 1);
    surfaceDirty = true;
    surfaceTurnedMs = millis();
  } else if (page == DialPage::Waypoint) {
    cycleIndex(categoryIndex, delta, kCategories);
  } else if (page == DialPage::Settings && !editing) {
    // Fields 1-5; road type (field 0) is on the ROAD page.
    int next = static_cast<int>(fieldIndex) - 1 + delta;
    while (next < 0) next += 5;
    fieldIndex = static_cast<uint8_t>(next % 5 + 1);
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
  if (page == DialPage::About && M5Dial.BtnA.wasPressed()) {
    dinoGame.press(millis());
  }
  if (M5Dial.BtnA.pressedFor(900) && !longPressHandled) {
    longPressHandled = true;
    page = static_cast<DialPage>((static_cast<uint8_t>(page) + 1) %
                                 static_cast<uint8_t>(DialPage::Count));
    editing = false;
  }
  if (M5Dial.BtnA.wasReleased()) {
    if (!longPressHandled) {
      if (page == DialPage::Waypoint) {
        sendCommand(telemetry::DialAction::MarkWaypoint,
                    kCategories[categoryIndex]);
      } else if (page == DialPage::Drive) {
        page = DialPage::Waypoint;
      } else if (page == DialPage::Road) {
        cycleIndex(roadIndex, 1, kRoads);
        sendCommand(telemetry::DialAction::SetRoad, kRoads[roadIndex]);
      } else if (page == DialPage::Link) {
        cycleTargetBoard();
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

// The Dial's screen is a 240 px circle. Each line is centred and sized to the
// chord at its height: it drops a text size, then clips, rather than run
// under the bezel.
constexpr int32_t kCenter = 120;
constexpr int32_t kRadius = 120;
constexpr int32_t kEdgeMargin = 6;

int32_t chordWidth(int32_t top, int32_t height) {
  const int32_t d = std::max(std::abs(top - kCenter),
                             std::abs(top + height - kCenter));
  if (d >= kRadius) return 0;
  const float half = std::sqrt(static_cast<float>(kRadius * kRadius - d * d));
  return static_cast<int32_t>(2 * half) - 2 * kEdgeMargin;
}

void centerText(M5Canvas& display, int32_t y, uint8_t size, uint16_t color,
                const char* format, ...) {
  char text[48];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  size_t length = strlen(text);
  // The built-in font is 6x8 px per character at size 1.
  while (size > 1 &&
         static_cast<int32_t>(length) * 6 * size > chordWidth(y, 8 * size)) {
    --size;
  }
  const size_t fits = std::max<int32_t>(chordWidth(y, 8 * size), 0) / (6 * size);
  if (length > fits) text[fits] = '\0';
  display.setTextSize(size);
  display.setTextColor(color, BLACK);
  display.drawCenterString(text, kCenter, y);
}

uint16_t batteryColor(int level) {
  return level < 20 ? RED : level < 50 ? ORANGE : GREEN;
}

// M5Unified cannot measure an M5Dial battery, so the Dial shows its own level
// only if a future board reports one, plus the Cardputer's relayed battery.
void drawBattery(M5Canvas& display, int32_t y) {
  const int32_t ownLevel = M5.Power.getBatteryLevel();
  const bool haveOwn = ownLevel >= 0 && ownLevel <= 100;
  if (haveStatus && latestStatus.battery_percent >= 0) {
    const int level = latestStatus.battery_percent;
    if (haveOwn) {
      centerText(display, y, 2, batteryColor(level), "BAT %d%% DIAL %ld%%",
                 level, static_cast<long>(ownLevel));
    } else {
      centerText(display, y, 2, batteryColor(level), "LOGGER BAT %d%%", level);
    }
  } else if (haveOwn) {
    centerText(display, y, 2, WHITE, "DIAL BAT %ld%%",
               static_cast<long>(ownLevel));
  }
}

// Link state as a ring around the bezel, and page position as a row of dots.
void drawFrame(M5Canvas& display, bool linked) {
  display.fillArc(kCenter, kCenter, kRadius - 1, kRadius - 4, 0, 360,
                  linked ? GREEN : ORANGE);
  const int current = static_cast<int>(page);
  constexpr int kPages = static_cast<int>(DialPage::Count);
  for (int i = 0; i < kPages; ++i) {
    const int32_t x = kCenter + (2 * i - (kPages - 1)) * 7;
    if (i == current) display.fillCircle(x, 224, 4, WHITE);
    else display.drawCircle(x, 224, 4, TFT_DARKGREY);
  }
}

void drawHint(M5Canvas& display, const char* hint) {
  if (!message.isEmpty() && millis() - messageSinceMs < 3500) {
    centerText(display, 196, 2, YELLOW, "%s", message.c_str());
  } else {
    centerText(display, 202, 1, TFT_DARKGREY, "%s", hint);
  }
}

void drawDrivePage(M5Canvas& display) {
  const bool live = haveStatus && latestStatus.fix_valid;
  if (live) {
    centerText(display, 36, 6, WHITE, "%.0f", latestStatus.speed_kmh);
  } else {
    centerText(display, 36, 6, TFT_DARKGREY, "--");
  }
  centerText(display, 86, 2, TFT_LIGHTGREY, "km/h");
  if (haveStatus) {
    centerText(display, 106, 2, latestStatus.fix_valid ? GREEN : RED,
               "%s %u SAT", latestStatus.fix_valid ? "FIX" : "NO FIX",
               latestStatus.satellites);
    centerText(display, 124, 2, WHITE, "%s %02lu:%02lu",
               logModeText(latestStatus.log_mode),
               static_cast<unsigned long>(latestStatus.log_elapsed_s / 3600),
               static_cast<unsigned long>(
                   (latestStatus.log_elapsed_s / 60) % 60));
    centerText(display, 142, 2, WHITE, "%.8s %.0f/%.0f", latestStatus.road,
               latestStatus.front_psi, latestStatus.rear_psi);
    drawBattery(display, 160);
    centerText(display, 178, 2, TFT_CYAN, "%.24s", latestStatus.place);
  } else {
    drawBattery(display, 160);
  }
  drawHint(display, "TAP:WAYPOINT HOLD:PAGE");
}

void drawWaypointPage(M5Canvas& display) {
  constexpr int kCount = sizeof(kCategories) / sizeof(kCategories[0]);
  const int previous = (categoryIndex + kCount - 1) % kCount;
  const int next = (categoryIndex + 1) % kCount;
  centerText(display, 42, 2, TFT_DARKGREY, "%s", kCategories[previous]);
  centerText(display, 64, 3, WHITE, "%s", kCategories[categoryIndex]);
  centerText(display, 94, 2, TFT_DARKGREY, "%s", kCategories[next]);
  centerText(display, 122, 2, GREEN, "TAP: SAVE HERE");
  if (haveStatus) {
    centerText(display, 148, 2, WHITE, "LAST %.20s", latestStatus.last_waypoint);
    centerText(display, 170, 2, TFT_CYAN, "%.24s", latestStatus.place);
  }
  drawHint(display, "TURN:TYPE HOLD:PAGE");
}

void drawSettingsPage(M5Canvas& display) {
  centerText(display, 46, 2, TFT_LIGHTGREY, "%u/5 %s", fieldIndex,
             kFieldNames[fieldIndex]);
  centerText(display, 76, 4, editing ? YELLOW : WHITE, "%s",
             fieldValue().c_str());
  centerText(display, 126, 2, WHITE,
             editing ? "TURN: CHANGE" : "TURN: SELECT");
  centerText(display, 148, 2, GREEN, editing ? "TAP: SAVE" : "TAP: EDIT");
  drawHint(display, "HOLD: NEXT PAGE");
}

const char* loggerName(uint8_t board) {
  switch (static_cast<telemetry::LoggerBoard>(board)) {
    case telemetry::LoggerBoard::Cardputer: return "CARDPUTER";
    case telemetry::LoggerBoard::CardputerAdv: return "CARDPUTER ADV";
    case telemetry::LoggerBoard::Core2: return "CORE2";
    default: return "LOGGER";
  }
}

const char* targetName() {
  return targetBoard == static_cast<uint8_t>(telemetry::LoggerBoard::Unknown)
             ? "ANY LOGGER"
             : loggerName(targetBoard);
}

const char* sourceName(uint8_t source) {
  switch (source) {
    case 1: return "ATOM GPS+IMU";
    case 3: return "GPS+ATOM IMU";
    default: return "OWN GPS";
  }
}

// What the Dial is talking to: the logger, the radio link and what the logger
// itself is linked to.
void drawLinkPage(M5Canvas& display, uint32_t nowMs) {
  if (!paired) {
    centerText(display, 56, 2, RED, "NO LOGGER");
    centerText(display, 82, 2, WHITE, "SCANNING CH %u", channel);
    centerText(display, 116, 2, TFT_LIGHTGREY, "PAIRS WITH");
    centerText(display, 136, 2, TFT_CYAN, "%s", targetName());
    drawHint(display, "TAP:CHANGE HOLD:PAGE");
    return;
  }
  const uint8_t board = haveStatus ? latestStatus.logger_board : 0;
  centerText(display, 44, 3, WHITE, "%s", loggerName(board));
  centerText(display, 74, 2, TFT_LIGHTGREY, "%02X:%02X:%02X:%02X:%02X:%02X",
             peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4],
             peerMac[5]);
  if (haveStatus) {
    const int rssi = latestRssi;
    centerText(display, 94, 2, rssi > -70 ? GREEN : rssi > -85 ? ORANGE : RED,
               "CH %u  %d dBm", channel, rssi);
    centerText(display, 118, 2, TFT_CYAN, "%s", sourceName(latestStatus.source));
    if (board != 0) {
      const uint8_t flags = latestStatus.link_flags;
      const bool atomUsed = latestStatus.source == 1 || latestStatus.source == 3;
      if (flags & telemetry::AtomLinked) {
        centerText(display, 138, 2, GREEN, "ATOMS3 LINKED");
      } else {
        centerText(display, 138, 2, atomUsed ? RED : TFT_DARKGREY,
                   atomUsed ? "ATOMS3 LOST" : "NO ATOMS3");
      }
      const bool sdOk = flags & telemetry::SdReady;
      centerText(display, 158, 2, sdOk ? WHITE : RED, "WIFI %s  SD %s",
                 (flags & telemetry::WifiOnline) ? "ON" : "OFF",
                 sdOk ? "OK" : "ERR");
    } else {
      centerText(display, 144, 1, TFT_DARKGREY, "UPDATE LOGGER FIRMWARE");
      centerText(display, 156, 1, TFT_DARKGREY, "FOR MORE DETAIL");
    }
    centerText(display, 180, 1, TFT_LIGHTGREY, "UPDATED %.1f s AGO",
               (nowMs - lastStatusMs) / 1000.0f);
  } else {
    centerText(display, 100, 2, ORANGE, "WAITING FOR STATUS");
  }
  const float probe = firstProbeCelsius();
  if (!std::isnan(probe)) {
    centerText(display, 190, 1, TFT_CYAN, "PROBE %.1f C", probe);
  }
  drawHint(display, "TAP:CHANGE HOLD:PAGE");
}

uint16_t surfaceColor(uint8_t index) {
  switch (index) {
    case 1: return GREEN;
    case 2: return YELLOW;
    case 3: return ORANGE;
    case 4: return RED;
    default: return TFT_DARKGREY;
  }
}

void drawRoadPage(M5Canvas& display) {
  centerText(display, 44, 2, TFT_LIGHTGREY, "ROAD");
  centerText(display, 64, 4, WHITE, "%s", kRoads[roadIndex]);
  centerText(display, 112, 2, TFT_LIGHTGREY, "SURFACE");
  centerText(display, 132, 4, surfaceColor(surfaceIndex), "%s",
             telemetry::kRoadSurfaces[surfaceIndex]);
  if (surfaceDirty) centerText(display, 176, 1, YELLOW, "SENDING...");
  drawHint(display, "TURN:SURFACE TAP:ROAD");
}

// Sends the surface once the knob has rested for a second.
void updateSurface(uint32_t nowMs) {
  if (!surfaceDirty || nowMs - surfaceTurnedMs < 1000 || pendingSequence != 0)
    return;
  if (!paired) return;  // Kept dirty until there is a logger to send to.
  sendCommand(telemetry::DialAction::SetSurface,
              telemetry::kRoadSurfaces[surfaceIndex]);
  surfaceDirty = false;
}

void drawVersionPage(M5Canvas& display, uint32_t nowMs) {
  dino::draw(display, kCenter - dino::kWidth, 40, 2,
             (nowMs / 4000) % dino::kFrameCount);
  centerText(display, 82, 2, TFT_CYAN, "SPECIES %s", version::kNumber);
  centerText(display, 104, 2, YELLOW, "DNA %s", version::kGit);
  centerText(display, 126, 2, GREEN, "%s", version::kDate);
  centerText(display, 156, 2, ORANGE, "%s", version::jokeAt(nowMs));
  drawHint(display, "HOLD: NEXT PAGE");
}

void drawAboutPage(M5Canvas& display, uint32_t nowMs) {
  // Inset from the round bezel so the dinosaur and the rocks stay visible.
  constexpr int kScale = 3;
  constexpr int kLeft = 24;
  constexpr int kWidth = 2 * kCenter - 2 * kLeft;
  constexpr int kGroundY = 50 + dino::kHeight * kScale + 1;
  const float speed =
      haveStatus && latestStatus.fix_valid ? latestStatus.speed_kmh : -1.0f;
  dinoGame.update(nowMs, speed, kWidth / kScale);
  if (dinoGame.takeNewBest()) preferences.putUShort(dino::kBestKey, dinoGame.best());
  dinoGame.draw(display, kLeft, kGroundY, kWidth, kScale, nowMs);
  switch (dinoGame.state()) {
    case dino::Game::State::Ready:
      centerText(display, 38, 1, YELLOW, "TAP OR TURN TO PLAY");
      break;
    case dino::Game::State::Playing:
    case dino::Game::State::Over: {
      char text[32];
      dinoGame.statusText(text, sizeof(text));
      centerText(display, 38, 1,
                 dinoGame.state() == dino::Game::State::Playing ? WHITE
                 : dinoGame.beatBest()                         ? GREEN
                                                               : RED,
                 "%s", text);
      break;
    }
    default:
      break;
  }
  centerText(display, 134, 2, WHITE, "%s", dino::kCredit1);
  centerText(display, 158, 3, TFT_CYAN, "%s", dino::kCredit2);
  drawHint(display, "HOLD: NEXT PAGE");
}

void draw() {
  if (!canvasReady) return;
  auto& display = canvas;
  display.fillScreen(BLACK);
  const bool linked = paired && haveStatus;
  drawFrame(display, linked);
  const char* title = page == DialPage::Waypoint ? "WAYPOINT"
                      : page == DialPage::Settings ? "SETTINGS"
                      : page == DialPage::Link ? "CONNECTION"
                      : page == DialPage::Road ? "ROAD"
                      : page == DialPage::Version ? "FOSSIL RECORD"
                      : page == DialPage::About ? "JP226PRINTS"
                      : linked ? "LINKED" : "SEARCHING";
  centerText(display, 20, 2, linked ? GREEN : ORANGE, "%s", title);
  switch (page) {
    case DialPage::Drive: drawDrivePage(display); break;
    case DialPage::Waypoint: drawWaypointPage(display); break;
    case DialPage::Settings: drawSettingsPage(display); break;
    case DialPage::Link: drawLinkPage(display, millis()); break;
    case DialPage::Road: drawRoadPage(display); break;
    case DialPage::Version: drawVersionPage(display, millis()); break;
    case DialPage::About: drawAboutPage(display, millis()); break;
    default: break;
  }
  display.pushSprite(&M5Dial.Display, 0, 0);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  M5Dial.begin(M5.config(), true, false);
  // Upside down (180 degrees) to suit the printed mount. Use 0 for a Dial
  // held the normal way up.
  M5Dial.Display.setRotation(2);
  M5Dial.Display.setBrightness(120);
  M5Dial.Display.fillScreen(BLACK);
  canvas.setColorDepth(8);
  canvasReady = canvas.createSprite(M5Dial.Display.width(),
                                    M5Dial.Display.height()) != nullptr;
  // Before ESP-NOW starts, so the first beacons are filtered correctly.
  preferences.begin("dial", false);
  targetBoard = preferences.getUChar(
      "target", static_cast<uint8_t>(telemetry::LoggerBoard::CardputerAdv));
  dinoGame.setBest(preferences.getUShort(dino::kBestKey, 0));
  // DS18B20 probes on Port B (G1/G2) or Port A (G13/G15).
  probes.begin({1, 2, 13, 15});
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
  probes.update(nowMs);
  sendTemperatures(nowMs);
  updateSurface(nowMs);
  handleControls();
  static uint32_t lastDrawMs = 0;
  // The dinosaur page animates; everything else redraws four times a second.
  if (nowMs - lastDrawMs >= (page == DialPage::About ? 60U : 250U)) {
    lastDrawMs = nowMs;
    draw();
  }
  delay(5);
}
