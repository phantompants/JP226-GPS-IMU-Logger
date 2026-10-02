// Atom Echo companion for the Cardputer ADV logger: spoken alerts, a waypoint
// button and a status light. It talks to the logger exactly like the M5Dial,
// sending DialCommand heartbeats and marks and listening to DialStatus.
//
// Logging starting or stopping is announced as "T-Rex says, rawr! Is
// logging" or "... Not logging".
//
// Logger choice: hold the main button and press the side reset button; it
// says "Pairs with ..." for the next logger in turn.
//
// Button: tap to save a waypoint, hold 1 s for a spoken status report, hold
// 4 s to mute or unmute the automatic alerts (button replies always speak).
// Light: blue blink = searching, amber = linked without a fix, green = fix,
// white = waiting for the logger to confirm a waypoint.

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <cstring>

#include "Clips.h"
#include "TelemetryProtocol.h"
#include "Version.h"

namespace {

// Which logger the Echo pairs with. Hold the main button while pressing the
// side reset button to step to the next one; the choice is spoken and saved.
constexpr telemetry::LoggerBoard kTargetBoards[] = {
    telemetry::LoggerBoard::CardputerAdv, telemetry::LoggerBoard::Cardputer,
    telemetry::LoggerBoard::Core2, telemetry::LoggerBoard::Unknown};
constexpr size_t kTargetBoardCount =
    sizeof(kTargetBoards) / sizeof(kTargetBoards[0]);
// Read by the receive callback; a single byte, so no lock is needed.
volatile uint8_t targetBoard =
    static_cast<uint8_t>(telemetry::LoggerBoard::CardputerAdv);
constexpr char kWaypointCategory[] = "MARK";
constexpr uint8_t kChannels = 13;
constexpr uint32_t kScanDwellMs = 450;
constexpr uint32_t kHeartbeatMs = 1000;
constexpr uint32_t kStatusStaleMs = 5000;
constexpr uint32_t kAckTimeoutMs = 3500;
// A change must hold this long before it is announced, so a brief dropout
// does not produce a "lost" and "regained" pair.
constexpr uint32_t kSettleMs = 3000;
constexpr uint32_t kStatusHoldMs = 1000;
constexpr uint32_t kMuteHoldMs = 4000;
constexpr uint8_t kVolume = 200;
// The base's amplifier runs at a higher gain and distorts at kVolume.
constexpr uint8_t kSpkBaseVolume = 140;

portMUX_TYPE receiveMux = portMUX_INITIALIZER_UNLOCKED;
telemetry::DialStatusPacket receivedStatus{};
telemetry::DialAckPacket receivedAck{};
uint8_t candidateMac[6]{};
uint8_t candidateChannel = 0;
bool candidateReady = false;
bool statusReady = false;
bool ackReady = false;
bool paired = false;
uint8_t peerMac[6]{};
uint8_t channel = 1;

telemetry::DialStatusPacket status{};
bool haveStatus = false;
uint32_t lastScanMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t nextSequence = 1;
telemetry::DialCommandPacket pendingPacket{};
uint32_t pendingSequence = 0;
uint32_t pendingSinceMs = 0;
uint32_t lastPendingSendMs = 0;
uint8_t pendingAttempts = 0;

Preferences preferences;
bool muted = false;
uint32_t pressStartMs = 0;
bool pressActive = false;
uint8_t holdStage = 0;

// A short queue so alerts that arrive together play one after another.
constexpr size_t kQueueSize = 6;
const uint8_t* queue[kQueueSize]{};
size_t queueLength[kQueueSize]{};
size_t queueHead = 0;
size_t queueCount = 0;

template <size_t N>
void say(const uint8_t (&clip)[N]) {
  if (queueCount == kQueueSize) return;
  const size_t tail = (queueHead + queueCount) % kQueueSize;
  queue[tail] = clip;
  queueLength[tail] = N;
  ++queueCount;
}

// Automatic alerts respect mute; replies to the button do not.
template <size_t N>
void alert(const uint8_t (&clip)[N]) {
  if (!muted) say(clip);
}

void updateSpeech() {
  if (queueCount == 0 || M5.Speaker.isPlaying()) return;
  M5.Speaker.playWav(queue[queueHead], queueLength[queueHead]);
  queueHead = (queueHead + 1) % kQueueSize;
  --queueCount;
}

void beep(float frequency, uint32_t durationMs) {
  M5.Speaker.tone(frequency, durationMs);
}

// A tracked on/off condition announced only after it has settled.
struct Watched {
  bool known = false;
  bool announced = false;
  bool current = false;
  uint32_t changedMs = 0;

  void reset() { *this = Watched{}; }

  // Returns true once when `value` has differed from the last announced value
  // for kSettleMs. The first value seen becomes the baseline silently.
  bool settle(bool value, uint32_t nowMs) {
    if (!known) {
      known = true;
      announced = current = value;
      return false;
    }
    if (value != current) {
      current = value;
      changedMs = nowMs;
    }
    if (current != announced && nowMs - changedMs >= kSettleMs) {
      announced = current;
      return true;
    }
    return false;
  }
};

Watched fixWatch;
Watched atomWatch;
Watched sdWatch;
Watched loggingWatch;
bool batteryLowSaid = false;
bool batteryCriticalSaid = false;

void resetWatches() {
  fixWatch.reset();
  atomWatch.reset();
  sdWatch.reset();
  loggingWatch.reset();
  batteryLowSaid = false;
  batteryCriticalSaid = false;
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
  } else if (samePeer && type == telemetry::PacketType::DialStatus &&
             length == sizeof(telemetry::DialStatusPacket)) {
    telemetry::DialStatusPacket packet{};
    std::memcpy(&packet, bytes, sizeof(packet));
    if (!telemetry::validatePacket(packet, type)) return;
    portENTER_CRITICAL(&receiveMux);
    receivedStatus = packet;
    statusReady = true;
    portEXIT_CRITICAL(&receiveMux);
  } else if (samePeer && type == telemetry::PacketType::DialAck &&
             length == sizeof(telemetry::DialAckPacket)) {
    telemetry::DialAckPacket packet{};
    std::memcpy(&packet, bytes, sizeof(packet));
    if (!telemetry::validatePacket(packet, type)) return;
    portENTER_CRITICAL(&receiveMux);
    receivedAck = packet;
    ackReady = true;
    portEXIT_CRITICAL(&receiveMux);
  }
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
  portEXIT_CRITICAL(&receiveMux);
  lastStatusMs = nowMs;
  lastHeartbeatMs = 0;
  Serial.printf("Paired with %02X:%02X:%02X:%02X:%02X:%02X on channel %u\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel);
}

void dropLink() {
  esp_now_del_peer(peerMac);
  portENTER_CRITICAL(&receiveMux);
  paired = false;
  portEXIT_CRITICAL(&receiveMux);
  haveStatus = false;
  pendingSequence = 0;
  resetWatches();
}

// DialStatusPacket::log_mode: 1 moving and 2/3 parked rows are logging;
// 0 waiting for a fix and 4 fix lost are not.
bool loggerIsLogging() {
  return status.log_mode >= 1 && status.log_mode <= 3;
}

template <bool Alert>
void sayLogging() {
  if (loggerIsLogging()) {
    if (Alert) alert(clips::trex_logging);
    else say(clips::trex_logging);
  } else {
    if (Alert) alert(clips::trex_not_logging);
    else say(clips::trex_not_logging);
  }
}

void announceChanges(bool first, uint32_t nowMs) {
  if (first) {
    alert(clips::logger_found);
    sayLogging<true>();
  }
  if (loggingWatch.settle(loggerIsLogging(), nowMs)) sayLogging<true>();
  if (fixWatch.settle(status.fix_valid != 0, nowMs)) {
    if (status.fix_valid) alert(clips::fix_ok);
    else alert(clips::fix_lost);
  }
  // Loggers that predate logger_board leave link_flags empty, so only trust
  // the flags from newer firmware.
  if (status.logger_board != 0) {
    const bool atomUsed = status.source == 1 || status.source == 3;
    if (atomUsed) {
      if (atomWatch.settle(status.link_flags & telemetry::AtomLinked, nowMs)) {
        if (status.link_flags & telemetry::AtomLinked) alert(clips::atom_ok);
        else alert(clips::atom_lost);
      }
    } else {
      atomWatch.reset();
    }
    if (sdWatch.settle(status.link_flags & telemetry::SdReady, nowMs)) {
      if (status.link_flags & telemetry::SdReady) alert(clips::sd_ok);
      else alert(clips::sd_error);
    }
  }
  const int battery = status.battery_percent;
  if (battery >= 0) {
    if (battery < 10 && !batteryCriticalSaid) {
      batteryCriticalSaid = batteryLowSaid = true;
      alert(clips::battery_critical);
    } else if (battery < 20 && !batteryLowSaid) {
      batteryLowSaid = true;
      alert(clips::battery_low);
    }
    // Re-arm with some margin so a reading hovering at the line stays quiet.
    if (battery > 15) batteryCriticalSaid = false;
    if (battery > 25) batteryLowSaid = false;
  }
}

void updateRadio(uint32_t nowMs) {
  if (paired && nowMs - lastStatusMs > kStatusStaleMs) {
    dropLink();
    alert(clips::logger_lost);
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

  telemetry::DialStatusPacket newStatus{};
  telemetry::DialAckPacket ack{};
  bool gotStatus = false;
  bool gotAck = false;
  portENTER_CRITICAL(&receiveMux);
  if (statusReady) {
    newStatus = receivedStatus;
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
    const bool first = !haveStatus;
    status = newStatus;
    haveStatus = true;
    lastStatusMs = nowMs;
    announceChanges(first, nowMs);
  }
  if (gotAck && pendingSequence != 0 &&
      ack.command_sequence == pendingSequence) {
    pendingSequence = 0;
    if (ack.accepted) {
      say(clips::saved);
      Serial.printf("Waypoint saved: %.24s\n", ack.waypoint_id);
    } else {
      say(clips::rejected);
    }
  }
  // Resend an unconfirmed mark; the logger repeats its ack for a duplicate.
  if (pendingSequence != 0 && nowMs - lastPendingSendMs >= 1000 &&
      pendingAttempts < 3) {
    esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&pendingPacket),
                 sizeof(pendingPacket));
    lastPendingSendMs = nowMs;
    ++pendingAttempts;
  }
  if (pendingSequence != 0 && nowMs - pendingSinceMs > kAckTimeoutMs) {
    pendingSequence = 0;
    say(clips::no_confirm);
  }
}

void sayPairing() {
  switch (static_cast<telemetry::LoggerBoard>(targetBoard)) {
    case telemetry::LoggerBoard::CardputerAdv: say(clips::pair_adv); break;
    case telemetry::LoggerBoard::Cardputer: say(clips::pair_cardputer); break;
    case telemetry::LoggerBoard::Core2: say(clips::pair_core2); break;
    default: say(clips::pair_any); break;
  }
}

void cycleTargetBoard() {
  size_t index = 0;
  while (index < kTargetBoardCount &&
         static_cast<uint8_t>(kTargetBoards[index]) != targetBoard) {
    ++index;
  }
  targetBoard =
      static_cast<uint8_t>(kTargetBoards[(index + 1) % kTargetBoardCount]);
  preferences.putUChar("target", targetBoard);
}

void speakStatus() {
  if (!paired || !haveStatus) {
    say(clips::no_logger);
    sayPairing();  // A reminder of which logger it is waiting for.
    return;
  }
  if (status.fix_valid) say(clips::fix_ok);
  else say(clips::no_fix);
  sayLogging<false>();
  if (status.logger_board != 0 && !(status.link_flags & telemetry::SdReady)) {
    say(clips::sd_error);
  }
  if (status.battery_percent >= 0 && status.battery_percent < 20) {
    if (status.battery_percent < 10) say(clips::battery_critical);
    else say(clips::battery_low);
  }
}

void markWaypoint() {
  if (pendingSequence != 0) return;  // Still waiting on the last one.
  if (!paired) {
    say(clips::no_logger);
    return;
  }
  beep(1500, 60);
  if (!sendCommand(telemetry::DialAction::MarkWaypoint, kWaypointCategory)) {
    say(clips::rejected);
  }
}

void toggleMute() {
  muted = !muted;
  preferences.putBool("muted", muted);
  if (muted) say(clips::muted);
  else say(clips::unmuted);
}

// Tap, 1 s hold and 4 s hold, with a tick at each threshold so the wearer
// can tell which one they will get on release.
void handleButton(uint32_t nowMs) {
  if (M5.BtnA.wasPressed()) {
    pressStartMs = nowMs;
    pressActive = true;
    holdStage = 0;
  }
  if (pressActive && M5.BtnA.isPressed()) {
    const uint32_t held = nowMs - pressStartMs;
    if (holdStage == 0 && held >= kStatusHoldMs) {
      holdStage = 1;
      beep(900, 40);
    } else if (holdStage == 1 && held >= kMuteHoldMs) {
      holdStage = 2;
      beep(600, 120);
    }
  }
  if (pressActive && M5.BtnA.wasReleased()) {
    pressActive = false;
    if (holdStage == 2) toggleMute();
    else if (holdStage == 1) speakStatus();
    else markWaypoint();
  }
}

void updateLed(uint32_t nowMs) {
  uint32_t color = 0;
  if (pendingSequence != 0) {
    color = 0xFFFFFF;
  } else if (!paired || !haveStatus) {
    color = (nowMs / 500) % 2 ? 0x0000FF : 0;
  } else {
    color = status.fix_valid ? 0x00FF00 : 0xFF8000;
  }
  // A short purple blink every 3 s is the reminder that alerts are muted.
  if (muted && nowMs % 3000 < 120) color = 0x8000FF;
  static uint32_t shown = 0xFFFFFFFFU;
  if (color == shown) return;
  shown = color;
  M5.Led.setAllColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
}

void printReport(uint32_t nowMs) {
  static uint32_t lastReportMs = 0;
  if (nowMs - lastReportMs < 5000) return;
  lastReportMs = nowMs;
  if (!paired) {
    Serial.printf("Searching for logger, channel %u%s\n", channel,
                  muted ? ", muted" : "");
    return;
  }
  Serial.printf("Linked ch %u, board %u, fix %u, sats %u, battery %d%%, "
                "flags 0x%02X%s\n",
                channel, haveStatus ? status.logger_board : 0,
                haveStatus ? status.fix_valid : 0,
                haveStatus ? status.satellites : 0,
                haveStatus ? status.battery_percent : -1,
                haveStatus ? status.link_flags : 0, muted ? ", muted" : "");
}

// The Atomic SPK base pulls its SD card's MOSI (G19) and SCLK (G23) high. On
// a bare Echo those pins are the amp's bit clock and the idle mic's data, and
// read low with pull-downs. Same check M5Unified uses for the base.
bool atomicSpkBasePresent() {
  pinMode(GPIO_NUM_19, INPUT_PULLDOWN);
  pinMode(GPIO_NUM_23, INPUT_PULLDOWN);
  delay(2);
  const bool present = digitalRead(GPIO_NUM_19) && digitalRead(GPIO_NUM_23);
  pinMode(GPIO_NUM_19, INPUT);
  pinMode(GPIO_NUM_23, INPUT);
  return present;
}

}  // namespace

void setup() {
  auto config = M5.config();
  config.internal_mic = false;  // The mic shares the speaker's I2S pins.
  // No sensors to probe, and the I2C scan would claim G21/G25, which drive
  // the Atomic SPK base's speaker.
  config.internal_imu = false;
  config.internal_rtc = false;
  config.external_imu = false;
  config.external_rtc = false;
  // Probe for the base before M5.begin() touches the pins.
  const bool spkBase = atomicSpkBasePresent();
  M5.begin(config);
  Serial.begin(115200);
  // M5Unified's board detection is unreliable here (it has reported this Echo
  // as an Atom Lite and an Atom U), so the speaker is always set up by hand.
  // M5.begin() still opens the internal I2C bus on G21/G25; free those pins
  // before the base's speaker takes them over.
  M5.In_I2C.release();
  auto speaker = M5.Speaker.config();
  if (spkBase) {
    // The base's own amplifier. The Echo's amp stays unclocked (G19/G33
    // idle), and the base's SD card is never used: its SPI pins are the
    // Echo's mic and speaker clocks.
    speaker.pin_bck = GPIO_NUM_22;
    speaker.pin_ws = GPIO_NUM_21;
    speaker.pin_data_out = GPIO_NUM_25;
    speaker.magnification = 16;
  } else {
    speaker.pin_bck = GPIO_NUM_19;
    speaker.pin_ws = GPIO_NUM_33;
    speaker.pin_data_out = GPIO_NUM_22;
    speaker.magnification = 12;
  }
  M5.Speaker.end();
  M5.Speaker.config(speaker);
  M5.Speaker.begin();
  Serial.printf("Board %d, speaker: %s\n", static_cast<int>(M5.getBoard()),
                spkBase ? "Atomic SPK base" : "Echo built-in");
  M5.Speaker.setVolume(spkBase ? kSpkBaseVolume : kVolume);
  M5.Led.setBrightness(40);
  preferences.begin("echo", false);
  muted = preferences.getBool("muted", false);
  targetBoard = preferences.getUChar(
      "target", static_cast<uint8_t>(telemetry::LoggerBoard::CardputerAdv));
  // Main button held through a reset: step to the next logger.
  M5.update();
  const bool changeTarget = M5.BtnA.isPressed();
  if (changeTarget) cycleTargetBoard();

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
  Serial.printf("Atom Echo station MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("Firmware %s %s %s, pairs with board %u\n", version::kNumber,
                version::kGit, version::kDate, targetBoard);
  if (changeTarget) sayPairing();
  else say(clips::ready);
}

void loop() {
  M5.update();
  const uint32_t nowMs = millis();
  updateRadio(nowMs);
  handleButton(nowMs);
  updateSpeech();
  updateLed(nowMs);
  printReport(nowMs);
  delay(5);
}
