#include "RemoteTelemetryReceiver.h"

#include <WiFi.h>
#include <esp_wifi.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "Config.h"

RemoteTelemetryReceiver* RemoteTelemetryReceiver::instance_ = nullptr;

namespace {

constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF};

uint32_t saturatedAdd(uint32_t left, uint32_t right) {
  return UINT32_MAX - left < right ? UINT32_MAX : left + right;
}

}  // namespace

bool RemoteTelemetryReceiver::begin(const uint8_t allowedMac[6]) {
  std::memcpy(allowedMac_, allowedMac, sizeof(allowedMac_));
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    ready_ = false;
    return false;
  }

  instance_ = this;
  if (esp_now_register_recv_cb(receiveCallback) != ESP_OK) {
    esp_now_deinit();
    instance_ = nullptr;
    ready_ = false;
    return false;
  }

  esp_now_peer_info_t broadcast{};
  std::memcpy(broadcast.peer_addr, kBroadcastMac, sizeof(kBroadcastMac));
  broadcast.channel = 0;
  broadcast.ifidx = WIFI_IF_STA;
  broadcast.encrypt = false;
  const esp_err_t peerResult = esp_now_add_peer(&broadcast);
  ready_ = peerResult == ESP_OK || peerResult == ESP_ERR_ESPNOW_EXIST;
  return ready_;
}

void RemoteTelemetryReceiver::receiveCallback(const esp_now_recv_info_t* info,
                                              const uint8_t* data, int length) {
  if (instance_ != nullptr) instance_->receive(info, data, length);
}

bool RemoteTelemetryReceiver::sourceAllowed(const uint8_t* mac) const {
  if (!telemetry::macIsUnset(allowedMac_)) {
    return std::memcmp(mac, allowedMac_, sizeof(allowedMac_)) == 0;
  }
  // Discovery mode locks to the first node heard, but accepts a replacement
  // once that node has gone stale.
  const uint32_t nowMs = millis();
  portENTER_CRITICAL(&mux_);
  const bool allowed =
      !havePeer_ || std::memcmp(mac, peerMac_, sizeof(peerMac_)) == 0 ||
      nowMs - lastPacketMs_ > config::kRemoteStaleMs;
  portEXIT_CRITICAL(&mux_);
  return allowed;
}

void RemoteTelemetryReceiver::receive(const esp_now_recv_info_t* info,
                                      const uint8_t* data, int length) {
  if (info == nullptr || info->src_addr == nullptr || data == nullptr ||
      length < static_cast<int>(sizeof(telemetry::PacketHeader))) {
    return;
  }

  telemetry::PacketHeader header{};
  std::memcpy(&header, data, sizeof(header));
  if (header.magic != telemetry::kMagic ||
      header.version != telemetry::kProtocolVersion) {
    portENTER_CRITICAL(&mux_);
    ++versionErrors_;
    portEXIT_CRITICAL(&mux_);
    return;
  }
  if (header.size != length) return;

  const auto type = static_cast<telemetry::PacketType>(header.type);
  if (type == telemetry::PacketType::DialCommand &&
      length == static_cast<int>(sizeof(telemetry::DialCommandPacket))) {
    telemetry::DialCommandPacket packet{};
    std::memcpy(&packet, data, sizeof(packet));
    if (!telemetry::validatePacket(packet, telemetry::PacketType::DialCommand))
      return;
    if (!telemetry::macIsUnset(config::kDialEspNowMac) &&
        std::memcmp(info->src_addr, config::kDialEspNowMac, 6) != 0) return;
    const uint32_t nowMs = millis();
    portENTER_CRITICAL(&mux_);
    // Reuse this controller's slot, else a free one, else one gone stale.
    Controller* slot = nullptr;
    for (auto& controller : controllers_) {
      if (controller.active &&
          std::memcmp(controller.mac, info->src_addr, 6) == 0) {
        slot = &controller;
        break;
      }
    }
    for (size_t i = 0; slot == nullptr && i < kMaxControllers; ++i) {
      if (!controllerFresh(i, nowMs)) {
        slot = &controllers_[i];
        std::memcpy(slot->mac, info->src_addr, 6);
        slot->active = true;
        slot->commandReady = false;
        slot->haveCommandSequence = false;
        slot->haveAck = false;
        slot->ackRepeatPending = false;
      }
    }
    if (slot != nullptr) {
      slot->lastMs = nowMs;
      if (packet.action !=
          static_cast<uint8_t>(telemetry::DialAction::Heartbeat)) {
        if (slot->haveCommandSequence &&
            packet.sequence == slot->lastCommandSequence) {
          slot->ackRepeatPending = slot->haveAck;
        } else {
          slot->lastCommandSequence = packet.sequence;
          slot->haveCommandSequence = true;
          slot->command = packet;
          slot->commandReady = true;
        }
      }
    }
    portEXIT_CRITICAL(&mux_);
    return;
  }
  if (!sourceAllowed(info->src_addr)) return;
  if (type == telemetry::PacketType::Telemetry &&
      length == static_cast<int>(sizeof(telemetry::TelemetryPacket))) {
    telemetry::TelemetryPacket packet{};
    std::memcpy(&packet, data, sizeof(packet));
    if (!telemetry::validatePacket(packet, telemetry::PacketType::Telemetry)) {
      portENTER_CRITICAL(&mux_);
      ++crcErrors_;
      portEXIT_CRITICAL(&mux_);
      return;
    }
    acceptTelemetry(packet, info->src_addr, millis());
    return;
  }

  if (type == telemetry::PacketType::RawImuBatch &&
      length == static_cast<int>(sizeof(telemetry::RawImuBatchPacket))) {
    telemetry::RawImuBatchPacket packet{};
    std::memcpy(&packet, data, sizeof(packet));
    if (!telemetry::validatePacket(packet,
                                   telemetry::PacketType::RawImuBatch)) {
      portENTER_CRITICAL(&mux_);
      ++crcErrors_;
      portEXIT_CRITICAL(&mux_);
      return;
    }
    portENTER_CRITICAL(&mux_);
    if (rawCount_ == kRawQueueSize) {
      rawTail_ = (rawTail_ + 1U) % kRawQueueSize;
      --rawCount_;
    }
    rawQueue_[rawHead_] = packet;
    rawHead_ = (rawHead_ + 1U) % kRawQueueSize;
    ++rawCount_;
    portEXIT_CRITICAL(&mux_);
  }
}

void RemoteTelemetryReceiver::acceptTelemetry(
    const telemetry::TelemetryPacket& packet, const uint8_t* mac,
    uint32_t nowMs) {
  portENTER_CRITICAL(&mux_);
  // A restarted node begins again at sequence 0 with a smaller uptime, and a
  // replacement node has a different MAC. Either case, or a return after the
  // link went stale, starts a new stream rather than looking like old packets.
  const bool newStream =
      !havePacket_ || std::memcmp(mac, peerMac_, sizeof(peerMac_)) != 0 ||
      packet.uptime_ms < latest_.uptime_ms ||
      nowMs - lastPacketMs_ > config::kRemoteStaleMs;
  if (!newStream) {
    const int32_t delta = static_cast<int32_t>(packet.sequence - latest_.sequence);
    if (delta == 0) {
      ++duplicates_;
      portEXIT_CRITICAL(&mux_);
      return;
    }
    if (delta < 0) {
      portEXIT_CRITICAL(&mux_);
      return;
    }
    if (delta > 1) packetsLost_ += static_cast<uint32_t>(delta - 1);
  }
  latest_ = packet;
  lastPacketMs_ = nowMs;
  havePacket_ = true;
  havePeer_ = true;
  std::memcpy(peerMac_, mac, sizeof(peerMac_));
  portEXIT_CRITICAL(&mux_);
}

void RemoteTelemetryReceiver::sendDiscovery(uint32_t nowMs) {
  if (!ready_ || nowMs - lastDiscoveryMs_ < config::kRemoteDiscoveryIntervalMs) {
    return;
  }
  lastDiscoveryMs_ = nowMs;
  telemetry::DiscoveryPacket packet{};
  telemetry::preparePacket(packet, telemetry::PacketType::Discovery);
  packet.sequence = discoverySequence_++;
  packet.uptime_ms = nowMs;
  packet.time_flags = static_cast<uint8_t>(
      (beaconTimeFlags_ & 0x0FU) |
      (beaconBoard_ << telemetry::kTimeFlagBoardShift));
  if (beaconTimeFlags_ & telemetry::UtcValid) {
    packet.utc_epoch_s =
        beaconUtcEpochS_ + (nowMs - beaconUtcSetMs_) / 1000U;
  }
  packet.utc_offset_min = beaconUtcOffsetMin_;
  packet.battery_percent = beaconBatteryPercent_;
  packet.battery_mv = beaconBatteryMv_;
  telemetry::sealPacket(packet);
  esp_now_send(kBroadcastMac, reinterpret_cast<const uint8_t*>(&packet),
               sizeof(packet));
}

void RemoteTelemetryReceiver::setBeaconInfo(uint32_t utcEpochS,
                                            int16_t utcOffsetMin,
                                            uint8_t timeFlags,
                                            int8_t batteryPercent,
                                            uint16_t batteryMv,
                                            telemetry::LoggerBoard board) {
  beaconUtcEpochS_ = utcEpochS;
  beaconUtcSetMs_ = millis();
  beaconUtcOffsetMin_ = utcOffsetMin;
  beaconTimeFlags_ = timeFlags;
  beaconBatteryPercent_ = batteryPercent;
  beaconBatteryMv_ = batteryMv;
  beaconBoard_ = static_cast<uint8_t>(board) & 0x0FU;
}

bool RemoteTelemetryReceiver::controllerFresh(size_t slot,
                                              uint32_t nowMs) const {
  const Controller& controller = controllers_[slot];
  return controller.active &&
         telemetry::elapsedMs(nowMs, controller.lastMs) < kControllerStaleMs;
}

void RemoteTelemetryReceiver::update(uint32_t nowMs) {
  sendDiscovery(nowMs);
  for (auto& controller : controllers_) {
    uint8_t mac[6]{};
    uint8_t oldMac[6]{};
    bool addPeer = false;
    bool removeOld = false;
    telemetry::DialAckPacket repeat{};
    bool repeatReady = false;
    portENTER_CRITICAL(&mux_);
    if (controller.active &&
        (!controller.peerAdded ||
         std::memcmp(controller.addedMac, controller.mac, 6) != 0)) {
      std::memcpy(mac, controller.mac, 6);
      std::memcpy(oldMac, controller.addedMac, 6);
      removeOld = controller.peerAdded;
      addPeer = true;
    }
    if (controller.ackRepeatPending && controller.haveAck &&
        controller.peerAdded) {
      repeat = controller.lastAck;
      std::memcpy(mac, controller.mac, 6);
      controller.ackRepeatPending = false;
      repeatReady = true;
    }
    portEXIT_CRITICAL(&mux_);
    if (addPeer) {
      // A slot taken over by a new controller drops the old one's peer entry,
      // unless the other slot still talks to that address.
      if (removeOld) {
        bool shared = false;
        for (const auto& other : controllers_) {
          if (&other != &controller && other.peerAdded &&
              std::memcmp(other.addedMac, oldMac, 6) == 0) shared = true;
        }
        if (!shared) esp_now_del_peer(oldMac);
      }
      esp_now_peer_info_t peer{};
      std::memcpy(peer.peer_addr, mac, 6);
      peer.channel = 0;
      peer.ifidx = WIFI_IF_STA;
      const esp_err_t result = esp_now_add_peer(&peer);
      portENTER_CRITICAL(&mux_);
      if (result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST) {
        std::memcpy(controller.addedMac, mac, 6);
        controller.peerAdded = true;
      } else {
        controller.peerAdded = false;
      }
      portEXIT_CRITICAL(&mux_);
    }
    if (repeatReady) {
      esp_now_send(mac, reinterpret_cast<const uint8_t*>(&repeat),
                   sizeof(repeat));
    }
  }
}

bool RemoteTelemetryReceiver::popDialCommand(
    telemetry::DialCommandPacket& packet) {
  bool ready = false;
  portENTER_CRITICAL(&mux_);
  // Take turns so one busy controller cannot starve the other.
  for (size_t i = 0; i < kMaxControllers && !ready; ++i) {
    const size_t slot = (nextCommandSlot_ + i) % kMaxControllers;
    if (controllers_[slot].commandReady) {
      packet = controllers_[slot].command;
      controllers_[slot].commandReady = false;
      ackSlot_ = slot;
      nextCommandSlot_ = (slot + 1) % kMaxControllers;
      ready = true;
    }
  }
  portEXIT_CRITICAL(&mux_);
  return ready;
}

bool RemoteTelemetryReceiver::dialConnected(uint32_t nowMs) const {
  return controllerCount(nowMs) > 0;
}

uint8_t RemoteTelemetryReceiver::controllerCount(uint32_t nowMs) const {
  uint8_t count = 0;
  portENTER_CRITICAL(&mux_);
  for (size_t i = 0; i < kMaxControllers; ++i) {
    if (controllerFresh(i, nowMs)) ++count;
  }
  portEXIT_CRITICAL(&mux_);
  return count;
}

void RemoteTelemetryReceiver::sendDialStatus(
    const telemetry::DialStatusPacket& packet, uint32_t nowMs) {
  if (!ready_ || nowMs - lastDialStatusMs_ < 500) return;
  lastDialStatusMs_ = nowMs;
  for (size_t i = 0; i < kMaxControllers; ++i) {
    uint8_t mac[6]{};
    portENTER_CRITICAL(&mux_);
    const bool send = controllerFresh(i, nowMs) && controllers_[i].peerAdded;
    std::memcpy(mac, controllers_[i].addedMac, 6);
    portEXIT_CRITICAL(&mux_);
    if (send) {
      esp_now_send(mac, reinterpret_cast<const uint8_t*>(&packet),
                   sizeof(packet));
    }
  }
}

void RemoteTelemetryReceiver::sendDialAck(
    const telemetry::DialAckPacket& packet) {
  if (!ready_) return;
  uint8_t mac[6]{};
  portENTER_CRITICAL(&mux_);
  Controller& controller = controllers_[ackSlot_];
  controller.lastAck = packet;
  controller.haveAck = true;
  const bool send = controller.peerAdded;
  std::memcpy(mac, controller.addedMac, 6);
  portEXIT_CRITICAL(&mux_);
  if (send) {
    esp_now_send(mac, reinterpret_cast<const uint8_t*>(&packet),
                 sizeof(packet));
  }
}

NormalizedTelemetry RemoteTelemetryReceiver::snapshot(uint32_t nowMs) const {
  telemetry::TelemetryPacket packet{};
  bool havePacket = false;
  uint32_t lastPacketMs = 0;
  uint32_t lost = 0;
  uint32_t duplicates = 0;
  uint32_t crcErrors = 0;
  uint32_t versionErrors = 0;
  portENTER_CRITICAL(&mux_);
  packet = latest_;
  havePacket = havePacket_;
  lastPacketMs = lastPacketMs_;
  lost = packetsLost_;
  duplicates = duplicates_;
  crcErrors = crcErrors_;
  versionErrors = versionErrors_;
  portEXIT_CRITICAL(&mux_);

  NormalizedTelemetry result;
  result.metadata.source = TelemetrySource::AtomS3Remote;
  result.metadata.packetsLost = lost;
  result.metadata.duplicates = duplicates;
  result.metadata.crcErrors = crcErrors;
  result.metadata.versionErrors = versionErrors;
  if (!havePacket) return result;

  const uint32_t packetAge = telemetry::elapsedMs(nowMs, lastPacketMs);
  const bool fresh = packetAge <= config::kRemoteStaleMs;
  result.metadata.remoteConnected = fresh;
  result.metadata.remoteSequence = packet.sequence;
  result.metadata.packetAgeMs = packetAge;
  result.metadata.remoteTransmitFailures = packet.transmit_failures;

  const uint16_t flags = packet.status_flags;
  result.gps.positionFresh = fresh && (flags & telemetry::GpsPositionValid);
  result.gps.altitudeFresh = fresh && (flags & telemetry::GpsAltitudeValid);
  result.gps.speedFresh = fresh && (flags & telemetry::GpsSpeedValid);
  result.gps.courseFresh = fresh && (flags & telemetry::GpsCourseValid);
  result.gps.satellitesValid = fresh && (flags & telemetry::GpsSatellitesValid);
  result.gps.hdopValid = fresh && (flags & telemetry::GpsHdopValid);
  result.gps.utcValid = fresh && (flags & telemetry::GpsUtcValid);
  result.gps.fixValid = fresh && (flags & telemetry::GpsFixValid);
  result.gps.fixAgeMs =
      saturatedAdd(static_cast<uint32_t>(packet.gps_age_ms), packetAge);
  result.gps.utcEpochMs = packet.gps_utc_ms;
  result.gps.latitude = packet.latitude_deg;
  result.gps.longitude = packet.longitude_deg;
  result.gps.altitudeM = packet.altitude_m;
  result.gps.speedKmh = packet.speed_kmh;
  result.gps.courseDeg = packet.course_deg;
  result.gps.satellites = packet.satellites;
  result.gps.hdop = packet.hdop;

  result.imu.available = fresh && (flags & telemetry::ImuValid);
  result.imu.valid = result.imu.available;
  result.imu.calibrated = flags & telemetry::ImuCalibrated;
  result.imu.axMps2 = packet.accel_x_mps2;
  result.imu.ayMps2 = packet.accel_y_mps2;
  result.imu.azMps2 = packet.accel_z_mps2;
  result.imu.gxDps = packet.gyro_x_dps;
  result.imu.gyDps = packet.gyro_y_dps;
  result.imu.gzDps = packet.gyro_z_dps;
  result.imu.pitchDeg = packet.pitch_deg;
  result.imu.rollDeg = packet.roll_deg;
  result.imu.gTotalMps2 =
      std::sqrt(result.imu.axMps2 * result.imu.axMps2 +
                result.imu.ayMps2 * result.imu.ayMps2 +
                result.imu.azMps2 * result.imu.azMps2);
  if (flags & telemetry::ImuStatisticsValid) {
    result.imu.accelRmsMps2 = packet.accel_rms_mps2;
    result.imu.verticalAccelRmsMps2 = packet.vertical_accel_rms_mps2;
    result.imu.verticalAccelPeakPosMps2 =
        packet.vertical_accel_peak_pos_mps2;
    result.imu.verticalAccelPeakNegMps2 =
        packet.vertical_accel_peak_neg_mps2;
    result.imu.lateralAccelPeakMps2 = packet.lateral_accel_peak_mps2;
    result.imu.longitudinalAccelPeakMps2 =
        packet.longitudinal_accel_peak_mps2;
    result.imu.vibrationRmsMps2 = packet.vibration_rms_mps2;
    result.imu.sampleCount = packet.imu_sample_count;
  }
  return result;
}

bool RemoteTelemetryReceiver::popRawImuBatch(
    telemetry::RawImuBatchPacket& packet) {
  portENTER_CRITICAL(&mux_);
  if (rawCount_ == 0) {
    portEXIT_CRITICAL(&mux_);
    return false;
  }
  packet = rawQueue_[rawTail_];
  rawTail_ = (rawTail_ + 1U) % kRawQueueSize;
  --rawCount_;
  portEXIT_CRITICAL(&mux_);
  return true;
}

const char* RemoteTelemetryReceiver::statusText(uint32_t nowMs) const {
  if (!ready_) return "INIT FAILED";
  portENTER_CRITICAL(&mux_);
  const bool havePacket = havePacket_;
  const uint32_t lastPacketMs = lastPacketMs_;
  portEXIT_CRITICAL(&mux_);
  if (!havePacket) return "SEARCHING";
  return telemetry::elapsedMs(nowMs, lastPacketMs) <= config::kRemoteStaleMs
             ? "CONNECTED"
             : "LOST";
}

String RemoteTelemetryReceiver::peerMacText() const {
  uint8_t mac[6]{};
  bool havePeer = false;
  portENTER_CRITICAL(&mux_);
  havePeer = havePeer_;
  std::memcpy(mac, peerMac_, sizeof(mac));
  portEXIT_CRITICAL(&mux_);
  if (!havePeer) return "--:--:--:--:--:--";
  char text[18]{};
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}
