#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "TelemetryData.h"
#include "TelemetryProtocol.h"

class RemoteTelemetryReceiver {
 public:
  bool begin(const uint8_t allowedMac[6]);
  void update(uint32_t nowMs);
  NormalizedTelemetry snapshot(uint32_t nowMs) const;
  bool popRawImuBatch(telemetry::RawImuBatchPacket& packet);
  bool popDialCommand(telemetry::DialCommandPacket& packet);
  void sendDialStatus(const telemetry::DialStatusPacket& packet,
                      uint32_t nowMs);
  void sendDialAck(const telemetry::DialAckPacket& packet);
  bool dialConnected(uint32_t nowMs) const;
  bool ready() const { return ready_; }
  const char* statusText(uint32_t nowMs) const;
  String peerMacText() const;

 private:
  static void receiveCallback(const esp_now_recv_info_t* info,
                              const uint8_t* data, int length);
  void receive(const esp_now_recv_info_t* info, const uint8_t* data,
               int length);
  bool sourceAllowed(const uint8_t* mac) const;
  void acceptTelemetry(const telemetry::TelemetryPacket& packet,
                       const uint8_t* mac, uint32_t nowMs);
  void sendDiscovery(uint32_t nowMs);

  static RemoteTelemetryReceiver* instance_;
  static constexpr size_t kRawQueueSize = 4;

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  telemetry::TelemetryPacket latest_{};
  telemetry::RawImuBatchPacket rawQueue_[kRawQueueSize]{};
  telemetry::DialCommandPacket dialCommand_{};
  telemetry::DialAckPacket lastDialAck_{};
  uint8_t allowedMac_[6]{};
  uint8_t peerMac_[6]{};
  uint8_t dialMac_[6]{};
  uint8_t rawHead_ = 0;
  uint8_t rawTail_ = 0;
  uint8_t rawCount_ = 0;
  bool havePacket_ = false;
  bool havePeer_ = false;
  bool haveDial_ = false;
  bool dialCommandReady_ = false;
  bool dialPeerAdded_ = false;
  bool haveDialCommandSequence_ = false;
  bool haveDialAck_ = false;
  bool dialAckRepeatPending_ = false;
  bool ready_ = false;
  uint32_t lastPacketMs_ = 0;
  uint32_t lastDiscoveryMs_ = 0;
  uint32_t lastDialMs_ = 0;
  uint32_t lastDialStatusMs_ = 0;
  uint32_t lastDialCommandSequence_ = 0;
  uint32_t discoverySequence_ = 0;
  uint32_t packetsLost_ = 0;
  uint32_t duplicates_ = 0;
  uint32_t crcErrors_ = 0;
  uint32_t versionErrors_ = 0;
};
