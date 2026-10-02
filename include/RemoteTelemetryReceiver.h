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
  // Controllers (Dial, Atom Echo) share one command path. The ack for a
  // popped command goes back to the controller that sent it.
  bool popDialCommand(telemetry::DialCommandPacket& packet);
  void sendDialStatus(const telemetry::DialStatusPacket& packet,
                      uint32_t nowMs);
  void sendDialAck(const telemetry::DialAckPacket& packet);
  bool dialConnected(uint32_t nowMs) const;
  // Once a second to every controller, for the black box's backup log.
  void sendPositionReport(const telemetry::PositionReportPacket& packet,
                          uint32_t nowMs);
  // Latest report from a black box; false if none has been heard.
  bool blackBoxStatus(telemetry::BlackBoxStatusPacket& packet,
                      uint32_t& ageMs, uint32_t nowMs) const;
  // Latest DS18B20 report from one accessory (Dial, Echo or black box).
  bool temperatureReport(telemetry::ProbeSource source,
                         telemetry::TemperatureReportPacket& packet,
                         uint32_t& ageMs, uint32_t nowMs) const;
  uint8_t controllerCount(uint32_t nowMs) const;
  // Clock, UTC offset, battery and board type copied into each beacon.
  void setBeaconInfo(uint32_t utcEpochS, int16_t utcOffsetMin,
                     uint8_t timeFlags, int8_t batteryPercent,
                     uint16_t batteryMv, telemetry::LoggerBoard board);
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
  bool controllerFresh(size_t slot, uint32_t nowMs) const;

  // M5Dial, Atom Echo and black box together, with one spare.
  static constexpr size_t kMaxControllers = 4;
  static constexpr uint32_t kControllerStaleMs = 5'000;

  struct Controller {
    uint8_t mac[6]{};
    uint8_t addedMac[6]{};
    bool active = false;
    bool peerAdded = false;
    bool commandReady = false;
    bool haveCommandSequence = false;
    bool haveAck = false;
    bool ackRepeatPending = false;
    uint32_t lastMs = 0;
    uint32_t lastCommandSequence = 0;
    telemetry::DialCommandPacket command{};
    telemetry::DialAckPacket lastAck{};
  };

  static RemoteTelemetryReceiver* instance_;
  static constexpr size_t kRawQueueSize = 4;

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  telemetry::TelemetryPacket latest_{};
  telemetry::RawImuBatchPacket rawQueue_[kRawQueueSize]{};
  Controller controllers_[kMaxControllers]{};
  uint8_t allowedMac_[6]{};
  uint8_t peerMac_[6]{};
  uint8_t rawHead_ = 0;
  uint8_t rawTail_ = 0;
  uint8_t rawCount_ = 0;
  bool havePacket_ = false;
  bool havePeer_ = false;
  size_t ackSlot_ = 0;
  size_t nextCommandSlot_ = 0;
  bool ready_ = false;
  uint32_t lastPacketMs_ = 0;
  uint32_t lastDiscoveryMs_ = 0;
  uint32_t lastDialStatusMs_ = 0;
  uint32_t lastPositionReportMs_ = 0;
  telemetry::BlackBoxStatusPacket blackBox_{};
  bool haveBlackBox_ = false;
  uint32_t lastBlackBoxMs_ = 0;
  // Indexed by ProbeSource (Dial 1, Echo 2, BlackBox 3).
  static constexpr size_t kProbeSources = 4;
  telemetry::TemperatureReportPacket temperatures_[kProbeSources]{};
  uint32_t temperatureMs_[kProbeSources]{};
  bool haveTemperature_[kProbeSources]{};
  uint32_t discoverySequence_ = 0;
  uint32_t beaconUtcEpochS_ = 0;
  uint32_t beaconUtcSetMs_ = 0;
  int16_t beaconUtcOffsetMin_ = 0;
  uint8_t beaconTimeFlags_ = 0;
  int8_t beaconBatteryPercent_ = telemetry::kBatteryUnknown;
  uint16_t beaconBatteryMv_ = 0;
  uint8_t beaconBoard_ = 0;
  uint32_t packetsLost_ = 0;
  uint32_t duplicates_ = 0;
  uint32_t crcErrors_ = 0;
  uint32_t versionErrors_ = 0;
};
