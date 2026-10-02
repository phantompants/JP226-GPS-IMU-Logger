#pragma once

// DS18B20 waterproof temperature probes on a Grove port, shared by every
// device. Wire red to 5 V, black to GND and yellow to a Grove signal pin, with
// a 4.7 kOhm pull-up from yellow to 3.3 V (never to 5 V: the ESP32 pins are
// 3.3 V only). Either Grove signal pin works: both are searched, and the
// search repeats every minute so a probe can be plugged in later. Up to
// kMaxProbes can share one port on the same 1-Wire bus.
//
// Needs lib_deps: paulstoffregen/OneWire and milesburton/DallasTemperature.

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <cmath>
#include <initializer_list>

#include "TelemetryProtocol.h"

class TempProbes {
 public:
  static constexpr int kMaxProbes = 4;

  struct Reading {
    uint16_t id = 0;  // Last two bytes of the probe's ROM, to tell them apart
    float celsius = NAN;
  };

  // Candidate pins (up to four), in the order they are tried. With rescan
  // false the pins are only searched once, so they can be handed to
  // something else (the Cardputer's Grove GPS) when no probe is found.
  void begin(std::initializer_list<int> pins, bool rescan = true) {
    rescan_ = rescan;
    pinCount_ = 0;
    for (int pin : pins) {
      if (pinCount_ < kMaxPins && pin >= 0) pins_[pinCount_++] = pin;
    }
    search();
  }

  void update(uint32_t nowMs) {
    if (count_ == 0) {
      if (rescan_ && nowMs - lastSearchMs_ >= kSearchIntervalMs) search();
      return;
    }
    // Non-blocking: start a conversion, collect it ~800 ms later.
    if (!converting_ && nowMs - lastReadMs_ >= kReadIntervalMs) {
      sensors_.requestTemperatures();
      converting_ = true;
      conversionStartMs_ = nowMs;
    } else if (converting_ && nowMs - conversionStartMs_ >= kConversionMs) {
      converting_ = false;
      lastReadMs_ = nowMs;
      int valid = 0;
      for (int i = 0; i < count_; ++i) {
        const float c = sensors_.getTempC(addresses_[i]);
        // -127 means the probe stopped answering; 85 is the power-on value.
        readings_[i].celsius =
            (c == DEVICE_DISCONNECTED_C || c == 85.0f) ? NAN : c;
        if (!std::isnan(readings_[i].celsius)) ++valid;
      }
      if (valid == 0 && nowMs - lastSearchMs_ >= kSearchIntervalMs) search();
    }
  }

  int count() const { return count_; }
  int pin() const { return activePin_; }
  const Reading& reading(int index) const { return readings_[index]; }
  // True when at least one probe gave a reading.
  bool any() const {
    for (int i = 0; i < count_; ++i) {
      if (!std::isnan(readings_[i].celsius)) return true;
    }
    return false;
  }

 private:
  static constexpr uint32_t kReadIntervalMs = 10'000;
  static constexpr uint32_t kConversionMs = 800;
  static constexpr uint32_t kSearchIntervalMs = 60'000;

  void search() {
    lastSearchMs_ = millis();
    count_ = 0;
    activePin_ = -1;
    for (int p = 0; p < pinCount_; ++p) {
      const int pin = pins_[p];
      wire_.begin(pin);
      sensors_.setOneWire(&wire_);
      sensors_.begin();
      const int found = sensors_.getDeviceCount();
      if (found == 0) continue;
      sensors_.setWaitForConversion(false);
      sensors_.setResolution(11);  // 0.125 C steps, under 400 ms
      for (int i = 0; i < found && count_ < kMaxProbes; ++i) {
        if (!sensors_.getAddress(addresses_[count_], i)) continue;
        readings_[count_].id = static_cast<uint16_t>(
            (addresses_[count_][6] << 8) | addresses_[count_][5]);
        readings_[count_].celsius = NAN;
        ++count_;
      }
      if (count_ > 0) {
        activePin_ = pin;
        lastReadMs_ = 0;
        converting_ = false;
        return;
      }
    }
  }

  static constexpr int kMaxPins = 4;
  int pins_[kMaxPins]{};
  int pinCount_ = 0;
  bool rescan_ = true;
  int activePin_ = -1;
  OneWire wire_;
  DallasTemperature sensors_;
  DeviceAddress addresses_[kMaxProbes]{};
  Reading readings_[kMaxProbes]{};
  int count_ = 0;
  bool converting_ = false;
  uint32_t conversionStartMs_ = 0;
  uint32_t lastReadMs_ = 0;
  uint32_t lastSearchMs_ = 0;
};

// Fills a report for the logger from the probes' latest readings. Returns
// false when there is nothing to send.
inline bool makeTemperatureReport(const TempProbes& probes,
                                  telemetry::ProbeSource source,
                                  std::uint32_t sequence,
                                  telemetry::TemperatureReportPacket& packet) {
  if (probes.count() == 0) return false;
  telemetry::preparePacket(packet, telemetry::PacketType::TemperatureReport);
  packet.sequence = sequence;
  packet.source = static_cast<std::uint8_t>(source);
  packet.count = static_cast<std::uint8_t>(probes.count());
  for (int i = 0; i < probes.count(); ++i) {
    packet.probe_id[i] = probes.reading(i).id;
    packet.celsius[i] = probes.reading(i).celsius;
  }
  telemetry::sealPacket(packet);
  return true;
}
