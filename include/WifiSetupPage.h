#pragma once

#include <Arduino.h>
#include <M5GFX.h>

class LocationTime;

class WifiSetupPage {
 public:
  void open();
  void handleInput(LocationTime& locationTime);
  void update(uint32_t nowMs);
  void draw(uint32_t nowMs);
  void setCanvas(M5Canvas& canvas) { canvas_ = &canvas; }

  bool active() const { return state_ != State::Closed; }

 private:
  enum class State : uint8_t {
    Closed,
    Scanning,
    Networks,
    Password,
    Connecting,
    Result,
  };

  struct Network {
    String ssid;
    int32_t rssi = -127;
    bool secure = true;
  };

  static constexpr size_t kMaxNetworks = 20;
  static constexpr size_t kVisibleNetworks = 4;
  static constexpr size_t kMaxPasswordLength = 63;
  static constexpr uint32_t kConnectTimeoutMs = 20'000;

  void startScan();
  void collectScanResults(int count);
  void beginConnection(LocationTime& locationTime);
  void close();
  void markDirty();
  void drawNetworks();
  void drawPassword();
  void drawConnecting();
  void drawResult();

  State state_ = State::Closed;
  Network networks_[kMaxNetworks];
  size_t networkCount_ = 0;
  size_t selected_ = 0;
  String password_;
  String resultMessage_;
  uint32_t connectStartedMs_ = 0;
  uint32_t lastFrameHash_ = 0;
  bool haveFrameHash_ = false;
  bool dirty_ = false;
  M5Canvas* canvas_ = nullptr;
};
