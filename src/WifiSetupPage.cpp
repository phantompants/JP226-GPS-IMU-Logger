#include "WifiSetupPage.h"

#include <M5Cardputer.h>
#include <WiFi.h>

#include "LocationTime.h"

namespace {

constexpr uint16_t kBackground = TFT_BLACK;
constexpr uint16_t kForeground = TFT_WHITE;
constexpr uint16_t kAccent = TFT_CYAN;
constexpr uint16_t kSuccess = TFT_GREEN;
constexpr uint16_t kWarning = TFT_ORANGE;

String clipped(const String& value, size_t maximum) {
  if (value.length() <= maximum) return value;
  if (maximum <= 3) return value.substring(0, maximum);
  return value.substring(0, maximum - 3) + "...";
}

}  // namespace

void WifiSetupPage::open() {
  if (active()) return;
  startScan();
}

void WifiSetupPage::startScan() {
  WiFi.mode(WIFI_STA);
  WiFi.scanDelete();
  networkCount_ = 0;
  selected_ = 0;
  state_ = State::Scanning;
  resultMessage_ = "Scanning for Wi-Fi...";
  WiFi.scanNetworks(true, true);
  markDirty();
}

void WifiSetupPage::collectScanResults(int count) {
  networkCount_ = 0;
  for (int i = 0; i < count && networkCount_ < kMaxNetworks; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.isEmpty()) continue;

    size_t existing = kMaxNetworks;
    for (size_t n = 0; n < networkCount_; ++n) {
      if (networks_[n].ssid == ssid) {
        existing = n;
        break;
      }
    }

    const int32_t rssi = WiFi.RSSI(i);
    const bool secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    if (existing < networkCount_) {
      if (rssi > networks_[existing].rssi) {
        networks_[existing].rssi = rssi;
        networks_[existing].secure = secure;
      }
      continue;
    }

    networks_[networkCount_].ssid = ssid;
    networks_[networkCount_].rssi = rssi;
    networks_[networkCount_].secure = secure;
    ++networkCount_;
  }
  WiFi.scanDelete();
  state_ = State::Networks;
  resultMessage_ = networkCount_ == 0 ? "No networks found" : "";
  markDirty();
}

void WifiSetupPage::handleInput(LocationTime& locationTime) {
  if (!active() || !M5Cardputer.Keyboard.isChange() ||
      !M5Cardputer.Keyboard.isPressed()) {
    return;
  }

  auto& keys = M5Cardputer.Keyboard.keysState();

  if (state_ == State::Password) {
    if (keys.esc) {
      state_ = State::Networks;
      password_ = "";
      markDirty();
      return;
    }
    if (keys.del || keys.backspace) {
      if (!password_.isEmpty()) password_.remove(password_.length() - 1);
      markDirty();
      return;
    }
    if (keys.enter) {
      beginConnection(locationTime);
      return;
    }
    for (char c : keys.word) {
      if (c >= 32 && c <= 126 && password_.length() < kMaxPasswordLength) {
        password_ += c;
      }
    }
    markDirty();
    return;
  }

  if (keys.esc || M5Cardputer.Keyboard.isKeyPressed('q')) {
    close();
    return;
  }

  if (state_ == State::Networks) {
    if (M5Cardputer.Keyboard.isKeyPressed('r')) {
      startScan();
    } else if (networkCount_ > 0 &&
               (keys.up || M5Cardputer.Keyboard.isKeyPressed(','))) {
      selected_ = selected_ == 0 ? networkCount_ - 1 : selected_ - 1;
      markDirty();
    } else if (networkCount_ > 0 &&
               (keys.down || M5Cardputer.Keyboard.isKeyPressed('.'))) {
      selected_ = (selected_ + 1) % networkCount_;
      markDirty();
    } else if (networkCount_ > 0 && keys.enter) {
      password_ = "";
      if (networks_[selected_].secure) {
        state_ = State::Password;
        markDirty();
      } else {
        beginConnection(locationTime);
      }
    }
  } else if (state_ == State::Result) {
    if (M5Cardputer.Keyboard.isKeyPressed('r')) {
      startScan();
    } else if (keys.enter) {
      close();
    }
  }
}

void WifiSetupPage::beginConnection(LocationTime& locationTime) {
  if (networkCount_ == 0 || selected_ >= networkCount_) return;
  locationTime.setWifiCredentials(networks_[selected_].ssid, password_);
  password_ = "";
  connectStartedMs_ = millis();
  resultMessage_ = "Connecting...";
  state_ = State::Connecting;
  markDirty();
}

void WifiSetupPage::update(uint32_t nowMs) {
  if (state_ == State::Scanning) {
    const int result = WiFi.scanComplete();
    if (result >= 0) {
      collectScanResults(result);
    } else if (result == WIFI_SCAN_FAILED) {
      resultMessage_ = "Scan failed - press R to retry";
      state_ = State::Networks;
      markDirty();
    }
  } else if (state_ == State::Connecting) {
    if (WiFi.status() == WL_CONNECTED) {
      resultMessage_ = "Connected to " + WiFi.SSID();
      state_ = State::Result;
      markDirty();
    } else if (nowMs - connectStartedMs_ >= kConnectTimeoutMs) {
      resultMessage_ = "Connection failed or timed out";
      state_ = State::Result;
      markDirty();
    }
  }
}

void WifiSetupPage::draw(uint32_t nowMs) {
  if (!active() || (!dirty_ && nowMs - lastDrawMs_ < 500)) return;
  dirty_ = false;
  lastDrawMs_ = nowMs;

  auto& display = M5Cardputer.Display;
  display.startWrite();
  display.fillScreen(kBackground);
  display.setTextDatum(top_left);
  display.setTextWrap(false);
  display.setTextSize(2);
  display.setCursor(2, 2);
  display.setTextColor(kAccent, kBackground);
  display.println("Wi-Fi setup");
  display.drawFastHLine(0, 19, display.width(), kAccent);
  display.setCursor(2, 23);
  display.setTextColor(kForeground, kBackground);

  switch (state_) {
    case State::Scanning:
      display.println("Scanning nearby");
      display.println("Wi-Fi networks...");
      display.setTextColor(kSuccess, kBackground);
      display.println("Logging continues");
      display.setTextSize(1);
      display.setTextColor(kForeground, kBackground);
      display.println("Q: close");
      break;
    case State::Networks:
      drawNetworks();
      break;
    case State::Password:
      drawPassword();
      break;
    case State::Connecting:
      drawConnecting();
      break;
    case State::Result:
      drawResult();
      break;
    default:
      break;
  }
  display.endWrite();
}

void WifiSetupPage::drawNetworks() {
  auto& display = M5Cardputer.Display;
  display.setTextSize(2);
  if (networkCount_ == 0) {
    display.setTextColor(kWarning, kBackground);
    display.println("No networks found");
  } else {
    size_t first = selected_ >= kVisibleNetworks
                       ? selected_ - kVisibleNetworks + 1
                       : 0;
    for (size_t row = 0; row < kVisibleNetworks; ++row) {
      const size_t index = first + row;
      if (index >= networkCount_) break;
      display.setTextColor(index == selected_ ? TFT_BLACK : kForeground,
                           index == selected_ ? kAccent : kBackground);
      display.printf("%c%-16s%s\n", index == selected_ ? '>' : ' ',
                     clipped(networks_[index].ssid, 16).c_str(),
                     networks_[index].secure ? "*" : " ");
    }
  }
  display.setTextSize(1);
  display.setTextColor(kForeground, kBackground);
  display.println("Enter: select   ,/.: move");
  display.println("R: rescan  Q: close  *=locked");
}

void WifiSetupPage::drawPassword() {
  auto& display = M5Cardputer.Display;
  display.setTextSize(2);
  display.println("Network:");
  display.println(clipped(networks_[selected_].ssid, 19));
  display.println("Password:");
  String masked;
  const size_t visible = password_.length() > 16 ? 16 : password_.length();
  for (size_t i = 0; i < visible; ++i) masked += '*';
  if (password_.length() > visible) masked = ".." + masked;
  display.setTextColor(kAccent, kBackground);
  display.println(masked + "_");
  display.setTextColor(kForeground, kBackground);
  display.setTextSize(1);
  display.printf("%u/63 chars  Enter: connect\n",
                 static_cast<unsigned>(password_.length()));
  display.println("Backspace: delete  Fn+`: back");
}

void WifiSetupPage::drawConnecting() {
  auto& display = M5Cardputer.Display;
  display.setTextSize(2);
  display.println("Connecting to:");
  display.println(clipped(networks_[selected_].ssid, 19));
  display.setTextColor(kSuccess, kBackground);
  display.println("Logging continues");
  display.setTextSize(1);
  display.setTextColor(kForeground, kBackground);
  display.println("Q: close");
}

void WifiSetupPage::drawResult() {
  auto& display = M5Cardputer.Display;
  display.setTextSize(2);
  display.setTextColor(WiFi.status() == WL_CONNECTED ? kSuccess : kWarning,
                       kBackground);
  if (WiFi.status() == WL_CONNECTED) display.println("CONNECTED");
  else display.println("NOT CONNECTED");
  if (WiFi.status() == WL_CONNECTED) {
    display.setTextColor(kForeground, kBackground);
    display.println(clipped(WiFi.SSID(), 19));
    display.setTextSize(1);
    display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    display.setTextColor(kForeground, kBackground);
    display.setTextSize(1);
    display.println("Check password or signal.");
  }
  display.setTextColor(kForeground, kBackground);
  display.setTextSize(1);
  display.println("Enter/Q: close  R: scan again");
}

void WifiSetupPage::close() {
  WiFi.scanDelete();
  password_ = "";
  state_ = State::Closed;
  dirty_ = false;
}

void WifiSetupPage::markDirty() {
  dirty_ = true;
  lastDrawMs_ = 0;
}
