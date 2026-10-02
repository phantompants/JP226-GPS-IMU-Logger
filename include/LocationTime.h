#pragma once

#include <Arduino.h>
#include <FS.h>
#include <Preferences.h>

class LocationTime {
 public:
  void begin(fs::FS& storage, Preferences& preferences);
  void update(double latitude, double longitude, bool locationFresh,
              double speedKmh, uint32_t nowMs);
  void setWifiCredentials(const String& ssid, const String& password);

  const char* zoneName() const { return zoneName_.c_str(); }
  const char* zoneSource() const { return zoneSource_.c_str(); }
  const char* wifiSsid() const { return wifiSsid_.c_str(); }
  bool wifiConnected() const;

 private:
  void loadConfig(fs::FS& storage);
  void startWifi(uint32_t nowMs);
  void startNtp();
  bool lookupTimezone(double latitude, double longitude);
  void applyTimezone(const String& rule, const String& name,
                     const char* source, bool persist);
  static bool jsonString(const String& json, const char* key, String& value);
  static bool jsonIntegerAfter(const String& json, const char* marker,
                               int32_t& value);
  static String posixRuleForZone(const String& ianaZone,
                                int32_t currentUtcOffsetSec);
  static String fixedOffsetRule(int32_t currentUtcOffsetSec);

  Preferences* preferences_ = nullptr;
  String wifiSsid_;
  String wifiPassword_;
  String configuredTimezone_;
  String zoneName_ = "Australia/Sydney";
  String zoneSource_ = "DEFAULT";
  bool timezoneAuto_ = true;
  bool wifiStarted_ = false;
  uint32_t lastWifiAttemptMs_ = 0;
  uint32_t lastLookupAttemptMs_ = 0;
  uint32_t lastLookupSuccessMs_ = 0;
};
