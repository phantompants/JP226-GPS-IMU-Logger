#include "LocationTime.h"

#include <HTTPClient.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>

#include "Config.h"

namespace {

bool parseBool(const String& value, bool fallback) {
  String normalized = value;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized == "1" || normalized == "true" || normalized == "yes" ||
      normalized == "on") {
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" ||
      normalized == "off") {
    return false;
  }
  return fallback;
}

}  // namespace

void LocationTime::begin(fs::FS& storage, Preferences& preferences) {
  preferences_ = &preferences;

  const String savedRule = preferences.getString("tz_rule", "");
  const String savedName = preferences.getString("tz_name", "");
  if (!savedRule.isEmpty()) {
    applyTimezone(savedRule, savedName.isEmpty() ? "Last known" : savedName,
                  "SAVED", false);
  } else {
    applyTimezone(config::kPosixTimezone, "Australia/Sydney", "DEFAULT", false);
  }

  // Credentials entered on the Cardputer take priority. The SD configuration
  // remains a convenient first-boot fallback.
  wifiSsid_ = preferences.getString("wifi_ssid", "");
  wifiPassword_ = preferences.getString("wifi_pass", "");
  loadConfig(storage);
  if (!configuredTimezone_.isEmpty()) {
    applyTimezone(configuredTimezone_, "Configured", "CONFIG", false);
  }

  if (!wifiSsid_.isEmpty()) {
    startWifi(millis());
    configTime(0, 0, config::kNtpServer1, config::kNtpServer2,
               config::kNtpServer3);
  }
}

void LocationTime::loadConfig(fs::FS& storage) {
  File file = storage.open(config::kLoggerConfigPath, FILE_READ);
  if (!file) return;

  String fileWifiSsid;
  String fileWifiPassword;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith("#") || line.startsWith(";")) {
      continue;
    }
    const int separator = line.indexOf('=');
    if (separator <= 0) continue;
    String key = line.substring(0, separator);
    String value = line.substring(separator + 1);
    key.trim();
    value.trim();
    key.toLowerCase();

    if (key == "wifi_ssid") {
      fileWifiSsid = value;
    } else if (key == "wifi_password") {
      fileWifiPassword = value;
    } else if (key == "timezone") {
      configuredTimezone_ = value;
    } else if (key == "timezone_auto") {
      timezoneAuto_ = parseBool(value, timezoneAuto_);
    }
  }
  file.close();

  if (wifiSsid_.isEmpty() && !fileWifiSsid.isEmpty()) {
    wifiSsid_ = fileWifiSsid;
    wifiPassword_ = fileWifiPassword;
  }
}

void LocationTime::startWifi(uint32_t nowMs) {
  if (wifiSsid_.isEmpty()) return;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(wifiSsid_.c_str(), wifiPassword_.c_str());
  wifiStarted_ = true;
  lastWifiAttemptMs_ = nowMs;
  Serial.printf("Wi-Fi: connecting to %s\n", wifiSsid_.c_str());
}

bool LocationTime::wifiConnected() const {
  return WiFi.status() == WL_CONNECTED;
}

void LocationTime::setWifiCredentials(const String& ssid,
                                      const String& password) {
  if (ssid.isEmpty()) return;
  wifiSsid_ = ssid;
  wifiPassword_ = password;
  if (preferences_ != nullptr) {
    preferences_->putString("wifi_ssid", wifiSsid_);
    preferences_->putString("wifi_pass", wifiPassword_);
  }

  WiFi.disconnect(false, false);
  wifiStarted_ = false;
  startWifi(millis());
  configTime(0, 0, config::kNtpServer1, config::kNtpServer2,
             config::kNtpServer3);
}

void LocationTime::update(double latitude, double longitude, bool locationFresh,
                          double speedKmh, uint32_t nowMs) {
  if (!wifiSsid_.isEmpty() && !wifiConnected() &&
      (!wifiStarted_ || nowMs - lastWifiAttemptMs_ >=
                           config::kWifiRetryIntervalMs)) {
    startWifi(nowMs);
  }

  if (!timezoneAuto_ || !wifiConnected() || !locationFresh ||
      speedKmh > config::kMoveStopKmh) {
    return;
  }

  if (lastLookupSuccessMs_ > 0 &&
      nowMs - lastLookupSuccessMs_ < config::kTimezoneLookupIntervalMs) {
    return;
  }
  if (lastLookupAttemptMs_ > 0 &&
      nowMs - lastLookupAttemptMs_ < config::kTimezoneLookupRetryMs) {
    return;
  }

  lastLookupAttemptMs_ = nowMs;
  if (lookupTimezone(latitude, longitude)) {
    lastLookupSuccessMs_ = nowMs;
  }
}

bool LocationTime::lookupTimezone(double latitude, double longitude) {
  WiFiClientSecure client;
  // This public, no-key endpoint returns only timezone metadata. NTP/GPS still
  // controls UTC; a bad response cannot change recorded coordinates or motion.
  client.setInsecure();

  char url[192]{};
  snprintf(url, sizeof(url), "%s?latitude=%.6f&longitude=%.6f",
           config::kTimezoneLookupUrl, latitude, longitude);

  HTTPClient http;
  http.setConnectTimeout(config::kTimezoneHttpTimeoutMs);
  http.setTimeout(config::kTimezoneHttpTimeoutMs);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return false;
  const int responseCode = http.GET();
  if (responseCode != HTTP_CODE_OK) {
    Serial.printf("Timezone lookup failed: HTTP %d\n", responseCode);
    http.end();
    return false;
  }

  const String body = http.getString();
  http.end();
  String ianaZone;
  int32_t offsetSeconds = 0;
  if (!jsonString(body, "timeZone", ianaZone) ||
      !jsonIntegerAfter(body, "\"currentUtcOffset\":{\"seconds\":",
                        offsetSeconds)) {
    Serial.println("Timezone lookup returned an unexpected response");
    return false;
  }

  applyTimezone(posixRuleForZone(ianaZone, offsetSeconds), ianaZone,
                "GPS+NET", true);
  return true;
}

void LocationTime::applyTimezone(const String& rule, const String& name,
                                 const char* source, bool persist) {
  if (rule.isEmpty()) return;
  setenv("TZ", rule.c_str(), 1);
  tzset();
  zoneName_ = name;
  zoneSource_ = source;
  Serial.printf("Timezone: %s (%s, %s)\n", zoneName_.c_str(), rule.c_str(),
                zoneSource_.c_str());

  if (persist && preferences_ != nullptr) {
    preferences_->putString("tz_rule", rule);
    preferences_->putString("tz_name", name);
  }
}

bool LocationTime::jsonString(const String& json, const char* key,
                              String& value) {
  const String marker = String("\"") + key + "\":\"";
  const int start = json.indexOf(marker);
  if (start < 0) return false;
  const int valueStart = start + marker.length();
  const int valueEnd = json.indexOf('"', valueStart);
  if (valueEnd < 0) return false;
  value = json.substring(valueStart, valueEnd);
  return !value.isEmpty();
}

bool LocationTime::jsonIntegerAfter(const String& json, const char* marker,
                                    int32_t& value) {
  const int start = json.indexOf(marker);
  if (start < 0) return false;
  const char* number = json.c_str() + start + strlen(marker);
  char* end = nullptr;
  const long parsed = strtol(number, &end, 10);
  if (end == number) return false;
  value = static_cast<int32_t>(parsed);
  return true;
}

String LocationTime::posixRuleForZone(const String& ianaZone,
                                      int32_t currentUtcOffsetSec) {
  if (ianaZone == "Australia/Sydney" || ianaZone == "Australia/Melbourne" ||
      ianaZone == "Australia/Hobart" || ianaZone == "Australia/ACT") {
    return "AEST-10AEDT,M10.1.0,M4.1.0/3";
  }
  if (ianaZone == "Australia/Brisbane" || ianaZone == "Australia/Lindeman") {
    return "AEST-10";
  }
  if (ianaZone == "Australia/Adelaide" ||
      ianaZone == "Australia/Broken_Hill") {
    return "ACST-9:30ACDT,M10.1.0,M4.1.0/3";
  }
  if (ianaZone == "Australia/Darwin") return "ACST-9:30";
  if (ianaZone == "Australia/Perth") return "AWST-8";
  if (ianaZone == "Australia/Eucla") return "ACWST-8:45";
  if (ianaZone == "Australia/Lord_Howe") {
    return "LHST-10:30LHDT-11,M10.1.0,M4.1.0/2";
  }
  return fixedOffsetRule(currentUtcOffsetSec);
}

String LocationTime::fixedOffsetRule(int32_t currentUtcOffsetSec) {
  const bool eastOfUtc = currentUtcOffsetSec >= 0;
  uint32_t magnitude = static_cast<uint32_t>(
      eastOfUtc ? currentUtcOffsetSec : -currentUtcOffsetSec);
  const uint32_t hours = magnitude / 3600U;
  const uint32_t minutes = (magnitude % 3600U) / 60U;
  char rule[24]{};
  snprintf(rule, sizeof(rule), "LOC%c%lu:%02lu", eastOfUtc ? '-' : '+',
           static_cast<unsigned long>(hours),
           static_cast<unsigned long>(minutes));
  return String(rule);
}
