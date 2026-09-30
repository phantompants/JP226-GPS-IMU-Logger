#include "PlaceResolver.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <cstring>

namespace {

constexpr uint32_t kMinimumQueryIntervalMs = 5UL * 60UL * 1000UL;
constexpr double kMinimumQueryDistanceM = 2000.0;
constexpr double kMaximumPlaceCentroidDistanceM = 2000.0;

String firstName(JsonObjectConst address) {
  for (const char* key : {"city", "town", "village", "hamlet", "locality",
                          "municipality"}) {
    const char* value = address[key] | "";
    if (value[0] != '\0') return String(value);
  }
  return "";
}

}  // namespace

bool NominatimCompatibleProvider::lookup(double latitude, double longitude,
                                          String& name) {
  name = "";
  if (!configured() || WiFi.status() != WL_CONNECTED) return false;
  String url = endpoint_;
  url += endpoint_.indexOf('?') >= 0 ? '&' : '?';
  url += "format=jsonv2&addressdetails=1&zoom=12&layer=address&lat=";
  url += String(latitude, 7);
  url += "&lon=";
  url += String(longitude, 7);

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  WiFiClient plainClient;
  HTTPClient http;
  http.setConnectTimeout(3000);
  http.setTimeout(4000);
  http.setUserAgent("JP226-GPS-IMU-Logger/1.0");
  const bool https = url.startsWith("https://");
  if (!(https ? http.begin(secureClient, url)
              : http.begin(plainClient, url))) return false;
  const int response = http.GET();
  if (response != HTTP_CODE_OK) {
    http.end();
    return false;
  }
  const String body = http.getString();
  http.end();
  JsonDocument document;
  if (deserializeJson(document, body)) return false;
  JsonObjectConst address = document["address"].as<JsonObjectConst>();
  String locality = firstName(address);
  if (locality.isEmpty()) return true;

  const char* latitudeText = document["lat"] | "";
  const char* longitudeText = document["lon"] | "";
  if (latitudeText[0] == '\0' || longitudeText[0] == '\0') return true;
  const double placeLatitude = String(latitudeText).toDouble();
  const double placeLongitude = String(longitudeText).toDouble();
  if (TinyGPSPlus::distanceBetween(latitude, longitude, placeLatitude,
                                   placeLongitude) >
      kMaximumPlaceCentroidDistanceM) {
    return true;
  }
  locality.trim();
  if (locality.length() > 63) locality = locality.substring(0, 63);
  name = locality;
  return true;
}

void PlaceResolver::begin(fs::FS& storage, const char* configPath) {
  File file = storage.open(configPath, FILE_READ);
  if (!file) return;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.startsWith("#") || line.startsWith(";") || line.isEmpty()) {
      continue;
    }
    const int separator = line.indexOf('=');
    if (separator < 0) continue;
    String key = line.substring(0, separator);
    String value = line.substring(separator + 1);
    key.trim();
    value.trim();
    key.toLowerCase();
    if (key == "place_lookup_url" &&
        (value.startsWith("https://") || value.startsWith("http://"))) {
      provider_.setEndpoint(value);
    }
  }
  file.close();
}

void PlaceResolver::update(double latitude, double longitude,
                           bool positionFresh, uint32_t nowMs) {
  if (!provider_.configured() || !positionFresh ||
      WiFi.status() != WL_CONNECTED || busy_) {
    return;
  }
  if (haveLastQuery_) {
    if (nowMs - lastQueryMs_ < kMinimumQueryIntervalMs) return;
    if (!currentPlace_.isEmpty() &&
        TinyGPSPlus::distanceBetween(latitude, longitude, lastQueryLatitude_,
                                     lastQueryLongitude_) <
            kMinimumQueryDistanceM) {
      return;
    }
  }
  jobLatitude_ = latitude;
  jobLongitude_ = longitude;
  lastQueryLatitude_ = latitude;
  lastQueryLongitude_ = longitude;
  lastQueryMs_ = nowMs;
  haveLastQuery_ = true;
  busy_ = true;
  if (xTaskCreate(taskEntry, "place-lookup", 12288, this, 1, nullptr) !=
      pdPASS) {
    busy_ = false;
  }
}

void PlaceResolver::taskEntry(void* argument) {
  auto* self = static_cast<PlaceResolver*>(argument);
  self->runLookup();
  vTaskDelete(nullptr);
}

void PlaceResolver::runLookup() {
  String result;
  if (provider_.lookup(jobLatitude_, jobLongitude_, result)) {
    portENTER_CRITICAL(&mux_);
    result.toCharArray(result_, sizeof(result_));
    resultReady_ = true;
    portEXIT_CRITICAL(&mux_);
  }
  busy_ = false;
}

bool PlaceResolver::takeChange(String& place) {
  char result[64]{};
  bool ready = false;
  portENTER_CRITICAL(&mux_);
  if (resultReady_) {
    std::memcpy(result, result_, sizeof(result));
    resultReady_ = false;
    ready = true;
  }
  portEXIT_CRITICAL(&mux_);
  if (!ready || currentPlace_ == result) return false;
  currentPlace_ = result;
  place = currentPlace_;
  return true;
}
