#include "WeatherService.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

namespace {

constexpr uint32_t kRefreshMs = 15UL * 60UL * 1000UL;
constexpr uint32_t kRetryMs = 2UL * 60UL * 1000UL;
constexpr double kMoveRefreshM = 10'000.0;

float jsonFloat(JsonVariantConst value) {
  return value.is<float>() ? value.as<float>() : NAN;
}

// The lookup cannot verify the server's certificate (no certificate store
// on the device), so values outside physical limits are dropped rather than
// logged.
float within(float value, float low, float high) {
  return std::isnan(value) || value < low || value > high ? NAN : value;
}

}  // namespace

void WeatherService::update(double latitude, double longitude,
                            bool positionFresh, uint32_t nowMs) {
  if (!positionFresh || busy_ || WiFi.status() != WL_CONNECTED) return;
  if (lastAttemptMs_ != 0 && nowMs - lastAttemptMs_ < kRetryMs) return;
  const bool moved =
      haveLast_ && TinyGPSPlus::distanceBetween(latitude, longitude,
                                                lastLatitude_,
                                                lastLongitude_) > kMoveRefreshM;
  if (lastSuccessMs_ != 0 && nowMs - lastSuccessMs_ < kRefreshMs && !moved) {
    return;
  }
  lastAttemptMs_ = nowMs;
  jobLatitude_ = latitude;
  jobLongitude_ = longitude;
  busy_ = true;
  // TLS needs a deep stack; the lookup runs beside loop() so logging never
  // waits on the network.
  if (xTaskCreate(taskEntry, "weather", 12288, this, 1, nullptr) != pdPASS) {
    busy_ = false;
  }
}

void WeatherService::taskEntry(void* argument) {
  static_cast<WeatherService*>(argument)->runFetch();
  vTaskDelete(nullptr);
}

void WeatherService::runFetch() {
  char url[320];
  snprintf(url, sizeof(url),
           "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
           "&current=temperature_2m,relative_humidity_2m,surface_pressure,"
           "precipitation,weather_code,wind_speed_10m,wind_direction_10m"
           "&wind_speed_unit=kmh",
           jobLatitude_, jobLongitude_);
  WiFiClientSecure client;
  client.setInsecure();  // Public data; no certificate store on the device.
  HTTPClient http;
  http.setTimeout(5000);
  WeatherReading result;
  if (http.begin(client, url)) {
    if (http.GET() == HTTP_CODE_OK) {
      JsonDocument doc;
      if (!deserializeJson(doc, http.getString())) {
        JsonVariantConst current = doc["current"];
        result.temperatureC = jsonFloat(current["temperature_2m"]);
        result.humidityPct = jsonFloat(current["relative_humidity_2m"]);
        result.pressureHpa = jsonFloat(current["surface_pressure"]);
        result.precipitationMm = jsonFloat(current["precipitation"]);
        result.windKmh = jsonFloat(current["wind_speed_10m"]);
        result.windDirectionDeg = jsonFloat(current["wind_direction_10m"]);
        result.code = current["weather_code"] | -1;
        result.temperatureC = within(result.temperatureC, -60, 60);
        result.humidityPct = within(result.humidityPct, 0, 100);
        result.pressureHpa = within(result.pressureHpa, 300, 1100);
        result.precipitationMm = within(result.precipitationMm, 0, 500);
        result.windKmh = within(result.windKmh, 0, 400);
        result.windDirectionDeg = within(result.windDirectionDeg, 0, 360);
        if (result.code < 0 || result.code > 99) result.code = -1;
        result.valid = !std::isnan(result.temperatureC);
      }
    }
    http.end();
  }
  if (result.valid) {
    result.fetchedMs = millis();
    portENTER_CRITICAL(&mux_);
    reading_ = result;
    portEXIT_CRITICAL(&mux_);
    lastSuccessMs_ = result.fetchedMs;
    lastLatitude_ = jobLatitude_;
    lastLongitude_ = jobLongitude_;
    haveLast_ = true;
  }
  busy_ = false;
}

bool WeatherService::latest(WeatherReading& reading) const {
  portENTER_CRITICAL(&mux_);
  reading = reading_;
  portEXIT_CRITICAL(&mux_);
  return reading.valid;
}

// Short labels for the WMO weather codes Open-Meteo reports.
const char* WeatherService::describe(int code) {
  if (code < 0) return "--";
  if (code == 0) return "Clear";
  if (code <= 2) return "Partly cloudy";
  if (code == 3) return "Overcast";
  if (code <= 48) return "Fog";
  if (code <= 57) return "Drizzle";
  if (code <= 67) return "Rain";
  if (code <= 77) return "Snow";
  if (code <= 82) return "Showers";
  if (code <= 86) return "Snow showers";
  return "Thunderstorm";
}
