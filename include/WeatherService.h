#pragma once

#include <Arduino.h>

#include <atomic>

// Local weather from Open-Meteo (free, no account or key) for the current GPS
// position, fetched in a background task while the logger is on Wi-Fi: every
// 15 minutes, or sooner after moving 10 km. Logged with every CSV row.
struct WeatherReading {
  bool valid = false;
  float temperatureC = NAN;
  float humidityPct = NAN;
  float pressureHpa = NAN;
  float windKmh = NAN;
  float windDirectionDeg = NAN;
  float precipitationMm = NAN;
  int code = -1;  // WMO weather code (0 clear ... 95+ thunderstorm)
  uint32_t fetchedMs = 0;
};

class WeatherService {
 public:
  void update(double latitude, double longitude, bool positionFresh,
              uint32_t nowMs);
  // False until the first successful fetch.
  bool latest(WeatherReading& reading) const;
  static const char* describe(int code);

 private:
  static void taskEntry(void* argument);
  void runFetch();

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  WeatherReading reading_{};
  std::atomic<bool> busy_{false};
  double jobLatitude_ = 0.0;
  double jobLongitude_ = 0.0;
  double lastLatitude_ = 0.0;
  double lastLongitude_ = 0.0;
  bool haveLast_ = false;
  uint32_t lastAttemptMs_ = 0;
  uint32_t lastSuccessMs_ = 0;
};
