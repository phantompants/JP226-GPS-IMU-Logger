#pragma once

#include <Arduino.h>
#include <FS.h>
#include <atomic>

class ILocationProvider {
 public:
  virtual ~ILocationProvider() = default;
  virtual bool lookup(double latitude, double longitude, String& name) = 0;
};

// The endpoint comes from logger.cfg and can point at a self-hosted or other
// Nominatim-compatible reverse-geocoding service.
class NominatimCompatibleProvider : public ILocationProvider {
 public:
  void setEndpoint(const String& endpoint) { endpoint_ = endpoint; }
  bool configured() const { return !endpoint_.isEmpty(); }
  bool lookup(double latitude, double longitude, String& name) override;

 private:
  String endpoint_;
};

class PlaceResolver {
 public:
  void begin(fs::FS& storage, const char* configPath);
  void update(double latitude, double longitude, bool positionFresh,
              uint32_t nowMs);
  bool takeChange(String& place);
  bool enabled() const { return provider_.configured(); }

 private:
  static void taskEntry(void* argument);
  void runLookup();

  NominatimCompatibleProvider provider_;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  char result_[64]{};
  std::atomic<bool> busy_{false};
  bool resultReady_ = false;
  double jobLatitude_ = 0.0;
  double jobLongitude_ = 0.0;
  double lastQueryLatitude_ = 0.0;
  double lastQueryLongitude_ = 0.0;
  bool haveLastQuery_ = false;
  uint32_t lastQueryMs_ = 0;
  String currentPlace_;
};
