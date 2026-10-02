#pragma once

#include <Arduino.h>
#include <FS.h>
#include <Preferences.h>

#include "TelemetryData.h"

struct VehicleContext {
  String roadType;
  String roadSurface;  // DRY, DAMP, WET or VERY WET; empty if not set
  float tyreSetFrontPsi = NAN;
  float tyreSetRearPsi = NAN;
  String suspensionFront;
  String suspensionRear;
  String vehicleLoad;
};

struct WaypointRecord {
  String id;
  String timestamp;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitudeM = NAN;
  double hdop = NAN;
  double speedKmh = NAN;
  double headingDeg = NAN;
  String name;
  String source = "USER_WAYPOINT";
  String note;
  String photoReference;
  String category = "GENERIC";
  VehicleContext vehicle;
  String telemetrySource;
  uint32_t revision = 0;
};

class WaypointStore {
 public:
  void begin(fs::FS& storage, Preferences& preferences,
             const char* directory);
  bool create(const GpsSnapshot& gps, const TelemetryMetadata& metadata,
              const VehicleContext& vehicle, time_t nowUtc,
              const String& timestamp, const String& category,
              WaypointRecord& created);
  bool editName(const String& name, time_t nowUtc, const String& timestamp);
  bool editNote(const String& note, time_t nowUtc, const String& timestamp);
  bool editPhoto(const String& photo, time_t nowUtc, const String& timestamp);
  bool appendEvent(const String& type, const String& waypointId,
                   const String& detail, const String& telemetrySource,
                   time_t nowUtc, const String& timestamp);
  bool hasLast() const { return !last_.id.isEmpty(); }
  const WaypointRecord& last() const { return last_; }
  double distanceFromLastM(double latitude, double longitude) const;
  static String csvField(const String& value);
  static String sanitizeText(const String& input, size_t maximum = 64);

 private:
  enum class EditField : uint8_t { Name, Note, Photo };
  bool edit(EditField field, const String& text, time_t nowUtc,
            const String& timestamp);
  bool appendWaypoint(const WaypointRecord& record, const char* action,
                      time_t nowUtc, const String& recordedAt);
  String pathForDay(const char* prefix, time_t nowUtc) const;
  void loadLast();
  void saveLast();

  fs::FS* storage_ = nullptr;
  Preferences* preferences_ = nullptr;
  String directory_;
  WaypointRecord last_;
};
