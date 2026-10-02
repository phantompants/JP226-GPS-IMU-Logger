#include "WaypointStore.h"

#include <TinyGPSPlus.h>
#include <time.h>

#include <cmath>

namespace {

void appendNumber(String& row, double value, unsigned decimals) {
  row += ',';
  if (std::isfinite(value)) row += String(value, decimals);
}

const char* sourceName(TelemetrySource source) {
  switch (source) {
    case TelemetrySource::AtomS3Remote: return "ATOMS3_REMOTE";
    case TelemetrySource::CardputerAdv: return "CARDPUTER_ADV";
    case TelemetrySource::LocalGpsAtomImu: return "LOCAL_GPS_ATOM_IMU";
    default: return "LOCAL_GPS";
  }
}

}  // namespace

void WaypointStore::begin(fs::FS& storage, Preferences& preferences,
                          const char* directory) {
  storage_ = &storage;
  preferences_ = &preferences;
  directory_ = directory;
  loadLast();
}

String WaypointStore::sanitizeText(const String& input, size_t maximum) {
  String result;
  result.reserve(maximum);
  for (size_t index = 0; index < input.length() && result.length() < maximum;
       ++index) {
    const char c = input[index];
    if (c >= 32 && c <= 126) result += c;
  }
  result.trim();
  return result;
}

String WaypointStore::csvField(const String& value) {
  const bool quote = value.indexOf(',') >= 0 || value.indexOf('"') >= 0 ||
                     value.indexOf('\n') >= 0 || value.indexOf('\r') >= 0;
  if (!quote) return value;
  String result = "\"";
  for (size_t index = 0; index < value.length(); ++index) {
    if (value[index] == '"') result += '"';
    result += value[index];
  }
  result += '"';
  return result;
}

String WaypointStore::pathForDay(const char* prefix, time_t nowUtc) const {
  struct tm local {};
  localtime_r(&nowUtc, &local);
  char name[80]{};
  snprintf(name, sizeof(name), "%s/%s%04d-%02d-%02d.csv",
           directory_.c_str(), prefix, local.tm_year + 1900,
           local.tm_mon + 1, local.tm_mday);
  return String(name);
}

bool WaypointStore::appendWaypoint(const WaypointRecord& record,
                                   const char* action, time_t nowUtc,
                                   const String& recordedAt) {
  if (storage_ == nullptr ||
      (!storage_->exists(directory_) && !storage_->mkdir(directory_))) {
    return false;
  }
  File file = storage_->open(pathForDay("waypoints_", nowUtc), FILE_APPEND);
  if (!file) return false;
  if (file.size() == 0) {
    if (file.println(
        "waypoint_id,timestamp,latitude,longitude,altitude_m,hdop,speed_kmh,"
        "heading_deg,poi_name,poi_source,user_note,photo_reference,road_type,"
        "tyre_set_front_psi,tyre_set_rear_psi,suspension_front,"
        "suspension_rear,vehicle_load,telemetry_source,category,revision,"
        "record_action,recorded_at") == 0) {
      file.close();
      return false;
    }
  }

  String row = csvField(record.id) + ',' + csvField(record.timestamp);
  appendNumber(row, record.latitude, 7);
  appendNumber(row, record.longitude, 7);
  appendNumber(row, record.altitudeM, 2);
  appendNumber(row, record.hdop, 2);
  appendNumber(row, record.speedKmh, 2);
  appendNumber(row, record.headingDeg, 2);
  row += ',' + csvField(record.name) + ',' + csvField(record.source) + ',' +
         csvField(record.note) + ',' + csvField(record.photoReference) + ',' +
         csvField(record.vehicle.roadType);
  appendNumber(row, record.vehicle.tyreSetFrontPsi, 1);
  appendNumber(row, record.vehicle.tyreSetRearPsi, 1);
  row += ',' + csvField(record.vehicle.suspensionFront) + ',' +
         csvField(record.vehicle.suspensionRear) + ',' +
         csvField(record.vehicle.vehicleLoad) + ',' +
         csvField(record.telemetrySource) + ',' + csvField(record.category) +
         ',' + String(record.revision) + ',' + action + ',' + recordedAt;

  const size_t written = file.println(row);
  file.flush();
  file.close();
  return written > row.length();
}

bool WaypointStore::appendEvent(const String& type, const String& waypointId,
                                const String& detail,
                                const String& telemetrySource,
                                time_t nowUtc, const String& timestamp) {
  if (storage_ == nullptr ||
      (!storage_->exists(directory_) && !storage_->mkdir(directory_))) {
    return false;
  }
  File file = storage_->open(pathForDay("events_", nowUtc), FILE_APPEND);
  if (!file) return false;
  if (file.size() == 0) {
    file.println("timestamp,event_type,waypoint_id,detail,telemetry_source");
  }
  const String row = csvField(timestamp) + ',' + csvField(type) + ',' +
                     csvField(waypointId) + ',' + csvField(detail) + ',' +
                     csvField(telemetrySource);
  const size_t written = file.println(row);
  file.flush();
  file.close();
  return written > row.length();
}

bool WaypointStore::create(const GpsSnapshot& gps,
                           const TelemetryMetadata& metadata,
                           const VehicleContext& vehicle, time_t nowUtc,
                           const String& timestamp, const String& category,
                           WaypointRecord& created) {
  if (!gps.fixValid || nowUtc <= 0 || storage_ == nullptr ||
      preferences_ == nullptr) {
    return false;
  }
  const uint32_t next = preferences_->getULong("wp_next", 1);
  char id[20]{};
  snprintf(id, sizeof(id), "WP%05lu", static_cast<unsigned long>(next));
  WaypointRecord record;
  record.id = id;
  record.timestamp = timestamp;
  record.latitude = gps.latitude;
  record.longitude = gps.longitude;
  if (gps.altitudeFresh) record.altitudeM = gps.altitudeM;
  if (gps.hdopValid) record.hdop = gps.hdop;
  if (gps.speedFresh) record.speedKmh = gps.speedKmh;
  if (gps.courseFresh) record.headingDeg = gps.courseDeg;
  record.name = record.id;
  record.category = sanitizeText(category, 24);
  if (record.category.isEmpty()) record.category = "GENERIC";
  record.vehicle = vehicle;
  record.telemetrySource = sourceName(metadata.source);
  // Reserve the ID before the SD write: an interrupted write may leave a gap,
  // but a reboot cannot reuse an ID already recorded on the card.
  if (preferences_->putULong("wp_next", next + 1) == 0) return false;
  if (!appendWaypoint(record, "CREATE", nowUtc, timestamp)) return false;
  last_ = record;
  saveLast();
  appendEvent("WAYPOINT_CREATED", record.id, record.category,
              record.telemetrySource, nowUtc, timestamp);
  created = record;
  return true;
}

bool WaypointStore::edit(EditField field, const String& text, time_t nowUtc,
                          const String& timestamp) {
  if (!hasLast() || nowUtc <= 0) return false;
  WaypointRecord updated = last_;
  const String clean = sanitizeText(text);
  if (clean.isEmpty()) return false;
  String eventType;
  if (field == EditField::Name) {
    updated.name = clean;
    updated.source = "USER_LABEL";
    eventType = "WAYPOINT_RENAMED";
  } else if (field == EditField::Note) {
    updated.note = clean;
    eventType = "WAYPOINT_NOTE";
  } else {
    updated.photoReference = clean;
    if (updated.name == updated.id) updated.source = "PHOTO_REFERENCE";
    eventType = "PHOTO_REFERENCE";
  }
  ++updated.revision;
  if (!appendWaypoint(updated, "EDIT", nowUtc, timestamp)) return false;
  last_ = updated;
  saveLast();
  appendEvent(eventType, updated.id, clean, updated.telemetrySource, nowUtc,
              timestamp);
  return true;
}

bool WaypointStore::editName(const String& name, time_t nowUtc,
                             const String& timestamp) {
  return edit(EditField::Name, name, nowUtc, timestamp);
}

bool WaypointStore::editNote(const String& note, time_t nowUtc,
                             const String& timestamp) {
  return edit(EditField::Note, note, nowUtc, timestamp);
}

bool WaypointStore::editPhoto(const String& photo, time_t nowUtc,
                              const String& timestamp) {
  return edit(EditField::Photo, photo, nowUtc, timestamp);
}

double WaypointStore::distanceFromLastM(double latitude,
                                        double longitude) const {
  if (!hasLast()) return NAN;
  return TinyGPSPlus::distanceBetween(last_.latitude, last_.longitude,
                                      latitude, longitude);
}

void WaypointStore::saveLast() {
  if (preferences_ == nullptr) return;
  preferences_->putString("wp_id", last_.id);
  preferences_->putString("wp_time", last_.timestamp);
  preferences_->putDouble("wp_lat", last_.latitude);
  preferences_->putDouble("wp_lon", last_.longitude);
  preferences_->putDouble("wp_alt", last_.altitudeM);
  preferences_->putDouble("wp_hdop", last_.hdop);
  preferences_->putDouble("wp_speed", last_.speedKmh);
  preferences_->putDouble("wp_head", last_.headingDeg);
  preferences_->putString("wp_name", last_.name);
  preferences_->putString("wp_source", last_.source);
  preferences_->putString("wp_note", last_.note);
  preferences_->putString("wp_photo", last_.photoReference);
  preferences_->putString("wp_cat", last_.category);
  preferences_->putString("wp_road", last_.vehicle.roadType);
  preferences_->putFloat("wp_front", last_.vehicle.tyreSetFrontPsi);
  preferences_->putFloat("wp_rear", last_.vehicle.tyreSetRearPsi);
  preferences_->putString("wp_susf", last_.vehicle.suspensionFront);
  preferences_->putString("wp_susr", last_.vehicle.suspensionRear);
  preferences_->putString("wp_load", last_.vehicle.vehicleLoad);
  preferences_->putString("wp_tele", last_.telemetrySource);
  preferences_->putULong("wp_rev", last_.revision);
}

void WaypointStore::loadLast() {
  if (preferences_ == nullptr) return;
  last_.id = preferences_->getString("wp_id", "");
  if (last_.id.isEmpty()) return;
  last_.timestamp = preferences_->getString("wp_time", "");
  last_.latitude = preferences_->getDouble("wp_lat", 0.0);
  last_.longitude = preferences_->getDouble("wp_lon", 0.0);
  last_.altitudeM = preferences_->getDouble("wp_alt", NAN);
  last_.hdop = preferences_->getDouble("wp_hdop", NAN);
  last_.speedKmh = preferences_->getDouble("wp_speed", NAN);
  last_.headingDeg = preferences_->getDouble("wp_head", NAN);
  last_.name = preferences_->getString("wp_name", last_.id);
  last_.source = preferences_->getString("wp_source", "USER_WAYPOINT");
  last_.note = preferences_->getString("wp_note", "");
  last_.photoReference = preferences_->getString("wp_photo", "");
  last_.category = preferences_->getString("wp_cat", "GENERIC");
  last_.vehicle.roadType = preferences_->getString("wp_road", "");
  last_.vehicle.tyreSetFrontPsi = preferences_->getFloat("wp_front", NAN);
  last_.vehicle.tyreSetRearPsi = preferences_->getFloat("wp_rear", NAN);
  last_.vehicle.suspensionFront = preferences_->getString("wp_susf", "");
  last_.vehicle.suspensionRear = preferences_->getString("wp_susr", "");
  last_.vehicle.vehicleLoad = preferences_->getString("wp_load", "");
  last_.telemetrySource = preferences_->getString("wp_tele", "LOCAL_GPS");
  last_.revision = preferences_->getULong("wp_rev", 0);
}
