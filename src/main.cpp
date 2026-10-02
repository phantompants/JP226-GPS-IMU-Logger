#include <Arduino.h>
#if JP226_CORE2
#include <M5Unified.h>
#else
#include <M5Cardputer.h>
#endif
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstring>

#include "Config.h"
#include "DinoSprite.h"
#include "KmlExporter.h"
#include "LocationTime.h"
#include "LogSchedule.h"
#include "PlaceResolver.h"
#include "RemoteTelemetryReceiver.h"
#include "TelemetryData.h"
#include "WaypointStore.h"
#include "WebPortal.h"
#include "WifiSetupPage.h"

namespace {

constexpr char kCsvHeader[] =
    "timestamp,lat,lon,alt_m,speed_kmh,heading_deg,satellites,hdop,vdop,"
    "acc_x_g,acc_y_g,acc_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,pitch_deg,"
    "roll_deg,g_total,roughness_index,local_timestamp,fix_valid,fix_age_ms,"
    "imu_available,imu_type,log_state,uptime_ms,telemetry_source,"
    "remote_sequence,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
    "accel_rms_mps2,vertical_accel_rms_mps2,"
    "vertical_accel_peak_pos_mps2,vertical_accel_peak_neg_mps2,"
    "lateral_accel_peak_mps2,longitudinal_accel_peak_mps2,"
    "vibration_rms_mps2,imu_samples,gps_age_ms,packet_age_ms,"
    "packets_lost,duplicate_packets,crc_errors,remote_tx_failures,"
    "poi,poi_source,auto_place,waypoint_id";

constexpr float kStandardGravityMps2 = 9.80665f;

enum class LogMode : uint8_t {
  WaitingForFix = 0,
  Moving,
  StoppedFirstHour,
  StoppedHourly,
  FixLost,
};

enum class GpsPreference : uint8_t {
  Auto = 0,
  Grove = 1,
  Cap = 2,
};

enum class DashboardPage : uint8_t {
  Combined = 0,
  Speed,
  HudSpeed,
  GpsStatus,
  ImuStatus,
  GpsSource,
  ImuSource,
  Wifi,
  Logger,
  TimeNetwork,
  KmlExport,
  Waypoint,
  About,
  Count,
};

enum class WaypointEditor : uint8_t { Closed, Name, Note, Photo };

class ImuSampler {
 public:
  void begin() {
    available_ = M5.Imu.getType() != m5::imu_none;
    typeName_ = imuTypeName(M5.Imu.getType());
  }

  void update(uint32_t nowMs) {
    if (!available_ || nowMs - lastSampleMs_ < config::kImuSampleIntervalMs) {
      return;
    }
    lastSampleMs_ = nowMs;
    if (!M5.Imu.update()) {
      return;
    }

    const auto data = M5.Imu.getImuData();
    latest_.available = true;
    latest_.valid = true;
    latest_.axMps2 = data.accel.x * kStandardGravityMps2;
    latest_.ayMps2 = data.accel.y * kStandardGravityMps2;
    latest_.azMps2 = data.accel.z * kStandardGravityMps2;
    latest_.gxDps = data.gyro.x;
    latest_.gyDps = data.gyro.y;
    latest_.gzDps = data.gyro.z;
    latest_.gTotalMps2 =
        std::sqrt(latest_.axMps2 * latest_.axMps2 +
                  latest_.ayMps2 * latest_.ayMps2 +
                  latest_.azMps2 * latest_.azMps2);
    latest_.pitchDeg =
        std::atan2(-latest_.axMps2,
                   std::sqrt(latest_.ayMps2 * latest_.ayMps2 +
                             latest_.azMps2 * latest_.azMps2)) *
        180.0f / PI;
    latest_.rollDeg =
        std::atan2(latest_.ayMps2, latest_.azMps2) * 180.0f / PI;

    if (!gravityFilterReady_) {
      gravityMagnitude_ = latest_.gTotalMps2;
      gravityFilterReady_ = true;
    } else {
      constexpr float alpha = 0.02f;
      gravityMagnitude_ += alpha * (latest_.gTotalMps2 - gravityMagnitude_);
    }
    const float vertical = latest_.azMps2 - gravityMagnitude_;
    const float magnitudeHighPass = latest_.gTotalMps2 - gravityMagnitude_;
    sumMagnitudeHighPassSquares_ +=
        static_cast<double>(magnitudeHighPass) * magnitudeHighPass;
    const float dynamicMagnitude =
        std::sqrt(latest_.axMps2 * latest_.axMps2 +
                  latest_.ayMps2 * latest_.ayMps2 + vertical * vertical);
    sumAccelSquares_ += static_cast<double>(dynamicMagnitude) * dynamicMagnitude;
    sumVerticalSquares_ += static_cast<double>(vertical) * vertical;
    verticalPeakPos_ = std::max(verticalPeakPos_, vertical);
    verticalPeakNeg_ = std::min(verticalPeakNeg_, vertical);
    lateralPeak_ = std::max(lateralPeak_, std::fabs(latest_.ayMps2));
    longitudinalPeak_ = std::max(longitudinalPeak_, std::fabs(latest_.axMps2));
    ++statisticsCount_;
  }

  ImuSample snapshotAndResetRoughness() {
    ImuSample result = latest_;
    result.available = available_;
    if (available_ && statisticsCount_ > 0) {
      result.accelRmsMps2 =
          std::sqrt(sumAccelSquares_ / static_cast<double>(statisticsCount_));
      result.verticalAccelRmsMps2 = std::sqrt(
          sumVerticalSquares_ / static_cast<double>(statisticsCount_));
      result.verticalAccelPeakPosMps2 = verticalPeakPos_;
      result.verticalAccelPeakNegMps2 = verticalPeakNeg_;
      result.lateralAccelPeakMps2 = lateralPeak_;
      result.longitudinalAccelPeakMps2 = longitudinalPeak_;
      result.vibrationRmsMps2 = result.verticalAccelRmsMps2;
      result.legacyRoughnessMps2 = std::sqrt(
          sumMagnitudeHighPassSquares_ /
          static_cast<double>(statisticsCount_));
      result.sampleCount = static_cast<uint16_t>(
          std::min<uint32_t>(statisticsCount_, UINT16_MAX));
    }
    sumAccelSquares_ = 0.0;
    sumVerticalSquares_ = 0.0;
    sumMagnitudeHighPassSquares_ = 0.0;
    verticalPeakPos_ = 0.0f;
    verticalPeakNeg_ = 0.0f;
    lateralPeak_ = 0.0f;
    longitudinalPeak_ = 0.0f;
    statisticsCount_ = 0;
    return result;
  }

  bool available() const { return available_; }
  const char* typeName() const { return typeName_; }
  const ImuSample& latest() const { return latest_; }

 private:
  static const char* imuTypeName(m5::imu_t type) {
    switch (type) {
      case m5::imu_bmi270:
        return "BMI270";
      case m5::imu_mpu6886:
        return "MPU6886";
      case m5::imu_mpu6050:
        return "MPU6050";
      case m5::imu_mpu9250:
        return "MPU9250";
      case m5::imu_sh200q:
        return "SH200Q";
      case m5::imu_none:
        return "none";
      default:
        return "unknown";
    }
  }

  bool available_ = false;
  bool gravityFilterReady_ = false;
  const char* typeName_ = "none";
  uint32_t lastSampleMs_ = 0;
  float gravityMagnitude_ = kStandardGravityMps2;
  double sumAccelSquares_ = 0.0;
  double sumVerticalSquares_ = 0.0;
  double sumMagnitudeHighPassSquares_ = 0.0;
  float verticalPeakPos_ = 0.0f;
  float verticalPeakNeg_ = 0.0f;
  float lateralPeak_ = 0.0f;
  float longitudinalPeak_ = 0.0f;
  uint32_t statisticsCount_ = 0;
  ImuSample latest_;
};

class CsvLineBuilder {
 public:
  CsvLineBuilder() { data_[0] = '\0'; }

  void append(const char* format, ...) {
    if (length_ >= sizeof(data_) - 1) {
      return;
    }
    va_list args;
    va_start(args, format);
    const int count = vsnprintf(data_ + length_, sizeof(data_) - length_, format, args);
    va_end(args);
    if (count <= 0) {
      return;
    }
    const size_t written = static_cast<size_t>(count);
    length_ += written < sizeof(data_) - length_ ? written : sizeof(data_) - length_ - 1;
  }

  void field(double value, int decimals, bool valid = true) {
    if (valid && std::isfinite(value)) {
      append(",%.*f", decimals, value);
    } else {
      append(",");
    }
  }

  const char* c_str() const { return data_; }
  size_t length() const { return length_; }

 private:
  char data_[768]{};
  size_t length_ = 0;
};

class GpsReceiver {
 public:
  GpsReceiver(uint8_t uartNumber, const char* name, int rxPin, int txPin)
      : serial_(uartNumber),
        name_(name),
        rxPin_(rxPin),
        txPin_(txPin),
        gngsaVdop_(parser_, "GNGSA", 17),
        gpgsaVdop_(parser_, "GPGSA", 17) {}

  // Takes effect at the next begin().
  void setPins(int rxPin, int txPin) {
    rxPin_ = rxPin;
    txPin_ = txPin;
  }

  void begin(uint32_t baud) {
    if (!started_) {
      serial_.setRxBufferSize(2048);
      started_ = true;
    } else {
      serial_.end();
    }
    baud_ = baud;
    lastSentenceMs_ = 0;
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
  }

  void poll(uint32_t nowMs) {
    if (!started_) return;
    const uint32_t checksumCount = parser_.passedChecksum();
    while (serial_.available() > 0) {
      parser_.encode(static_cast<char>(serial_.read()));
    }
    if (parser_.passedChecksum() != checksumCount) {
      lastSentenceMs_ = nowMs;
      detectedEver_ = true;
    }
  }

  bool live(uint32_t nowMs) const {
    return detectedEver_ && lastSentenceMs_ > 0 &&
           nowMs - lastSentenceMs_ <= config::kGpsSourceStaleMs;
  }

  bool started() const { return started_; }
  const char* name() const { return name_; }
  uint32_t baud() const { return baud_; }
  int rxPin() const { return rxPin_; }
  int txPin() const { return txPin_; }
  TinyGPSPlus& parser() { return parser_; }
  TinyGPSCustom& gngsaVdop() { return gngsaVdop_; }
  TinyGPSCustom& gpgsaVdop() { return gpgsaVdop_; }

 private:
  HardwareSerial serial_;
  const char* name_;
  int rxPin_;
  int txPin_;
  uint32_t baud_ = 0;
  uint32_t lastSentenceMs_ = 0;
  bool started_ = false;
  bool detectedEver_ = false;
  TinyGPSPlus parser_;
  TinyGPSCustom gngsaVdop_;
  TinyGPSCustom gpgsaVdop_;
};

GpsReceiver groveGps(1, "GROVE", config::kGroveGpsRxPin,
                     config::kGroveGpsTxPin);
GpsReceiver capGps(2, "CAP", config::kCapGpsRxPin, config::kCapGpsTxPin);
GpsReceiver* activeGps = &groveGps;
#if JP226_CORE2
// The Core2 microSD card shares the display's SPI bus, so the bus is never
// stopped; only the SD driver is restarted on a retry.
SPIClass& sdSpi = SPI;
#else
SPIClass sdSpi(FSPI);
#endif
Preferences preferences;
ImuSampler imu;
RemoteTelemetryReceiver remoteReceiver;
WaypointStore waypointStore;
PlaceResolver placeResolver;
VehicleContext vehicleContext;
LocationTime locationTime;
KmlExporter kmlExporter;
WebPortal webPortal;
#if JP226_CORE2
// No keyboard for the network list and password entry: the Core2 takes its
// Wi-Fi credentials from logger.cfg on the SD card.
struct NoWifiSetup {
  bool active() const { return false; }
  void update(uint32_t) {}
  void draw(uint32_t) {}
};
NoWifiSetup wifiSetup;
#else
WifiSetupPage wifiSetup;
#endif
// M5.Display, not M5Cardputer.Display: the latter is a reference member that
// may not be bound yet when this global is constructed.
M5Canvas dashboardCanvas(&M5.Display);
File logFile;
String currentLogPath;
String lastKmlScanDate;
#if ENABLE_RAW_IMU_LOGGING
File rawImuFile;
String currentRawImuPath;
String rawImuBuffer;
uint16_t rawImuBufferedRows = 0;
#endif

bool sdMounted = false;
bool locationTimeStarted = false;
bool placeResolverStarted = false;
bool screenOn = true;
bool immediateLogRequested = false;
bool stopCandidateActive = false;
bool haveSeenValidFix = false;
bool persistedStopPending = false;
uint32_t stopCandidateMs = 0;
time_t stopCandidateUtc = 0;
uint32_t lastValidFixMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastFixLostLogMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastSdAttemptMs = 0;
uint32_t lastClockSyncMs = 0;
uint32_t lastRtcWriteMs = 0;
uint64_t rowsWritten = 0;
uint32_t tripStartedMs = 0;
uint8_t displayBrightness = 128;
size_t groveBaudIndex = 0;
uint32_t groveBaudStartedMs = 0;
bool grovePinsSwapped = false;
time_t stopStartUtc = 0;
time_t nextStoppedDueUtc = 0;
time_t persistedStopStartUtc = 0;
time_t persistedNextDueUtc = 0;
LogMode mode = LogMode::WaitingForFix;
GpsPreference gpsPreference = GpsPreference::Auto;
DashboardPage dashboardPage = DashboardPage::Combined;
TelemetrySource telemetrySource = TelemetrySource::LocalGps;
NormalizedTelemetry activeTelemetry;
bool dashboardCanvasReady = false;
bool haveDashboardFrameHash = false;
uint32_t lastDashboardFrameHash = 0;
WaypointEditor waypointEditor = WaypointEditor::Closed;
String waypointInput;
String waypointMessage;
uint32_t waypointMessageStartedMs = 0;
bool waypointShowLast = false;
String autoPlace;
int8_t batteryPercent = telemetry::kBatteryUnknown;
uint16_t batteryMv = 0;
bool batteryCharging = false;
uint32_t lastBatteryMs = 0;
uint32_t lastBeaconInfoMs = 0;

uint32_t frameBufferHash(const M5Canvas& canvas) {
  const auto* bytes = static_cast<const uint8_t*>(canvas.getBuffer());
  const size_t length = canvas.bufferLength();
  uint32_t hash = 2166136261U;
  for (size_t index = 0; index < length; ++index) {
    hash ^= bytes[index];
    hash *= 16777619U;
  }
  return hash;
}

int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
  const unsigned dayOfYear =
      (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 +
                            dayOfYear;
  return era * 146097LL + static_cast<int64_t>(dayOfEra) - 719468LL;
}

time_t gpsUtcEpoch(TinyGPSPlus& gps) {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return 0;
  }
  const int year = gps.date.year();
  const unsigned month = gps.date.month();
  const unsigned day = gps.date.day();
  if (year < 2024 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31) {
    return 0;
  }
  const int64_t seconds =
      daysFromCivil(year, month, day) * 86400LL + gps.time.hour() * 3600LL +
      gps.time.minute() * 60LL + gps.time.second();
  return static_cast<time_t>(seconds);
}

bool clockIsReady() {
  const time_t now = time(nullptr);
  struct tm utc {};
  return now > 0 && gmtime_r(&now, &utc) != nullptr && utc.tm_year + 1900 >= 2024;
}

void syncClockFromTelemetry(const GpsSnapshot& gps, uint32_t nowMs) {
  if (!gps.utcValid || gps.utcEpochMs == 0 ||
      (clockIsReady() && nowMs - lastClockSyncMs < config::kClockResyncIntervalMs)) {
    return;
  }
  const time_t epoch = static_cast<time_t>(gps.utcEpochMs / 1000ULL);
  if (epoch <= 0) {
    return;
  }
  timeval value{epoch,
                static_cast<suseconds_t>((gps.utcEpochMs % 1000ULL) * 1000ULL)};
  settimeofday(&value, nullptr);
  lastClockSyncMs = nowMs;

  if (M5.Rtc.isEnabled()) {
    struct tm utc {};
    gmtime_r(&epoch, &utc);
    M5.Rtc.setDateTime(&utc);
  }
}

// Boards with a battery-backed RTC (the Core2) keep UTC across power-off, so
// logging can resume before the GPS has a fix.
void restoreClockFromRtc() {
  if (!M5.Rtc.isEnabled()) return;
  const auto now = M5.Rtc.getDateTime();
  if (now.date.year < 2024 || now.date.year > 2099 || now.date.month < 1 ||
      now.date.month > 12 || now.date.date < 1 || now.date.date > 31) {
    return;
  }
  const int64_t seconds =
      daysFromCivil(now.date.year, now.date.month, now.date.date) * 86400LL +
      now.time.hours * 3600LL + now.time.minutes * 60LL + now.time.seconds;
  timeval value{static_cast<time_t>(seconds), 0};
  settimeofday(&value, nullptr);
}

void syncRtcFromSystem(uint32_t nowMs) {
  constexpr uint32_t kRtcWriteIntervalMs = 60 * 60 * 1000UL;
  if (!M5.Rtc.isEnabled() || !clockIsReady() ||
      (lastRtcWriteMs > 0 && nowMs - lastRtcWriteMs < kRtcWriteIntervalMs)) {
    return;
  }
  const time_t nowUtc = time(nullptr);
  struct tm utc {};
  if (gmtime_r(&nowUtc, &utc) != nullptr) {
    M5.Rtc.setDateTime(&utc);
    lastRtcWriteMs = nowMs;
  }
}

bool parseCustomFloat(TinyGPSCustom& custom, double& result) {
  const char* value = custom.value();
  if (value == nullptr || value[0] == '\0') {
    return false;
  }
  char* end = nullptr;
  const double parsed = strtod(value, &end);
  if (end == value || !std::isfinite(parsed)) {
    return false;
  }
  result = parsed;
  return true;
}

GpsSnapshot takeGpsSnapshot(GpsReceiver& receiver) {
  TinyGPSPlus& gps = receiver.parser();
  GpsSnapshot sample;
  sample.positionFresh = gps.location.isValid() && gps.location.age() <= config::kMaxFixAgeMs;
  sample.speedFresh = gps.speed.isValid() && gps.speed.age() <= config::kMaxFixAgeMs;
  sample.altitudeFresh = gps.altitude.isValid() && gps.altitude.age() <= config::kMaxFixAgeMs;
  sample.courseFresh = gps.course.isValid() && gps.course.age() <= config::kMaxFixAgeMs;
  sample.satellitesValid = gps.satellites.isValid();
  sample.hdopValid = gps.hdop.isValid();
  sample.fixAgeMs = gps.location.isValid() ? gps.location.age() : UINT32_MAX;

  if (sample.positionFresh) {
    sample.latitude = gps.location.lat();
    sample.longitude = gps.location.lng();
  }
  if (sample.speedFresh) sample.speedKmh = gps.speed.kmph();
  if (sample.altitudeFresh) sample.altitudeM = gps.altitude.meters();
  if (sample.courseFresh) sample.courseDeg = gps.course.deg();
  if (sample.satellitesValid) sample.satellites = gps.satellites.value();
  if (sample.hdopValid) sample.hdop = gps.hdop.hdop();
  sample.vdopValid = parseCustomFloat(receiver.gngsaVdop(), sample.vdop) ||
                     parseCustomFloat(receiver.gpgsaVdop(), sample.vdop);

  const bool dateTimeFresh = gps.date.isValid() && gps.time.isValid() &&
                             gps.date.age() <= config::kMaxDateTimeAgeMs &&
                             gps.time.age() <= config::kMaxDateTimeAgeMs;
  sample.utcValid = dateTimeFresh;
  if (dateTimeFresh) {
    sample.utcEpochMs =
        static_cast<uint64_t>(gpsUtcEpoch(gps)) * 1000ULL +
        static_cast<uint64_t>(gps.time.centisecond()) * 10ULL;
  }
  sample.fixValid = sample.positionFresh && sample.speedFresh && dateTimeFresh &&
                    sample.satellitesValid &&
                    sample.satellites >= config::kMinimumSatellites &&
                    sample.hdopValid && sample.hdop <= config::kMaximumHdop;
  return sample;
}

const char* telemetrySourceName(TelemetrySource source) {
  switch (source) {
    case TelemetrySource::AtomS3Remote:
      return "ATOMS3_REMOTE";
    case TelemetrySource::CardputerAdv:
      return "CARDPUTER_ADV";
    case TelemetrySource::LocalGpsAtomImu:
      return "LOCAL_GPS_ATOM_IMU";
    default:
      return "LOCAL_GPS";
  }
}

// Short on-screen name for the selected source.
const char* telemetrySourceLabel(TelemetrySource source) {
  switch (source) {
    case TelemetrySource::AtomS3Remote:
      return "ATOM REMOTE";
    case TelemetrySource::LocalGpsAtomImu:
      return "GPS+ATOM IMU";
    default:
      return "LOCAL GPS";
  }
}

NormalizedTelemetry takeLocalTelemetry() {
  NormalizedTelemetry result;
  result.gps = takeGpsSnapshot(*activeGps);
  result.imu = imu.latest();
  result.metadata.source = TelemetrySource::LocalGps;
  return result;
}

NormalizedTelemetry takeActiveTelemetry(uint32_t nowMs) {
  switch (telemetrySource) {
    case TelemetrySource::AtomS3Remote:
      return remoteReceiver.snapshot(nowMs);
    case TelemetrySource::LocalGpsAtomImu: {
      NormalizedTelemetry result = takeLocalTelemetry();
      const NormalizedTelemetry remote = remoteReceiver.snapshot(nowMs);
      result.imu = remote.imu;
      result.metadata = remote.metadata;
      result.metadata.source = TelemetrySource::LocalGpsAtomImu;
      return result;
    }
    default:
      return takeLocalTelemetry();
  }
}

void currentPoi(const GpsSnapshot& gps, String& poi, String& source,
                String& waypointId) {
  poi = "";
  source = "NONE";
  waypointId = "";
  if (!gps.positionFresh) return;
  if (waypointStore.hasLast() &&
      waypointStore.distanceFromLastM(gps.latitude, gps.longitude) <= 2000.0) {
    const WaypointRecord& last = waypointStore.last();
    poi = last.name;
    source = last.source;
    waypointId = last.id;
    if (source == "PHOTO_REFERENCE" && !last.photoReference.isEmpty()) {
      poi = last.photoReference;
    }
  } else if (!autoPlace.isEmpty()) {
    poi = autoPlace;
    source = "AUTO_PLACE";
  }
}

const char* modeName(LogMode value) {
  switch (value) {
    case LogMode::Moving:
      return "MOVING";
    case LogMode::StoppedFirstHour:
      return "STOPPED_15M";
    case LogMode::StoppedHourly:
      return "STOPPED_HOURLY";
    case LogMode::FixLost:
      return "FIX_LOST";
    default:
      return "WAITING_FIX";
  }
}

void setStoppedPhase(time_t nowUtc) {
  if (stopStartUtc > 0 && nowUtc - stopStartUtc >=
                              static_cast<time_t>(config::kStoppedHourlyIntervalSec)) {
    mode = LogMode::StoppedHourly;
  } else {
    mode = LogMode::StoppedFirstHour;
  }
}

void saveStoppedState() {
  preferences.putUChar("state", 2);
  preferences.putULong64("stop_utc", static_cast<uint64_t>(stopStartUtc));
  preferences.putULong64("next_utc", static_cast<uint64_t>(nextStoppedDueUtc));
}

void clearStoppedState() {
  preferences.putUChar("state", 1);
  preferences.putULong64("stop_utc", 0);
  preferences.putULong64("next_utc", 0);
  stopStartUtc = 0;
  nextStoppedDueUtc = 0;
  persistedStopPending = false;
}

void enterMoving() {
  const bool changed = mode != LogMode::Moving;
  if (changed) {
    mode = LogMode::Moving;
    immediateLogRequested = true;
  }
  stopCandidateActive = false;
  if (changed || stopStartUtc != 0 || nextStoppedDueUtc != 0 ||
      persistedStopPending) {
    clearStoppedState();
  }
}

void enterStopped(time_t detectedStopUtc, time_t nowUtc, bool allowPersisted) {
  if (allowPersisted && persistedStopPending &&
      schedule::persistedStopIsPlausible(
          persistedStopStartUtc, persistedNextDueUtc, nowUtc,
          config::kPersistedStopMaxAgeSec,
          config::kStoppedHourlyIntervalSec)) {
    stopStartUtc = persistedStopStartUtc;
    nextStoppedDueUtc = persistedNextDueUtc;
  } else {
    stopStartUtc = detectedStopUtc > 0 ? detectedStopUtc : nowUtc;
    nextStoppedDueUtc = static_cast<time_t>(schedule::nextStoppedDue(
        stopStartUtc, nowUtc, config::kStoppedFirstHourIntervalSec,
        config::kStoppedHourlyIntervalSec));
  }
  persistedStopPending = false;
  setStoppedPhase(nowUtc);
  stopCandidateActive = false;
  immediateLogRequested = true;
  saveStoppedState();
}

void updateLoggingMode(const GpsSnapshot& sample, uint32_t nowMs, time_t nowUtc) {
  if (sample.fixValid) {
    haveSeenValidFix = true;
    lastValidFixMs = nowMs;

    if (sample.speedKmh >= config::kMoveStartKmh) {
      enterMoving();
      return;
    }

    if (mode == LogMode::Moving) {
      if (sample.speedKmh <= config::kMoveStopKmh) {
        if (!stopCandidateActive) {
          stopCandidateActive = true;
          stopCandidateMs = nowMs;
          stopCandidateUtc = nowUtc;
        } else if (nowMs - stopCandidateMs >= config::kStopConfirmMs) {
          enterStopped(stopCandidateUtc, nowUtc, false);
        }
      } else {
        stopCandidateActive = false;
      }
      return;
    }

    if (mode == LogMode::StoppedFirstHour || mode == LogMode::StoppedHourly) {
      setStoppedPhase(nowUtc);
      return;
    }

    if (sample.speedKmh <= config::kMoveStopKmh) {
      enterStopped(nowUtc, nowUtc, true);
    }
    return;
  }

  const bool timedOut = !haveSeenValidFix
                            ? nowMs >= config::kFixLossTimeoutMs
                            : nowMs - lastValidFixMs >= config::kFixLossTimeoutMs;
  if (timedOut && mode != LogMode::FixLost) {
    mode = LogMode::FixLost;
    stopCandidateActive = false;
    immediateLogRequested = true;
  }
}

void formatUtcTimestamp(char* output, size_t outputSize) {
  timeval value {};
  gettimeofday(&value, nullptr);
  struct tm utc {};
  gmtime_r(&value.tv_sec, &utc);
  snprintf(output, outputSize, "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
           utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour,
           utc.tm_min, utc.tm_sec, static_cast<long>(value.tv_usec / 1000));
}

void formatLocalTimestamp(char* output, size_t outputSize) {
  timeval value {};
  gettimeofday(&value, nullptr);
  struct tm local {};
  localtime_r(&value.tv_sec, &local);
  char offset[8]{};
  strftime(offset, sizeof(offset), "%z", &local);
  char offsetWithColon[8]{};
  if (strlen(offset) == 5) {
    snprintf(offsetWithColon, sizeof(offsetWithColon), "%c%c%c:%c%c", offset[0],
             offset[1], offset[2], offset[3], offset[4]);
  } else {
    strncpy(offsetWithColon, offset, sizeof(offsetWithColon) - 1);
  }
  snprintf(output, outputSize, "%04d-%02d-%02dT%02d:%02d:%02d.%03ld%s",
           local.tm_year + 1900, local.tm_mon + 1, local.tm_mday, local.tm_hour,
           local.tm_min, local.tm_sec, static_cast<long>(value.tv_usec / 1000),
           offsetWithColon);
}

String pathForLocalDay(time_t nowUtc) {
  struct tm local {};
  localtime_r(&nowUtc, &local);
  char path[64]{};
  snprintf(path, sizeof(path), "%s/%s%04d-%02d-%02d.csv", config::kLogDirectory,
           config::kFilePrefix, local.tm_year + 1900, local.tm_mon + 1,
           local.tm_mday);
  return String(path);
}

String localDateText(time_t nowUtc) {
  struct tm local {};
  localtime_r(&nowUtc, &local);
  char date[16]{};
  snprintf(date, sizeof(date), "%04d-%02d-%02d", local.tm_year + 1900,
           local.tm_mon + 1, local.tm_mday);
  return String(date);
}

void markSdFailed(uint32_t nowMs);
bool mountSd(uint32_t nowMs, bool force = false);

#if ENABLE_RAW_IMU_LOGGING
String rawImuPathForLocalDay(time_t nowUtc) {
  struct tm local {};
  localtime_r(&nowUtc, &local);
  char path[64]{};
  snprintf(path, sizeof(path), "%s/imu_%04d-%02d-%02d.csv",
           config::kLogDirectory, local.tm_year + 1900, local.tm_mon + 1,
           local.tm_mday);
  return String(path);
}

bool openRawImuFile(time_t nowUtc, uint32_t nowMs) {
  if (!mountSd(nowMs)) return false;
  const String requiredPath = rawImuPathForLocalDay(nowUtc);
  if (rawImuFile && currentRawImuPath == requiredPath) return true;
  if (rawImuFile) {
    if (!rawImuBuffer.isEmpty()) rawImuFile.print(rawImuBuffer);
    rawImuFile.flush();
    rawImuFile.close();
  }
  rawImuBuffer = "";
  rawImuBufferedRows = 0;
  currentRawImuPath = requiredPath;
  rawImuFile = SD.open(currentRawImuPath, FILE_APPEND);
  if (!rawImuFile) {
    markSdFailed(nowMs);
    return false;
  }
  if (rawImuFile.size() == 0) {
    rawImuFile.println(
        "received_timestamp,node_batch_ms,offset_100us,accel_x_mps2,"
        "accel_y_mps2,accel_z_mps2,gyro_x_dps,gyro_y_dps,gyro_z_dps");
    rawImuFile.flush();
  }
  return true;
}

void writeRawImuBatches(time_t nowUtc, uint32_t nowMs) {
  telemetry::RawImuBatchPacket batch{};
  while (remoteReceiver.popRawImuBatch(batch)) {
    if (!clockIsReady() || !openRawImuFile(nowUtc, nowMs)) continue;
    char receivedTimestamp[40]{};
    formatUtcTimestamp(receivedTimestamp, sizeof(receivedTimestamp));
    const uint8_t count = std::min<uint8_t>(
        batch.sample_count, telemetry::kRawImuSamplesPerPacket);
    for (uint8_t index = 0; index < count; ++index) {
      const auto& sample = batch.samples[index];
      char row[256]{};
      snprintf(row, sizeof(row),
               "%s,%lu,%u,%.5f,%.5f,%.5f,%.1f,%.1f,%.1f\n",
               receivedTimestamp,
               static_cast<unsigned long>(batch.batch_start_time_ms),
               sample.time_offset_100us,
               sample.accel_x_mg * kStandardGravityMps2 / 1000.0f,
               sample.accel_y_mg * kStandardGravityMps2 / 1000.0f,
               sample.accel_z_mg * kStandardGravityMps2 / 1000.0f,
               sample.gyro_x_deci_dps / 10.0f,
               sample.gyro_y_deci_dps / 10.0f,
               sample.gyro_z_deci_dps / 10.0f);
      rawImuBuffer += row;
      ++rawImuBufferedRows;
      if (rawImuBufferedRows >= 32) {
        if (rawImuFile.print(rawImuBuffer) == 0) {
          markSdFailed(nowMs);
          return;
        }
        rawImuFile.flush();
        rawImuBuffer = "";
        rawImuBufferedRows = 0;
      }
    }
  }
}
#endif

void closeCompletedDailyFile(time_t nowUtc) {
  if (currentLogPath.isEmpty() || currentLogPath == pathForLocalDay(nowUtc)) {
    return;
  }
  const String completedPath = currentLogPath;
  if (logFile) logFile.close();
  currentLogPath = "";
  kmlExporter.queueCompletedFile(completedPath);
}

void markSdFailed(uint32_t nowMs) {
  if (logFile) logFile.close();
#if ENABLE_RAW_IMU_LOGGING
  if (rawImuFile) rawImuFile.close();
  currentRawImuPath = "";
  rawImuBuffer = "";
  rawImuBufferedRows = 0;
#endif
  currentLogPath = "";
  sdMounted = false;
  lastSdAttemptMs = nowMs;
  SD.end();
}

bool mountSd(uint32_t nowMs, bool force) {
  if (sdMounted) return true;
  if (!force && nowMs - lastSdAttemptMs < config::kSdRetryIntervalMs) return false;
  lastSdAttemptMs = nowMs;
  SD.end();
#if !JP226_CORE2
  sdSpi.end();
#endif
  sdSpi.begin(config::kSdSckPin, config::kSdMisoPin, config::kSdMosiPin,
              config::kSdCsPin);
  sdMounted = SD.begin(config::kSdCsPin, sdSpi, config::kSdFrequencyHz) &&
              SD.cardType() != CARD_NONE;
  if (sdMounted && !SD.exists(config::kLogDirectory)) {
    sdMounted = SD.mkdir(config::kLogDirectory);
  }
  return sdMounted;
}

bool openDailyFile(time_t nowUtc, uint32_t nowMs) {
  if (!mountSd(nowMs)) return false;
  const String requiredPath = pathForLocalDay(nowUtc);
  if (logFile && currentLogPath == requiredPath) return true;
  if (logFile) logFile.close();
  if (!currentLogPath.isEmpty()) {
    kmlExporter.queueCompletedFile(currentLogPath);
  }
  currentLogPath = requiredPath;
  // A timezone change can legitimately reopen an earlier local-date CSV.
  // Remove any older export so it cannot remain stale while rows are appended.
  kmlExporter.invalidateForCsv(currentLogPath);
  logFile = SD.open(currentLogPath, FILE_APPEND);
  if (!logFile) {
    markSdFailed(nowMs);
    return false;
  }
  if (logFile.size() == 0) {
    if (logFile.println(kCsvHeader) == 0) {
      markSdFailed(nowMs);
      return false;
    }
    logFile.flush();
  }
  return true;
}

bool writeCsvRow(const NormalizedTelemetry& telemetryData, uint32_t nowMs,
                 time_t nowUtc) {
  if (!clockIsReady() || !openDailyFile(nowUtc, nowMs)) return false;

  const GpsSnapshot& gpsSample = telemetryData.gps;
  const bool remoteImu = usesRemoteImu(telemetryData.metadata.source);
  const ImuSample imuSample =
      remoteImu ? telemetryData.imu : imu.snapshotAndResetRoughness();
  char utcTimestamp[40]{};
  char localTimestamp[48]{};
  formatUtcTimestamp(utcTimestamp, sizeof(utcTimestamp));
  formatLocalTimestamp(localTimestamp, sizeof(localTimestamp));

  CsvLineBuilder line;
  line.append("%s", utcTimestamp);
  line.field(gpsSample.latitude, 6, gpsSample.positionFresh);
  line.field(gpsSample.longitude, 6, gpsSample.positionFresh);
  line.field(gpsSample.altitudeM, 1, gpsSample.altitudeFresh);
  line.field(gpsSample.speedKmh, 2, gpsSample.speedFresh);
  line.field(gpsSample.courseDeg, 1, gpsSample.courseFresh);
  if (gpsSample.satellitesValid) {
    line.append(",%lu", static_cast<unsigned long>(gpsSample.satellites));
  } else {
    line.append(",");
  }
  line.field(gpsSample.hdop, 2, gpsSample.hdopValid);
  line.field(gpsSample.vdop, 2, gpsSample.vdopValid);
  line.field(imuSample.axMps2 / kStandardGravityMps2, 4, imuSample.valid);
  line.field(imuSample.ayMps2 / kStandardGravityMps2, 4, imuSample.valid);
  line.field(imuSample.azMps2 / kStandardGravityMps2, 4, imuSample.valid);
  line.field(imuSample.gxDps, 3, imuSample.valid);
  line.field(imuSample.gyDps, 3, imuSample.valid);
  line.field(imuSample.gzDps, 3, imuSample.valid);
  line.field(imuSample.pitchDeg, 2, imuSample.valid);
  line.field(imuSample.rollDeg, 2, imuSample.valid);
  line.field(imuSample.gTotalMps2 / kStandardGravityMps2, 4,
             imuSample.valid);
  const float roughnessMps2 = remoteImu ? imuSample.vibrationRmsMps2
                                        : imuSample.legacyRoughnessMps2;
  line.field(roughnessMps2 / kStandardGravityMps2, 5, imuSample.valid);
  line.append(",%s,%u,", localTimestamp, gpsSample.fixValid ? 1U : 0U);
  if (gpsSample.fixAgeMs != UINT32_MAX) {
    line.append("%lu", static_cast<unsigned long>(gpsSample.fixAgeMs));
  }
  const char* imuType = remoteImu ? "MPU6886_REMOTE" : imu.typeName();
  line.append(",%u,%s,%s,%llu", imuSample.available ? 1U : 0U, imuType,
              modeName(mode),
              static_cast<unsigned long long>(esp_timer_get_time() / 1000ULL));
  line.append(",%s,", telemetrySourceName(telemetryData.metadata.source));
  if (remoteImu) {
    line.append("%lu", static_cast<unsigned long>(
                           telemetryData.metadata.remoteSequence));
  }
  line.field(imuSample.axMps2, 4, imuSample.valid);
  line.field(imuSample.ayMps2, 4, imuSample.valid);
  line.field(imuSample.azMps2, 4, imuSample.valid);
  line.field(imuSample.accelRmsMps2, 4, imuSample.valid);
  line.field(imuSample.verticalAccelRmsMps2, 4, imuSample.valid);
  line.field(imuSample.verticalAccelPeakPosMps2, 4, imuSample.valid);
  line.field(imuSample.verticalAccelPeakNegMps2, 4, imuSample.valid);
  line.field(imuSample.lateralAccelPeakMps2, 4, imuSample.valid);
  line.field(imuSample.longitudinalAccelPeakMps2, 4, imuSample.valid);
  line.field(imuSample.vibrationRmsMps2, 4, imuSample.valid);
  if (imuSample.valid) line.append(",%u", imuSample.sampleCount);
  else line.append(",");
  if (gpsSample.fixAgeMs != UINT32_MAX) {
    line.append(",%lu", static_cast<unsigned long>(gpsSample.fixAgeMs));
  } else {
    line.append(",");
  }
  if (telemetryData.metadata.packetAgeMs != UINT32_MAX) {
    line.append(",%lu", static_cast<unsigned long>(
                            telemetryData.metadata.packetAgeMs));
  } else {
    line.append(",");
  }
  if (remoteImu) {
    line.append(",%lu,%lu,%lu,%u",
                static_cast<unsigned long>(telemetryData.metadata.packetsLost),
                static_cast<unsigned long>(telemetryData.metadata.duplicates),
                static_cast<unsigned long>(telemetryData.metadata.crcErrors),
                telemetryData.metadata.remoteTransmitFailures);
  } else {
    line.append(",,,,");
  }
  String poi;
  String poiSource;
  String waypointId;
  currentPoi(gpsSample, poi, poiSource, waypointId);
  const String escapedPoi = WaypointStore::csvField(poi);
  const String escapedAutoPlace = WaypointStore::csvField(autoPlace);
  line.append(",%s,%s,%s,%s", escapedPoi.c_str(), poiSource.c_str(),
              escapedAutoPlace.c_str(), waypointId.c_str());

  const size_t written = logFile.println(line.c_str());
  logFile.flush();
  if (written <= line.length()) {
    markSdFailed(nowMs);
    return false;
  }
  ++rowsWritten;
  return true;
}

bool logIsDue(uint32_t nowMs, time_t nowUtc) {
  if (immediateLogRequested) return true;
  switch (mode) {
    case LogMode::Moving:
      return nowMs - lastLogMs >= config::kMovingLogIntervalMs;
    case LogMode::StoppedFirstHour:
    case LogMode::StoppedHourly:
      return nextStoppedDueUtc > 0 && nowUtc >= nextStoppedDueUtc;
    case LogMode::FixLost:
      return nowMs - lastFixLostLogMs >= config::kFixLostLogIntervalMs;
    default:
      return false;
  }
}

bool stationaryForFileWork() {
  return mode == LogMode::StoppedFirstHour || mode == LogMode::StoppedHourly;
}

void onLogSucceeded(uint32_t nowMs, time_t nowUtc) {
  immediateLogRequested = false;
  lastLogMs = nowMs;
  if (mode == LogMode::FixLost) {
    lastFixLostLogMs = nowMs;
  } else if (mode == LogMode::StoppedFirstHour || mode == LogMode::StoppedHourly) {
    nextStoppedDueUtc = static_cast<time_t>(schedule::nextStoppedDue(
        stopStartUtc, nowUtc, config::kStoppedFirstHourIntervalSec,
        config::kStoppedHourlyIntervalSec));
    setStoppedPhase(nowUtc);
    saveStoppedState();
  }
}

bool isCardputerAdv() {
  return M5.getBoard() == m5::board_t::board_M5CardputerADV;
}

const char* gpsPreferenceName() {
  switch (gpsPreference) {
    case GpsPreference::Grove:
      return "GROVE";
    case GpsPreference::Cap:
      return "CAP";
    default:
      return "AUTO";
  }
}

void selectGpsReceiver(uint32_t nowMs) {
  GpsReceiver* selected = activeGps;
  switch (gpsPreference) {
    case GpsPreference::Grove:
      selected = &groveGps;
      break;
    case GpsPreference::Cap:
      selected = capGps.started() ? &capGps : &groveGps;
      break;
    case GpsPreference::Auto:
      if (capGps.started() && capGps.live(nowMs)) {
        selected = &capGps;
      } else if (groveGps.live(nowMs)) {
        selected = &groveGps;
      } else if (selected == nullptr || !selected->started()) {
        selected = &groveGps;
      }
      break;
  }

  if (selected != activeGps) {
    activeGps = selected;
    Serial.printf("GPS source: %s at %lu baud (preference %s)\n",
                  activeGps->name(), static_cast<unsigned long>(activeGps->baud()),
                  gpsPreferenceName());
  }
}

void updateGpsReceivers(uint32_t nowMs) {
  groveGps.poll(nowMs);
  if (capGps.started()) capGps.poll(nowMs);

  if (!groveGps.live(nowMs) &&
      nowMs - groveBaudStartedMs >= config::kGpsBaudScanIntervalMs) {
    groveBaudIndex =
        (groveBaudIndex + 1) % config::kGroveGpsBaudCandidateCount;
    // After a full pass of baud rates, try the Grove lines the other way round.
    if (groveBaudIndex == 0 && config::kGroveGpsAltRxPin >= 0) {
      grovePinsSwapped = !grovePinsSwapped;
      groveGps.setPins(
          grovePinsSwapped ? config::kGroveGpsAltRxPin : config::kGroveGpsRxPin,
          grovePinsSwapped ? config::kGroveGpsAltTxPin : config::kGroveGpsTxPin);
    }
    groveGps.begin(config::kGroveGpsBaudCandidates[groveBaudIndex]);
    groveBaudStartedMs = nowMs;
    Serial.printf("Scanning Grove GPS at %lu baud\n",
                  static_cast<unsigned long>(groveGps.baud()));
  }

  selectGpsReceiver(nowMs);
}

void cycleGpsPreference() {
  if (isCardputerAdv()) {
    gpsPreference = static_cast<GpsPreference>(
        (static_cast<uint8_t>(gpsPreference) + 1U) % 3U);
  } else {
    gpsPreference = gpsPreference == GpsPreference::Auto
                        ? GpsPreference::Grove
                        : GpsPreference::Auto;
  }
  preferences.putUChar("gps_src", static_cast<uint8_t>(gpsPreference));
  selectGpsReceiver(millis());
  lastDisplayMs = 0;
}

void setTelemetrySource(TelemetrySource source) {
  telemetrySource = source;
  preferences.putUChar("tele_src", static_cast<uint8_t>(telemetrySource));
  immediateLogRequested = true;
  haveDashboardFrameHash = false;
  lastDisplayMs = 0;
}

void cycleTelemetrySource() {
  switch (telemetrySource) {
    case TelemetrySource::LocalGps:
      setTelemetrySource(TelemetrySource::AtomS3Remote);
      break;
    case TelemetrySource::AtomS3Remote:
      setTelemetrySource(TelemetrySource::LocalGpsAtomImu);
      break;
    default:
      setTelemetrySource(TelemetrySource::LocalGps);
      break;
  }
}

// GPS and IMU each come from this unit or from the AtomS3. GPS from the AtomS3
// always brings its IMU too: there is no AtomS3-GPS-with-own-IMU mode.
bool gpsFromAtom() { return telemetrySource == TelemetrySource::AtomS3Remote; }
bool imuFromAtom() { return telemetrySource != TelemetrySource::LocalGps; }

void toggleGpsSource() {
  // Leaving the AtomS3's GPS keeps its IMU, which is what a unit without
  // its own IMU needs; R on the IMU page switches that separately.
  setTelemetrySource(gpsFromAtom() ? TelemetrySource::LocalGpsAtomImu
                                   : TelemetrySource::AtomS3Remote);
}

// No change while GPS comes from the AtomS3, which always brings its IMU.
void toggleImuSource() {
  if (gpsFromAtom()) return;
  setTelemetrySource(imuFromAtom() ? TelemetrySource::LocalGps
                                   : TelemetrySource::LocalGpsAtomImu);
}

// Cardputer and Cardputer ADV measure the battery through an ADC divider, and
// M5Unified cannot report charging on them. The Core2's power chip can.
void updateBattery(uint32_t nowMs) {
  if (lastBatteryMs != 0 && nowMs - lastBatteryMs < 5000) return;
  lastBatteryMs = nowMs;
  batteryCharging = M5.Power.isCharging() == m5::Power_Class::is_charging;
  const int32_t level = M5.Power.getBatteryLevel();
  const int32_t millivolts = M5.Power.getBatteryVoltage();
  if (level < 0 || level > 100 || millivolts <= 0) {
    batteryPercent = telemetry::kBatteryUnknown;
    batteryMv = 0;
    return;
  }
  batteryPercent = static_cast<int8_t>(level);
  batteryMv = static_cast<uint16_t>(millivolts);
}

uint16_t batteryColor(int8_t percent) {
  if (percent < 20) return TFT_RED;
  if (percent < 50) return TFT_ORANGE;
  return TFT_GREEN;
}

// Minutes east of UTC for the current local zone, from strftime("%z").
bool localUtcOffsetMinutes(time_t nowUtc, int16_t& minutes) {
  struct tm local {};
  if (localtime_r(&nowUtc, &local) == nullptr) return false;
  char offset[8]{};
  if (strftime(offset, sizeof(offset), "%z", &local) != 5) return false;
  const int hours = (offset[1] - '0') * 10 + (offset[2] - '0');
  const int mins = (offset[3] - '0') * 10 + (offset[4] - '0');
  minutes = static_cast<int16_t>((offset[0] == '-' ? -1 : 1) *
                                 (hours * 60 + mins));
  return true;
}

telemetry::LoggerBoard loggerBoard() {
#if JP226_CORE2
  return telemetry::LoggerBoard::Core2;
#else
  return isCardputerAdv() ? telemetry::LoggerBoard::CardputerAdv
                          : telemetry::LoggerBoard::Cardputer;
#endif
}

void updateBeaconInfo(uint32_t nowMs) {
  if (lastBeaconInfoMs != 0 && nowMs - lastBeaconInfoMs < 1000) return;
  lastBeaconInfoMs = nowMs;
  uint8_t flags = 0;
  uint32_t utc = 0;
  int16_t offset = 0;
  if (clockIsReady()) {
    const time_t nowUtc = time(nullptr);
    utc = static_cast<uint32_t>(nowUtc);
    flags |= telemetry::UtcValid;
    if (localUtcOffsetMinutes(nowUtc, offset)) {
      flags |= telemetry::UtcOffsetValid;
    }
  }
  remoteReceiver.setBeaconInfo(utc, offset, flags, batteryPercent,
                               batteryMv, loggerBoard());
}

void setWaypointMessage(const String& message) {
  waypointMessage = message;
  waypointMessageStartedMs = millis();
  haveDashboardFrameHash = false;
  lastDisplayMs = 0;
}

bool createCurrentWaypoint(const String& category, WaypointRecord& created) {
  const time_t nowUtc = time(nullptr);
  if (!clockIsReady()) {
    setWaypointMessage("WAIT FOR GPS/NTP TIME");
    return false;
  }
  if (!activeTelemetry.gps.fixValid) {
    setWaypointMessage("WAIT FOR GPS FIX");
    return false;
  }
  if (!mountSd(millis())) {
    setWaypointMessage("SD UNAVAILABLE");
    return false;
  }
  char timestamp[40]{};
  formatUtcTimestamp(timestamp, sizeof(timestamp));
  if (!waypointStore.create(activeTelemetry.gps, activeTelemetry.metadata,
                            vehicleContext, nowUtc, timestamp, category,
                            created)) {
    setWaypointMessage("WAYPOINT SAVE FAILED");
    return false;
  }
  waypointShowLast = true;
  setWaypointMessage("WAYPOINT SAVED " + created.id);
  return true;
}

void handleDialCommand(const telemetry::DialCommandPacket& command) {
  const auto action = static_cast<telemetry::DialAction>(command.action);
  char textBuffer[sizeof(command.text) + 1]{};
  std::memcpy(textBuffer, command.text, sizeof(command.text));
  String value = WaypointStore::sanitizeText(textBuffer, 31);
  bool accepted = false;
  String waypointId;
  if (action == telemetry::DialAction::MarkWaypoint) {
    WaypointRecord created;
    accepted = createCurrentWaypoint(value.isEmpty() ? "GENERIC" : value,
                                     created);
    if (accepted) waypointId = created.id;
  } else if (action == telemetry::DialAction::SetRoad) {
    vehicleContext.roadType = value;
    preferences.putString("road", value);
    accepted = true;
  } else if (action == telemetry::DialAction::SetTyreFront ||
             action == telemetry::DialAction::SetTyreRear) {
    if (std::isfinite(command.value) && command.value >= 0.0f &&
        command.value <= 100.0f) {
      if (action == telemetry::DialAction::SetTyreFront) {
        vehicleContext.tyreSetFrontPsi = command.value;
        preferences.putFloat("tyre_f", command.value);
      } else {
        vehicleContext.tyreSetRearPsi = command.value;
        preferences.putFloat("tyre_r", command.value);
      }
      accepted = true;
    }
  } else if (action == telemetry::DialAction::SetSuspensionFront) {
    vehicleContext.suspensionFront = value;
    preferences.putString("susp_f", value);
    accepted = true;
  } else if (action == telemetry::DialAction::SetSuspensionRear) {
    vehicleContext.suspensionRear = value;
    preferences.putString("susp_r", value);
    accepted = true;
  } else if (action == telemetry::DialAction::SetLoad) {
    vehicleContext.vehicleLoad = value;
    preferences.putString("load", value);
    accepted = true;
  }
  if (accepted && action != telemetry::DialAction::MarkWaypoint &&
      sdMounted && clockIsReady()) {
    char timestamp[40]{};
    formatUtcTimestamp(timestamp, sizeof(timestamp));
    waypointStore.appendEvent("CONTEXT_CHANGED", "", String(command.action) +
                                  ":" + value + ":" + String(command.value),
                              telemetrySourceName(activeTelemetry.metadata.source),
                              time(nullptr), timestamp);
  }
  telemetry::DialAckPacket ack{};
  telemetry::preparePacket(ack, telemetry::PacketType::DialAck);
  ack.command_sequence = command.sequence;
  ack.accepted = accepted ? 1 : 0;
  waypointId.toCharArray(ack.waypoint_id, sizeof(ack.waypoint_id));
  telemetry::sealPacket(ack);
  remoteReceiver.sendDialAck(ack);
}

void sendDialStatus(uint32_t nowMs) {
  if (!remoteReceiver.dialConnected(nowMs)) return;
  static uint32_t lastPreparedMs = 0;
  if (nowMs - lastPreparedMs < 500) return;
  lastPreparedMs = nowMs;
  static uint32_t sequence = 0;
  telemetry::DialStatusPacket status{};
  telemetry::preparePacket(status, telemetry::PacketType::DialStatus);
  status.sequence = sequence++;
  status.uptime_ms = nowMs;
  status.log_elapsed_s =
      tripStartedMs == 0 ? 0 : (nowMs - tripStartedMs) / 1000;
  const GpsSnapshot& gps = activeTelemetry.gps;
  status.speed_kmh = gps.speedFresh ? gps.speedKmh : 0.0f;
  status.front_psi = vehicleContext.tyreSetFrontPsi;
  status.rear_psi = vehicleContext.tyreSetRearPsi;
  status.fix_valid = gps.fixValid ? 1 : 0;
  status.satellites = std::min(gps.satellites, static_cast<uint32_t>(255));
  status.log_mode = static_cast<uint8_t>(mode);
  status.source = static_cast<uint8_t>(activeTelemetry.metadata.source);
  status.battery_percent = batteryPercent;
  status.logger_board = static_cast<uint8_t>(loggerBoard());
  if (remoteReceiver.snapshot(nowMs).metadata.remoteConnected) {
    status.link_flags |= telemetry::AtomLinked;
  }
  if (locationTime.wifiConnected()) status.link_flags |= telemetry::WifiOnline;
  if (sdMounted) status.link_flags |= telemetry::SdReady;
  vehicleContext.roadType.toCharArray(status.road, sizeof(status.road));
  vehicleContext.suspensionFront.toCharArray(
      status.suspension_front, sizeof(status.suspension_front));
  vehicleContext.suspensionRear.toCharArray(
      status.suspension_rear, sizeof(status.suspension_rear));
  vehicleContext.vehicleLoad.toCharArray(
      status.vehicle_load, sizeof(status.vehicle_load));
  String poi;
  String poiSource;
  String waypointId;
  currentPoi(gps, poi, poiSource, waypointId);
  poi.toCharArray(status.place, sizeof(status.place));
  if (waypointStore.hasLast()) {
    waypointStore.last().id.toCharArray(status.last_waypoint,
                                        sizeof(status.last_waypoint));
  }
  telemetry::sealPacket(status);
  remoteReceiver.sendDialStatus(status, nowMs);
}

String displayClip(const String& value, size_t maximum) {
  if (value.length() <= maximum) return value;
  if (maximum <= 3) return value.substring(0, maximum);
  return value.substring(0, maximum - 3) + "...";
}

void drawMirroredSevenSegmentDigit(M5Canvas& display, char character, int x,
                                   int y, int width, int height, int thickness,
                                   uint16_t color) {
  constexpr uint8_t digitSegments[10] = {
      0x3F, 0x06, 0x5B, 0x4F, 0x66,
      0x6D, 0x7D, 0x07, 0x7F, 0x6F,
  };
  uint8_t segments = character == '-' ? 0x40 :
                     (character >= '0' && character <= '9'
                          ? digitSegments[character - '0']
                          : 0);

  auto mirroredRect = [&](int offsetX, int offsetY, int rectWidth,
                          int rectHeight) {
    display.fillRect(x + width - offsetX - rectWidth, y + offsetY, rectWidth,
                     rectHeight, color);
  };
  const int half = height / 2;
  if (segments & 0x01) mirroredRect(thickness, 0, width - 2 * thickness, thickness);
  if (segments & 0x02) mirroredRect(width - thickness, thickness, thickness,
                                    half - thickness);
  if (segments & 0x04) mirroredRect(width - thickness, half, thickness,
                                    half - thickness);
  if (segments & 0x08) mirroredRect(thickness, height - thickness,
                                    width - 2 * thickness, thickness);
  if (segments & 0x10) mirroredRect(0, half, thickness, half - thickness);
  if (segments & 0x20) mirroredRect(0, thickness, thickness, half - thickness);
  if (segments & 0x40) mirroredRect(thickness, half - thickness / 2,
                                    width - 2 * thickness, thickness);
}

// Status colours shared by every page: red when something has failed or been
// lost, orange while waiting, green when good.
constexpr uint16_t kStatusBad = TFT_RED;
constexpr uint16_t kStatusWait = TFT_ORANGE;
constexpr uint16_t kStatusGood = TFT_GREEN;
// Accelerometer and gyro axes, matching the AtomS3's IMU page.
constexpr uint16_t kAxisXColor = TFT_RED;
constexpr uint16_t kAxisYColor = TFT_GREEN;
constexpr uint16_t kAxisZColor = 0x54BF;  // Light blue; pure blue is dim.

dino::Game dinoGame;

// Speed fed to the dinosaur game; unknown speed counts as parked.
float dinoGameSpeedKmh() {
  return activeTelemetry.gps.speedFresh ? activeTelemetry.gps.speedKmh : -1.0f;
}

void saveDinoBest() {
  if (dinoGame.takeNewBest()) preferences.putUShort("dino_best", dinoGame.best());
}

uint16_t logModeColor(LogMode value) {
  switch (value) {
    case LogMode::Moving: return kStatusGood;
    case LogMode::FixLost: return kStatusBad;
    case LogMode::StoppedFirstHour:
    case LogMode::StoppedHourly: return TFT_WHITE;
    default: return kStatusWait;
  }
}

// Prints `label` in white and `value` in `color`, leaving the colour white.
void printStatus(M5Canvas& display, const char* label, uint16_t color,
                 const char* value) {
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.print(label);
  display.setTextColor(color, TFT_BLACK);
  display.print(value);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
}

#if !JP226_CORE2  // Cardputer UI: keyboard, 240x135

void beginWaypointEdit(WaypointEditor field) {
  if (!waypointStore.hasLast()) {
    setWaypointMessage("NO WAYPOINT YET");
    return;
  }
  waypointEditor = field;
  const WaypointRecord& last = waypointStore.last();
  waypointInput = field == WaypointEditor::Name
                      ? (last.name == last.id ? "" : last.name)
                      : field == WaypointEditor::Note
                            ? last.note
                            : last.photoReference;
  haveDashboardFrameHash = false;
  lastDisplayMs = 0;
}

void handleWaypointEditor() {
  if (waypointEditor == WaypointEditor::Closed ||
      !M5Cardputer.Keyboard.isChange() ||
      !M5Cardputer.Keyboard.isPressed()) {
    return;
  }
  auto& keys = M5Cardputer.Keyboard.keysState();
  if (keys.esc) {
    waypointEditor = WaypointEditor::Closed;
    setWaypointMessage("EDIT CANCELLED");
    return;
  }
  if (keys.del || keys.backspace) {
    if (!waypointInput.isEmpty()) waypointInput.remove(waypointInput.length() - 1);
  } else if (keys.enter) {
    char timestamp[40]{};
    formatUtcTimestamp(timestamp, sizeof(timestamp));
    const time_t nowUtc = time(nullptr);
    bool saved = false;
    if (mountSd(millis())) {
      switch (waypointEditor) {
        case WaypointEditor::Name:
          saved = waypointStore.editName(waypointInput, nowUtc, timestamp);
          break;
        case WaypointEditor::Note:
          saved = waypointStore.editNote(waypointInput, nowUtc, timestamp);
          break;
        case WaypointEditor::Photo:
          saved = waypointStore.editPhoto(waypointInput, nowUtc, timestamp);
          break;
        default:
          break;
      }
    }
    waypointEditor = WaypointEditor::Closed;
    setWaypointMessage(saved ? "WAYPOINT UPDATED" : "EDIT SAVE FAILED");
    return;
  } else {
    for (char c : keys.word) {
      if (c >= 32 && c <= 126 && waypointInput.length() < 64) {
        waypointInput += c;
      }
    }
  }
  haveDashboardFrameHash = false;
  lastDisplayMs = 0;
}

void selectDashboardPage(DashboardPage page) {
  dashboardPage = page;
  preferences.putUChar("page", static_cast<uint8_t>(page));
  haveDashboardFrameHash = false;
  lastDisplayMs = 0;
}

void moveDashboardPage(int direction) {
  const int count = static_cast<int>(DashboardPage::Count);
  const int current = static_cast<int>(dashboardPage);
  selectDashboardPage(
      static_cast<DashboardPage>((current + direction + count) % count));
}

void handleControls() {
  if (wifiSetup.active()) {
    wifiSetup.handleInput(locationTime);
    if (!wifiSetup.active()) {
      haveDashboardFrameHash = false;
      lastDisplayMs = 0;
    }
    return;
  }
  if (waypointEditor != WaypointEditor::Closed) {
    handleWaypointEditor();
    return;
  }

  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) {
    return;
  }

  auto& keys = M5Cardputer.Keyboard.keysState();
  if (dashboardPage == DashboardPage::About &&
      (keys.space || keys.enter || M5Cardputer.Keyboard.isKeyPressed(';'))) {
    dinoGame.press(millis());
  } else if (M5Cardputer.Keyboard.isKeyPressed('s')) {
    screenOn = !screenOn;
    if (screenOn) {
      M5Cardputer.Display.wakeup();
      M5Cardputer.Display.setBrightness(displayBrightness);
      haveDashboardFrameHash = false;
      lastDisplayMs = 0;
    } else {
      M5Cardputer.Display.sleep();
    }
  } else if (keys.tab || keys.right ||
             M5Cardputer.Keyboard.isKeyPressed(']')) {
    moveDashboardPage(1);
  } else if (keys.left || M5Cardputer.Keyboard.isKeyPressed('[')) {
    moveDashboardPage(-1);
  } else if (M5Cardputer.Keyboard.isKeyPressed('g')) {
    cycleGpsPreference();
  } else if (M5Cardputer.Keyboard.isKeyPressed('r')) {
    if (dashboardPage == DashboardPage::GpsSource) {
      toggleGpsSource();
    } else if (dashboardPage == DashboardPage::ImuSource) {
      toggleImuSource();  // The page explains when GPS pins it to the AtomS3.
    } else {
      cycleTelemetrySource();
    }
  } else if (M5Cardputer.Keyboard.isKeyPressed('p')) {
    selectDashboardPage(DashboardPage::Waypoint);
  } else if (dashboardPage == DashboardPage::Waypoint &&
             M5Cardputer.Keyboard.isKeyPressed('a')) {
    WaypointRecord created;
    createCurrentWaypoint("GENERIC", created);
  } else if (dashboardPage == DashboardPage::Waypoint &&
             M5Cardputer.Keyboard.isKeyPressed('n')) {
    beginWaypointEdit(WaypointEditor::Name);
  } else if (dashboardPage == DashboardPage::Waypoint &&
             M5Cardputer.Keyboard.isKeyPressed('t')) {
    beginWaypointEdit(WaypointEditor::Note);
  } else if (dashboardPage == DashboardPage::Waypoint &&
             M5Cardputer.Keyboard.isKeyPressed('f')) {
    beginWaypointEdit(WaypointEditor::Photo);
  } else if (dashboardPage == DashboardPage::Waypoint &&
             M5Cardputer.Keyboard.isKeyPressed('v')) {
    waypointShowLast = !waypointShowLast;
    lastDisplayMs = 0;
  } else if (M5Cardputer.Keyboard.isKeyPressed('w')) {
    if (!screenOn) {
      screenOn = true;
      M5Cardputer.Display.wakeup();
      M5Cardputer.Display.setBrightness(displayBrightness);
      haveDashboardFrameHash = false;
    }
    wifiSetup.open();
  } else if (M5Cardputer.Keyboard.isKeyPressed('k')) {
    if (sdMounted && clockIsReady()) kmlExporter.requestScan();
    selectDashboardPage(DashboardPage::KmlExport);
  } else if (M5Cardputer.Keyboard.isKeyPressed('-')) {
    displayBrightness = displayBrightness >= 30 ? displayBrightness - 30 : 0;
    if (screenOn) M5Cardputer.Display.setBrightness(displayBrightness);
  } else if (M5Cardputer.Keyboard.isKeyPressed('=')) {
    displayBrightness = displayBrightness <= 225 ? displayBrightness + 30 : 255;
    if (screenOn) M5Cardputer.Display.setBrightness(displayBrightness);
  } else {
    if (M5Cardputer.Keyboard.isKeyPressed('0')) {
      selectDashboardPage(DashboardPage::KmlExport);
      return;
    }
    for (uint8_t index = 0; index < 9; ++index) {
      if (M5Cardputer.Keyboard.isKeyPressed(static_cast<char>('1' + index))) {
        selectDashboardPage(static_cast<DashboardPage>(index));
        break;
      }
    }
  }
}

const char* boardName() {
  switch (M5.getBoard()) {
    case m5::board_t::board_M5CardputerADV:
      return "Cardputer ADV";
    case m5::board_t::board_M5Cardputer:
      return "Cardputer";
    default:
      return "Unknown board";
  }
}

void drawPageTitle(const char* title, uint16_t color = TFT_CYAN) {
  auto& display = dashboardCanvas;
  display.setTextDatum(top_left);
  display.setTextSize(2);
  display.setTextColor(color, TFT_BLACK);
  display.drawString(title, 2, 1);
  display.drawFastHLine(0, 19, display.width(), color);
  if (batteryPercent >= 0) {
    char battery[8]{};
    snprintf(battery, sizeof(battery), "%d%%", batteryPercent);
    display.setTextSize(1);
    display.setTextDatum(top_right);
    display.setTextColor(batteryColor(batteryPercent), TFT_BLACK);
    display.drawString(battery, display.width() - 2, 6);
    display.setTextDatum(top_left);
  }
}

void drawPageFooter(const char* hint = "[ ]/Tab: pages") {
  auto& display = dashboardCanvas;
  display.setTextDatum(bottom_left);
  display.setTextSize(1);
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  char footer[48]{};
  snprintf(footer, sizeof(footer), "%s  %u/%u", hint,
           static_cast<unsigned>(dashboardPage) + 1,
           static_cast<unsigned>(DashboardPage::Count));
  display.drawString(footer, 2, display.height() - 1);
  display.setTextDatum(top_left);
}

void drawCombinedPage(const GpsSnapshot& sample) {
  auto& display = dashboardCanvas;
  drawPageTitle("GPS + IMU");
  char speed[24]{};
  snprintf(speed, sizeof(speed), sample.speedFresh ? "%.1f km/h" : "-- km/h",
           sample.speedKmh);
  display.setTextDatum(middle_center);
  display.setTextSize(4);
  display.setTextColor(sample.fixValid ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  display.drawString(speed, display.width() / 2, 47);

  display.setTextDatum(top_left);
  display.setTextSize(2);
  display.setCursor(3, 73);
  printStatus(display, "GPS:", sample.fixValid ? kStatusGood : kStatusWait,
              sample.fixValid ? "FIX" : "WAIT");
  display.printf(" SAT:%lu\n", static_cast<unsigned long>(sample.satellites));
  const bool imuOk = activeTelemetry.imu.available;
  printStatus(display, "IMU:", imuOk ? kStatusGood : kStatusBad,
              imuOk ? "OK " : "N/A");
  printStatus(display, " SD:", sdMounted ? kStatusGood : kStatusBad,
              sdMounted ? "OK" : "ERR");
  display.println();
  drawPageFooter();
}

void drawSpeedPage(const GpsSnapshot& sample) {
  auto& display = dashboardCanvas;
  drawPageTitle("SPEED");
  char speed[16]{};
  snprintf(speed, sizeof(speed), sample.speedFresh ? "%.1f" : "--",
           sample.speedKmh);
  display.setTextDatum(middle_center);
  display.setTextSize(6);
  display.setTextColor(sample.fixValid ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  display.drawString(speed, display.width() / 2, 60);
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString("km/h", display.width() / 2, 104);
  drawPageFooter();
}

void drawHudSpeedPage(const GpsSnapshot& sample) {
  auto& display = dashboardCanvas;
  char speed[8]{};
  if (sample.speedFresh) {
    const int roundedSpeed = static_cast<int>(std::round(sample.speedKmh));
    snprintf(speed, sizeof(speed), "%d",
             roundedSpeed < 0 ? 0 : (roundedSpeed > 999 ? 999 : roundedSpeed));
  } else {
    strcpy(speed, "--");
  }

  constexpr int digitWidth = 52;
  constexpr int digitHeight = 94;
  constexpr int digitSpacing = 8;
  constexpr int digitTop = 7;
  const int length = strlen(speed);
  const int totalWidth = length * digitWidth + (length - 1) * digitSpacing;
  const int normalStart = (display.width() - totalWidth) / 2;
  const uint16_t color = sample.fixValid ? TFT_GREEN : TFT_ORANGE;

  // Draw the entire number as a horizontal mirror. In the windscreen
  // reflection it appears in the normal reading direction. This avoids the
  // negative-scale sprite operation that is unreliable on original hardware.
  for (int index = 0; index < length; ++index) {
    const int normalX = normalStart + index * (digitWidth + digitSpacing);
    const int mirroredX = display.width() - normalX - digitWidth;
    drawMirroredSevenSegmentDigit(display, speed[index], mirroredX, digitTop,
                                  digitWidth, digitHeight, 8, color);
  }
  display.setTextDatum(bottom_center);
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString("MIRRORED HUD", display.width() / 2, 122);
  display.setTextDatum(top_left);
  display.setTextSize(1);
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.drawString("[ ] / Tab changes page", 2, 126);
}

void drawGpsStatusPage(const GpsSnapshot& sample, uint32_t nowMs) {
  auto& display = dashboardCanvas;
  drawPageTitle("GPS STATUS");
  display.setTextSize(2);
  display.setTextColor(sample.fixValid ? kStatusGood : kStatusBad, TFT_BLACK);
  display.setCursor(3, 25);
  display.printf("%s  %lu SAT\n", sample.fixValid ? "FIX" : "NO FIX",
                 static_cast<unsigned long>(sample.satellites));
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  if (sample.positionFresh) {
    display.printf("LAT %.6f\nLON %.6f\n", sample.latitude, sample.longitude);
  } else {
    display.println("LAT --\nLON --");
  }
  if (telemetrySource == TelemetrySource::AtomS3Remote) {
    const bool live = activeTelemetry.metadata.remoteConnected;
    printStatus(display, "ATOM ", live ? kStatusGood : kStatusBad,
                live ? "LIVE" : "LOST");
  } else {
    const bool live = activeGps->live(nowMs);
    display.printf("%s ", activeGps->name());
    printStatus(display, "", live ? kStatusGood : kStatusWait,
                live ? "LIVE" : "SCAN");
  }
  display.print(" H:");
  if (sample.hdopValid) display.printf("%.1f", sample.hdop);
  else display.print("--");
  display.println();
  display.printf("TZ:%s", displayClip(locationTime.zoneName(), 17).c_str());
  drawPageFooter();
}

void drawImuStatusPage() {
  auto& display = dashboardCanvas;
  drawPageTitle("IMU STATUS");
  display.setTextSize(2);
  display.setCursor(3, 27);
  const ImuSample& value = activeTelemetry.imu;
  if (!value.available) {
    display.setTextColor(kStatusBad, TFT_BLACK);
    display.println("IMU unavailable");
    display.println(telemetrySource == TelemetrySource::AtomS3Remote
                        ? "REMOTE GPS/IMU LOST"
                    : telemetrySource == TelemetrySource::LocalGpsAtomImu
                        ? "ATOM IMU LOST"
                        : "Local IMU N/A");
    drawPageFooter();
    return;
  }
  display.setTextColor(value.valid ? kStatusGood : kStatusWait, TFT_BLACK);
  display.printf("%s %s\n",
                 usesRemoteImu(telemetrySource) ? "ATOM MPU6886"
                                                : imu.typeName(),
                 value.valid ? "LIVE" : "WAIT");
  struct Reading {
    uint16_t color;
    const char* format;
    float value;
  };
  // Acceleration X/Y/Z on the left; vibration, pitch and roll on the right.
  const Reading left[3] = {{kAxisXColor, "X:%+.1f", value.axMps2},
                           {kAxisYColor, "Y:%+.1f", value.ayMps2},
                           {kAxisZColor, "Z:%+.1f", value.azMps2}};
  const Reading right[3] = {{TFT_CYAN, "V:%.2f", value.vibrationRmsMps2},
                            {TFT_WHITE, "P:%+.1f", value.pitchDeg},
                            {TFT_WHITE, "R:%+.1f", value.rollDeg}};
  constexpr int32_t kLeftX = 3;
  constexpr int32_t kRightX = 124;
  constexpr int32_t kRowY[3] = {45, 63, 81};
  for (int row = 0; row < 3; ++row) {
    display.setTextColor(left[row].color, TFT_BLACK);
    display.setCursor(kLeftX, kRowY[row]);
    display.printf(left[row].format, left[row].value);
    display.setTextColor(right[row].color, TFT_BLACK);
    display.setCursor(kRightX, kRowY[row]);
    display.printf(right[row].format, right[row].value);
  }
  const Reading gyro[3] = {{kAxisXColor, "X:%+.1f ", value.gxDps},
                           {kAxisYColor, "Y:%+.1f ", value.gyDps},
                           {kAxisZColor, "Z:%+.1f", value.gzDps}};
  display.setTextSize(1);
  display.setCursor(kLeftX, 101);
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.print("m/s2  gyro ");
  for (const Reading& axis : gyro) {
    display.setTextColor(axis.color, TFT_BLACK);
    display.printf(axis.format, axis.value);
  }
  drawPageFooter();
}

// Big "FROM" line shared by the two source pages.
void drawSourceHeading(bool fromAtom, const char* ownName) {
  auto& display = dashboardCanvas;
  display.setTextSize(2);
  display.setCursor(3, 27);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.print("FROM ");
  display.setTextSize(3);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.println(fromAtom ? "ATOMS3" : ownName);
  display.setTextSize(2);
}

// AtomS3 link line used by both source pages.
void printAtomLink(uint32_t nowMs) {
  auto& display = dashboardCanvas;
  const bool live = activeTelemetry.metadata.remoteConnected;
  display.setCursor(3, 57);
  printStatus(display, "LINK ", live ? kStatusGood : kStatusBad,
              live ? "LIVE" : remoteReceiver.statusText(nowMs));
  display.println();
}

void drawGpsSourcePage(uint32_t nowMs) {
  auto& display = dashboardCanvas;
  drawPageTitle("GPS SOURCE");
  drawSourceHeading(gpsFromAtom(), "OWN GPS");
  if (gpsFromAtom()) {
    printAtomLink(nowMs);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.printf("Seq:%lu Age:", static_cast<unsigned long>(
                                      activeTelemetry.metadata.remoteSequence));
    if (activeTelemetry.metadata.packetAgeMs == UINT32_MAX) display.println("--");
    else display.printf("%lums\n", static_cast<unsigned long>(
                                     activeTelemetry.metadata.packetAgeMs));
    display.setTextSize(1);
    display.println(displayClip(remoteReceiver.peerMacText(), 24));
  } else {
    const bool live = activeGps->live(nowMs);
    display.setCursor(3, 57);
    display.printf("%s %lu ", activeGps->name(),
                   static_cast<unsigned long>(activeGps->baud()));
    printStatus(display, "", live ? kStatusGood : kStatusWait,
                live ? "LIVE" : "SCAN");
    display.println();
    display.printf("Prefer: %s\n", gpsPreferenceName());
  }
  drawPageFooter("R:own/atom G:grove/cap");
}

void drawImuSourcePage(uint32_t nowMs) {
  auto& display = dashboardCanvas;
  drawPageTitle("IMU SOURCE");
  drawSourceHeading(imuFromAtom(), "OWN IMU");
  if (imuFromAtom()) {
    printAtomLink(nowMs);
    const bool live = activeTelemetry.imu.available;
    printStatus(display, "IMU ", live ? kStatusGood : kStatusBad,
                live ? "ATOM MPU6886" : "NO DATA");
    display.println();
    if (gpsFromAtom()) {
      display.setTextSize(1);
      display.setTextColor(TFT_DARKGREY, TFT_BLACK);
      display.println("Comes with the AtomS3's GPS; change GPS first");
    }
  } else {
    const ImuSample local = imu.latest();
    display.setCursor(3, 57);
    if (!local.available) {
      display.setTextColor(kStatusBad, TFT_BLACK);
      display.println("NO IMU IN THIS UNIT");
      display.setTextSize(1);
      display.setTextColor(TFT_DARKGREY, TFT_BLACK);
      display.println("Press R to use an AtomS3's IMU");
    } else {
      printStatus(display, "", local.valid ? kStatusGood : kStatusWait,
                  imu.typeName());
      display.println(local.valid ? " LIVE" : " WAIT");
    }
  }
  drawPageFooter("R:own/atom");
}

void drawWifiPage() {
  auto& display = dashboardCanvas;
  drawPageTitle("WI-FI");
  display.setTextSize(2);
  display.setCursor(3, 27);
  if (locationTime.wifiConnected()) {
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.println("CONNECTED");
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.println(displayClip(WiFi.SSID(), 18));
    display.setTextColor(TFT_CYAN, TFT_BLACK);
    display.println("WEB: HTTP PORT 80");
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    display.setTextSize(1);
    display.setTextColor(TFT_CYAN, TFT_BLACK);
    display.println("http://jp226-logger.local");
    display.setTextSize(2);
  } else {
    display.setTextColor(kStatusBad, TFT_BLACK);
    display.println("OFFLINE");
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    const String saved = locationTime.wifiSsid();
    display.println(saved.isEmpty() ? "No saved network" : displayClip(saved, 18));
  }
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.println("W: WI-FI SETUP");
  drawPageFooter();
}

void drawLoggerPage(time_t nowUtc) {
  auto& display = dashboardCanvas;
  drawPageTitle("LOGGER / SD");
  display.setTextSize(2);
  display.setCursor(3, 27);
  display.setTextColor(sdMounted ? kStatusGood : kStatusBad, TFT_BLACK);
  display.printf("SD: %s  LOG: ON\n", sdMounted ? "READY" : "RETRY");
  printStatus(display, "State: ", logModeColor(mode), modeName(mode));
  display.println();
  display.printf("Rows: %llu\n", static_cast<unsigned long long>(rowsWritten));
  if (!currentLogPath.isEmpty()) {
    const int slash = currentLogPath.lastIndexOf('/');
    display.println(displayClip(currentLogPath.substring(slash + 1), 19));
  } else {
    display.println("Waiting for clock");
  }
  if (batteryPercent >= 0) {
    display.setTextColor(batteryColor(batteryPercent), TFT_BLACK);
    display.printf("BAT %d%% %.2fV\n", batteryPercent, batteryMv / 1000.0f);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
  }
  if (nextStoppedDueUtc > nowUtc) {
    display.setTextSize(1);
    display.printf("Next parked row in %lld min",
                   static_cast<long long>((nextStoppedDueUtc - nowUtc + 59) / 60));
  }
  drawPageFooter();
}

void drawTimeNetworkPage(time_t nowUtc) {
  auto& display = dashboardCanvas;
  drawPageTitle("TIME / NETWORK");
  if (!clockIsReady()) {
    display.setTextSize(2);
    display.setTextColor(TFT_ORANGE, TFT_BLACK);
    display.drawString("Waiting for time", 8, 42);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawString("GPS or Wi-Fi NTP", 8, 72);
    drawPageFooter();
    return;
  }

  struct tm local {};
  localtime_r(&nowUtc, &local);
  char clockText[16]{};
  char dateText[20]{};
  snprintf(clockText, sizeof(clockText), "%02d:%02d:%02d", local.tm_hour,
           local.tm_min, local.tm_sec);
  snprintf(dateText, sizeof(dateText), "%04d-%02d-%02d", local.tm_year + 1900,
           local.tm_mon + 1, local.tm_mday);
  display.setTextDatum(middle_center);
  display.setTextSize(4);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.drawString(clockText, display.width() / 2, 48);
  display.setTextSize(2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(dateText, display.width() / 2, 80);
  display.setTextSize(1);
  const String zone = String(locationTime.zoneName()) + " (" +
                      locationTime.zoneSource() + ")";
  display.drawString(displayClip(zone, 36), display.width() / 2, 105);
  display.setTextDatum(top_left);
  drawPageFooter();
}

void drawKmlExportPage() {
  auto& display = dashboardCanvas;
  drawPageTitle("KML EXPORT");
  display.setTextSize(2);
  display.setCursor(3, 25);
  const bool stationary = stationaryForFileWork();
  display.setTextColor(kmlExporter.failedCount() == 0 && stationary
                           ? TFT_GREEN
                           : TFT_ORANGE,
                       TFT_BLACK);
  display.printf("%s\n", stationary ? kmlExporter.phaseName() : "WAITING TO STOP");
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.printf("Done:%u  Err:%u\n", kmlExporter.completedCount(),
                 kmlExporter.failedCount());
  display.printf("Queue:%u Pts:%lu\n", kmlExporter.queuedCount(),
                 static_cast<unsigned long>(kmlExporter.pointsWritten()));
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.println(displayClip(kmlExporter.currentFile(), 36));
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.println(displayClip(kmlExporter.lastMessage(), 36));
  display.setTextSize(2);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString("K: SCAN OLD DAYS", 3, 105);
  drawPageFooter();
}

// Size-2 text fits 20 characters across and five rows between the title and
// the two small hint lines, so each row carries one fact.
void drawWaypointPage(const GpsSnapshot& gps, uint32_t nowMs) {
  auto& display = dashboardCanvas;
  drawPageTitle("WAYPOINT");
  constexpr int kLarge = 20;
  constexpr int kRowY[] = {22, 39, 56, 73, 90};
  char line[48]{};
  display.setTextDatum(top_left);
  display.setTextSize(2);

  char hdop[8] = "--";
  if (gps.hdopValid) snprintf(hdop, sizeof(hdop), "%.1f", gps.hdop);
  snprintf(line, sizeof(line), "%s %luSAT H%s", gps.fixValid ? "FIX" : "WAIT",
           static_cast<unsigned long>(gps.satellites), hdop);
  display.setTextColor(gps.fixValid ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  display.drawString(line, 3, kRowY[0]);
  display.setTextColor(TFT_WHITE, TFT_BLACK);

  const bool editing = waypointEditor != WaypointEditor::Closed;
  const bool messageActive = nowMs - waypointMessageStartedMs < 5000 &&
                             !waypointMessage.isEmpty();
  const bool haveLast = waypointStore.hasLast();
  if (haveLast && waypointShowLast) {
    const auto& last = waypointStore.last();
    display.drawString(displayClip("LAST " + last.id + " " + last.name, kLarge),
                       3, kRowY[1]);
    display.drawString(displayClip(last.timestamp, kLarge), 3, kRowY[2]);
    display.drawString(displayClip("NOTE " + last.note, kLarge), 3, kRowY[3]);
    if (!editing && !messageActive) {
      display.drawString(displayClip("PHOTO " + last.photoReference, kLarge),
                         3, kRowY[4]);
    }
  } else {
    String poi;
    String poiSource;
    String waypointId;
    currentPoi(gps, poi, poiSource, waypointId);
    display.drawString(displayClip("POI " + (poi.isEmpty() ? String("--") : poi),
                                   kLarge),
                       3, kRowY[1]);
    if (haveLast) {
      const auto& last = waypointStore.last();
      display.drawString(
          displayClip("LAST " + last.id + " " + last.name, kLarge), 3,
          kRowY[2]);
      if (gps.positionFresh) {
        snprintf(line, sizeof(line), "%.0fm %s",
                 waypointStore.distanceFromLastM(gps.latitude, gps.longitude),
                 last.category.c_str());
        display.drawString(displayClip(line, kLarge), 3, kRowY[3]);
      }
    } else {
      display.setTextColor(TFT_DARKGREY, TFT_BLACK);
      display.drawString("NO WAYPOINTS YET", 3, kRowY[2]);
      display.setTextColor(TFT_WHITE, TFT_BLACK);
    }
  }

  if (editing) {
    const char* label = waypointEditor == WaypointEditor::Name
                            ? "NAME"
                            : waypointEditor == WaypointEditor::Note ? "NOTE"
                                                                      : "PHOTO";
    display.setTextColor(TFT_CYAN, TFT_BLACK);
    // Keep the end of the input visible while typing.
    const size_t room = kLarge - strlen(label) - 2;
    String shown = waypointInput;
    if (shown.length() > room) shown = shown.substring(shown.length() - room);
    display.drawString(String(label) + ":" + shown + "_", 3, kRowY[4]);
  } else if (messageActive) {
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.drawString(displayClip(waypointMessage, kLarge), 3, kRowY[4]);
  }

  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  if (gps.positionFresh) {
    snprintf(line, sizeof(line), "LAT %.6f  LON %.6f", gps.latitude,
             gps.longitude);
    display.drawString(line, 3, 108);
  } else {
    display.drawString("LAT --  LON --", 3, 108);
  }
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString(editing ? "Enter: save  Esc: cancel"
                             : "A:add N:name T:note F:photo V:last",
                     3, 117);
  drawPageFooter("P:waypoint  [ ]:pages");
}

void drawAboutPage(uint32_t nowMs) {
  auto& display = dashboardCanvas;
  drawPageTitle("JP226PRINTS");
  constexpr int kScale = 3;
  constexpr int kGroundY = 26 + dino::kHeight * kScale + 1;
  const int width = display.width();
  dinoGame.update(nowMs, dinoGameSpeedKmh(), width / kScale);
  saveDinoBest();
  dinoGame.draw(display, 0, kGroundY, width, kScale, nowMs);
  // Game text sits at the right of the title bar, clear of the dinosaur.
  display.setTextDatum(top_right);
  display.setTextSize(1);
  switch (dinoGame.state()) {
    case dino::Game::State::Ready:
      display.setTextColor(TFT_YELLOW, TFT_BLACK);
      display.drawString("SPACE: PLAY", width - 3, 14);
      break;
    case dino::Game::State::Playing:
      display.setTextColor(TFT_WHITE, TFT_BLACK);
      display.drawString(String("SCORE ") + dinoGame.score() + "  BEST " +
                             dinoGame.best(),
                         width - 3, 14);
      break;
    case dino::Game::State::Over:
      display.setTextColor(TFT_RED, TFT_BLACK);
      display.drawString(String("GAME OVER ") + dinoGame.score() + "  BEST " +
                             dinoGame.best(),
                         width - 3, 14);
      break;
    default:
      break;
  }
  display.setTextDatum(middle_center);
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.drawString(dino::kCredit1, display.width() / 2, 90);
  display.setTextSize(3);
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.drawString(dino::kCredit2, display.width() / 2, 110);
  display.setTextDatum(top_left);
  drawPageFooter();
}

void drawStatus(const GpsSnapshot& sample, uint32_t nowMs, time_t nowUtc) {
  if (!screenOn || wifiSetup.active()) return;
  // The dinosaur page animates; everything else refreshes twice a second.
  const uint32_t interval = dashboardPage == DashboardPage::About
                                ? 60
                                : config::kDisplayIntervalMs;
  if (nowMs - lastDisplayMs < interval) return;
  lastDisplayMs = nowMs;

  if (!dashboardCanvasReady) {
    auto& physicalDisplay = M5Cardputer.Display;
    physicalDisplay.fillScreen(TFT_BLACK);
    physicalDisplay.setTextSize(2);
    physicalDisplay.setTextColor(TFT_RED, TFT_BLACK);
    physicalDisplay.setCursor(4, 35);
    physicalDisplay.println("Display buffer");
    physicalDisplay.println("unavailable");
    return;
  }

  auto& display = dashboardCanvas;
  display.startWrite();
  display.fillScreen(TFT_BLACK);
  display.setTextWrap(false);
  display.setTextDatum(top_left);

  switch (dashboardPage) {
    case DashboardPage::Combined:
      drawCombinedPage(sample);
      break;
    case DashboardPage::Speed:
      drawSpeedPage(sample);
      break;
    case DashboardPage::HudSpeed:
      drawHudSpeedPage(sample);
      break;
    case DashboardPage::GpsStatus:
      drawGpsStatusPage(sample, nowMs);
      break;
    case DashboardPage::ImuStatus:
      drawImuStatusPage();
      break;
    case DashboardPage::GpsSource:
      drawGpsSourcePage(nowMs);
      break;
    case DashboardPage::ImuSource:
      drawImuSourcePage(nowMs);
      break;
    case DashboardPage::Wifi:
      drawWifiPage();
      break;
    case DashboardPage::Logger:
      drawLoggerPage(nowUtc);
      break;
    case DashboardPage::TimeNetwork:
      drawTimeNetworkPage(nowUtc);
      break;
    case DashboardPage::KmlExport:
      drawKmlExportPage();
      break;
    case DashboardPage::Waypoint:
      drawWaypointPage(sample, nowMs);
      break;
    case DashboardPage::About:
      drawAboutPage(nowMs);
      break;
    default:
      break;
  }
  display.setTextDatum(top_left);
  display.setTextSize(1);
  display.endWrite();
  const uint32_t frameHash = frameBufferHash(display);
  if (!haveDashboardFrameHash || frameHash != lastDashboardFrameHash) {
    display.pushSprite(0, 0);
    lastDashboardFrameHash = frameHash;
    haveDashboardFrameHash = true;
  }
}

#else
#include "Core2Ui.inc"
#endif  // JP226_CORE2

// Board, IMU, internal I2C devices and radio links, printed every 10 s from
// 5 s after boot so a serial monitor opened after start-up still sees them.
// Worst time per loop() step since the last hardware report, to find what
// delays key handling: a key tapped and released within one slow pass is
// never seen.
enum LoopStep : uint8_t {
  StepControls, StepGps, StepImu, StepRadio, StepClockWifi, StepStatus,
  StepCsv, StepSd, StepKml, StepWeb, StepDraw, StepCount
};
constexpr const char* kLoopStepNames[StepCount] = {
    "controls", "gps", "imu", "radio", "clock/wifi", "status",
    "csv", "sd", "kml", "web", "draw"};
uint32_t loopStepMaxUs[StepCount]{};
uint32_t loopMaxUs = 0;
uint32_t loopStepStartUs = 0;

void markLoopStep(LoopStep step) {
  const uint32_t nowUs = micros();
  loopStepMaxUs[step] = std::max(loopStepMaxUs[step], nowUs - loopStepStartUs);
  loopStepStartUs = nowUs;
}

void printLoopTiming() {
  uint8_t order[StepCount];
  for (uint8_t i = 0; i < StepCount; ++i) order[i] = i;
  std::sort(order, order + StepCount, [](uint8_t a, uint8_t b) {
    return loopStepMaxUs[a] > loopStepMaxUs[b];
  });
  Serial.printf("Loop max %lu ms; slowest:",
                static_cast<unsigned long>(loopMaxUs / 1000));
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.printf(" %s %lu ms", kLoopStepNames[order[i]],
                  static_cast<unsigned long>(loopStepMaxUs[order[i]] / 1000));
  }
  Serial.println();
  std::fill(loopStepMaxUs, loopStepMaxUs + StepCount, 0);
  loopMaxUs = 0;
}

void printHardwareReport(uint32_t nowMs) {
  static uint32_t lastReportMs = 0;
  if (nowMs < 5000 || (lastReportMs != 0 && nowMs - lastReportMs < 10000)) {
    return;
  }
  lastReportMs = nowMs;
  bool found[0x80]{};
  M5.In_I2C.scanID(found);
  Serial.printf("\nBoard %d, IMU %s, battery %d%%, internal I2C:",
                static_cast<int>(M5.getBoard()), imu.typeName(),
                batteryPercent);
  for (int address = 0x08; address < 0x78; ++address) {
    if (found[address]) Serial.printf(" 0x%02X", address);
  }
  Serial.printf("\nSource %s, remote %s %s, IMU %s, Dial %s, channel %u\n",
                telemetrySourceLabel(telemetrySource),
                remoteReceiver.statusText(nowMs),
                remoteReceiver.peerMacText().c_str(),
                activeTelemetry.imu.available ? "live" : "unavailable",
                remoteReceiver.dialConnected(nowMs) ? "connected" : "not seen",
                WiFi.channel());
  printLoopTiming();
}

void loadPersistentState() {
  preferences.begin("gpsimu", false);
  dinoGame.setBest(preferences.getUShort("dino_best", 0));
  const uint8_t savedPage = preferences.getUChar("page", 0);
#if JP226_CORE2
  restoreCore2Page(savedPage);
#else
  if (savedPage < static_cast<uint8_t>(DashboardPage::Count)) {
    dashboardPage = static_cast<DashboardPage>(savedPage);
  }
#endif
  const uint8_t savedGpsPreference = preferences.getUChar("gps_src", 0);
  if (savedGpsPreference <= static_cast<uint8_t>(GpsPreference::Cap) &&
      (savedGpsPreference != static_cast<uint8_t>(GpsPreference::Cap) ||
       isCardputerAdv())) {
    gpsPreference = static_cast<GpsPreference>(savedGpsPreference);
  }
  const uint8_t defaultSource = config::kDefaultToAtomS3Remote
                                    ? static_cast<uint8_t>(
                                          TelemetrySource::AtomS3Remote)
                                    : static_cast<uint8_t>(
                                          TelemetrySource::LocalGps);
  const uint8_t savedTelemetrySource =
      preferences.getUChar("tele_src", defaultSource);
  if (savedTelemetrySource ==
          static_cast<uint8_t>(TelemetrySource::LocalGps) ||
      savedTelemetrySource ==
          static_cast<uint8_t>(TelemetrySource::AtomS3Remote) ||
      savedTelemetrySource ==
          static_cast<uint8_t>(TelemetrySource::LocalGpsAtomImu)) {
    telemetrySource = static_cast<TelemetrySource>(savedTelemetrySource);
  }
  if (preferences.getUChar("state", 0) == 2) {
    persistedStopStartUtc =
        static_cast<time_t>(preferences.getULong64("stop_utc", 0));
    persistedNextDueUtc =
        static_cast<time_t>(preferences.getULong64("next_utc", 0));
    persistedStopPending = persistedStopStartUtc > 0 && persistedNextDueUtc > 0;
  }
  vehicleContext.roadType = preferences.getString("road", "");
  vehicleContext.tyreSetFrontPsi = preferences.getFloat("tyre_f", NAN);
  vehicleContext.tyreSetRearPsi = preferences.getFloat("tyre_r", NAN);
  vehicleContext.suspensionFront = preferences.getString("susp_f", "");
  vehicleContext.suspensionRear = preferences.getString("susp_r", "");
  vehicleContext.vehicleLoad = preferences.getString("load", "");
}

}  // namespace

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // Plugged into a PC with no serial monitor open, each print otherwise waits
  // up to 2 s for a reader, stalling loop() long enough to miss key presses.
  Serial.setTxTimeoutMs(0);
#endif
  setenv("TZ", config::kPosixTimezone, 1);
  tzset();

  auto m5Config = M5.config();
#if JP226_CORE2
  M5.begin(m5Config);
#else
  M5Cardputer.begin(m5Config, true);
#endif
  M5.Display.setRotation(1);
  M5.Display.setTextSize(1);
  M5.Display.setTextWrap(false);
  M5.Display.setBrightness(displayBrightness);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(2, 2);
  M5.Display.println("Starting GPS + IMU logger...");
  restoreClockFromRtc();

#if JP226_CORE2
  // A 320x240 16-bit frame is 150 KB, so it lives in PSRAM.
  dashboardCanvas.setPsram(true);
  dashboardCanvas.setColorDepth(16);
#else
  dashboardCanvas.setColorDepth(8);
#endif
  dashboardCanvasReady =
      dashboardCanvas.createSprite(M5.Display.width(), M5.Display.height()) !=
      nullptr;
#if !JP226_CORE2
  if (dashboardCanvasReady) wifiSetup.setCanvas(dashboardCanvas);
#endif

  imu.begin();
  loadPersistentState();
  waypointStore.begin(SD, preferences, config::kLogDirectory);
  remoteReceiver.begin(config::kAtomEspNowMac);
  kmlExporter.begin(SD, config::kLogDirectory, config::kFilePrefix,
                    config::kKmlDirectory);
  webPortal.begin(SD, config::kLogDirectory, config::kKmlDirectory,
                  config::kWebHostname);

  groveBaudIndex = 0;
  groveGps.begin(config::kGroveGpsBaudCandidates[groveBaudIndex]);
  groveBaudStartedMs = millis();
  if (isCardputerAdv()) {
    // The Cap LoRa radio and microSD share SPI pins. NSS is active-low, so
    // holding it high prevents the unused SX1262 from driving the SD bus.
    pinMode(config::kCapLoraCsPin, OUTPUT);
    digitalWrite(config::kCapLoraCsPin, HIGH);
    capGps.begin(config::kCapGpsBaud);
  }
  selectGpsReceiver(millis());
  if (mountSd(millis(), true)) {
    locationTime.begin(SD, preferences);
    locationTimeStarted = true;
    placeResolver.begin(SD, config::kLoggerConfigPath);
    placeResolverStarted = true;
  }
}

void loop() {
  const uint32_t loopStartUs = micros();
  loopStepStartUs = loopStartUs;
#if JP226_CORE2
  M5.update();
#else
  M5Cardputer.update();
#endif
  handleControls();
  markLoopStep(StepControls);

  const uint32_t nowMs = millis();
  updateGpsReceivers(nowMs);
  markLoopStep(StepGps);
  imu.update(nowMs);
  markLoopStep(StepImu);
  remoteReceiver.update(nowMs);

  activeTelemetry = takeActiveTelemetry(nowMs);
  telemetry::DialCommandPacket dialCommand{};
  if (remoteReceiver.popDialCommand(dialCommand)) {
    handleDialCommand(dialCommand);
  }
  const GpsSnapshot& gpsSample = activeTelemetry.gps;
  if (gpsSample.fixValid && tripStartedMs == 0) tripStartedMs = nowMs;
  markLoopStep(StepRadio);
  syncClockFromTelemetry(gpsSample, nowMs);
  if (!wifiSetup.active()) {
    locationTime.update(gpsSample.latitude, gpsSample.longitude,
                        gpsSample.positionFresh, gpsSample.speedKmh, nowMs);
  }
  wifiSetup.update(nowMs);
  placeResolver.update(gpsSample.latitude, gpsSample.longitude,
                       gpsSample.positionFresh, nowMs);
  String resolvedPlace;
  if (placeResolver.takeChange(resolvedPlace)) {
    if (clockIsReady() && sdMounted) {
      char eventTimestamp[32];
      formatUtcTimestamp(eventTimestamp, sizeof(eventTimestamp));
      if (!autoPlace.isEmpty()) {
        waypointStore.appendEvent("PLACE_EXIT", "", autoPlace,
                                  telemetrySourceName(activeTelemetry.metadata.source),
                                  time(nullptr), eventTimestamp);
      }
      if (!resolvedPlace.isEmpty()) {
        waypointStore.appendEvent("PLACE_ENTER", "", resolvedPlace,
                                  telemetrySourceName(activeTelemetry.metadata.source),
                                  time(nullptr), eventTimestamp);
      }
    }
    autoPlace = resolvedPlace;
  }
  markLoopStep(StepClockWifi);
  syncRtcFromSystem(nowMs);
  const time_t nowUtc = time(nullptr);
  if (clockIsReady()) closeCompletedDailyFile(nowUtc);
  updateLoggingMode(gpsSample, nowMs, nowUtc);
  updateBattery(nowMs);
  printHardwareReport(nowMs);
  updateBeaconInfo(nowMs);
  sendDialStatus(nowMs);

  markLoopStep(StepStatus);
  if (clockIsReady() && logIsDue(nowMs, nowUtc) &&
      writeCsvRow(activeTelemetry, nowMs, nowUtc)) {
    onLogSucceeded(nowMs, nowUtc);
  }
#if ENABLE_RAW_IMU_LOGGING
  writeRawImuBatches(nowUtc, nowMs);
#endif
  markLoopStep(StepCsv);

  mountSd(nowMs);
  if (sdMounted && !locationTimeStarted) {
    locationTime.begin(SD, preferences);
    locationTimeStarted = true;
  }
  if (sdMounted && !placeResolverStarted) {
    placeResolver.begin(SD, config::kLoggerConfigPath);
    placeResolverStarted = true;
  }
  markLoopStep(StepSd);
  if (sdMounted && clockIsReady()) {
    const String currentDate = localDateText(nowUtc);
    if (currentDate != lastKmlScanDate) {
      lastKmlScanDate = currentDate;
      kmlExporter.requestScan();
    }
    if (webPortal.takeConversionRequest()) kmlExporter.requestScan();
    if (stationaryForFileWork()) {
      kmlExporter.update(currentLogPath, currentDate);
    }
  }

  markLoopStep(StepKml);
  WebPortalStatus webStatus;
  webStatus.sdMounted = sdMounted;
  webStatus.stationary = stationaryForFileWork();
  webStatus.fixValid = gpsSample.fixValid;
  webStatus.speedKmh = gpsSample.speedFresh ? gpsSample.speedKmh : 0.0;
  webStatus.logState = modeName(mode);
  webStatus.activeCsvPath = currentLogPath;
  webStatus.timezone = locationTime.zoneName();
  webStatus.kmlPhase = kmlExporter.phaseName();
  webStatus.kmlMessage = kmlExporter.lastMessage();
  webStatus.kmlQueued = kmlExporter.queuedCount();
  webStatus.kmlCompleted = kmlExporter.completedCount();
  webStatus.kmlFailed = kmlExporter.failedCount();
  webStatus.kmlPoints = kmlExporter.pointsWritten();
  webPortal.update(webStatus);
  markLoopStep(StepWeb);
  drawStatus(gpsSample, nowMs, nowUtc);
  wifiSetup.draw(nowMs);
  markLoopStep(StepDraw);
  loopMaxUs = std::max(loopMaxUs, micros() - loopStartUs);
  delay(2);
}
