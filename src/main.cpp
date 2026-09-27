#include <Arduino.h>
#include <M5Cardputer.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <time.h>

#include <cmath>
#include <cstdarg>
#include <cstring>

#include "Config.h"
#include "LogSchedule.h"

namespace {

constexpr char kCsvHeader[] =
    "timestamp,lat,lon,alt_m,speed_kmh,heading_deg,satellites,hdop,vdop,"
    "acc_x_g,acc_y_g,acc_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,pitch_deg,"
    "roll_deg,g_total,roughness_index,local_timestamp,fix_valid,fix_age_ms,"
    "imu_available,imu_type,log_state,uptime_ms";

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

struct GpsSnapshot {
  bool fixValid = false;
  bool positionFresh = false;
  bool speedFresh = false;
  bool altitudeFresh = false;
  bool courseFresh = false;
  bool satellitesValid = false;
  bool hdopValid = false;
  bool vdopValid = false;
  uint32_t fixAgeMs = UINT32_MAX;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitudeM = 0.0;
  double speedKmh = 0.0;
  double courseDeg = 0.0;
  uint32_t satellites = 0;
  double hdop = 0.0;
  double vdop = 0.0;
};

struct ImuSample {
  bool available = false;
  bool valid = false;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  float gTotal = 0.0f;
  float roughness = 0.0f;
};

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
    latest_.ax = data.accel.x;
    latest_.ay = data.accel.y;
    latest_.az = data.accel.z;
    latest_.gx = data.gyro.x;
    latest_.gy = data.gyro.y;
    latest_.gz = data.gyro.z;
    latest_.gTotal = std::sqrt(latest_.ax * latest_.ax + latest_.ay * latest_.ay +
                               latest_.az * latest_.az);
    latest_.pitch = std::atan2(-latest_.ax,
                               std::sqrt(latest_.ay * latest_.ay +
                                         latest_.az * latest_.az)) *
                    180.0f / PI;
    latest_.roll = std::atan2(latest_.ay, latest_.az) * 180.0f / PI;

    if (!gravityFilterReady_) {
      gravityMagnitude_ = latest_.gTotal;
      gravityFilterReady_ = true;
    } else {
      constexpr float alpha = 0.02f;
      gravityMagnitude_ += alpha * (latest_.gTotal - gravityMagnitude_);
    }
    const float vibration = latest_.gTotal - gravityMagnitude_;
    roughnessSumSquares_ += static_cast<double>(vibration) * vibration;
    ++roughnessCount_;
  }

  ImuSample snapshotAndResetRoughness() {
    ImuSample result = latest_;
    result.available = available_;
    if (available_ && roughnessCount_ > 0) {
      result.roughness =
          std::sqrt(roughnessSumSquares_ / static_cast<double>(roughnessCount_));
    }
    roughnessSumSquares_ = 0.0;
    roughnessCount_ = 0;
    return result;
  }

  bool available() const { return available_; }
  const char* typeName() const { return typeName_; }

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
  float gravityMagnitude_ = 1.0f;
  double roughnessSumSquares_ = 0.0;
  uint32_t roughnessCount_ = 0;
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
SPIClass sdSpi(FSPI);
Preferences preferences;
ImuSampler imu;
File logFile;
String currentLogPath;

bool sdMounted = false;
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
uint64_t rowsWritten = 0;
uint8_t displayBrightness = 128;
size_t groveBaudIndex = 0;
uint32_t groveBaudStartedMs = 0;
time_t stopStartUtc = 0;
time_t nextStoppedDueUtc = 0;
time_t persistedStopStartUtc = 0;
time_t persistedNextDueUtc = 0;
LogMode mode = LogMode::WaitingForFix;
GpsPreference gpsPreference = GpsPreference::Auto;

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

void syncClockFromGps(GpsReceiver& receiver, uint32_t nowMs) {
  TinyGPSPlus& gps = receiver.parser();
  if (!gps.date.isValid() || !gps.time.isValid() ||
      gps.date.age() > config::kMaxDateTimeAgeMs ||
      gps.time.age() > config::kMaxDateTimeAgeMs ||
      (clockIsReady() && nowMs - lastClockSyncMs < config::kClockResyncIntervalMs)) {
    return;
  }
  const time_t epoch = gpsUtcEpoch(gps);
  if (epoch <= 0) {
    return;
  }
  timeval value{epoch, static_cast<suseconds_t>(gps.time.centisecond()) * 10000};
  settimeofday(&value, nullptr);
  lastClockSyncMs = nowMs;

  if (M5.Rtc.isEnabled()) {
    struct tm utc {};
    gmtime_r(&epoch, &utc);
    M5.Rtc.setDateTime(&utc);
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
  sample.fixValid = sample.positionFresh && sample.speedFresh && dateTimeFresh &&
                    sample.satellitesValid &&
                    sample.satellites >= config::kMinimumSatellites &&
                    sample.hdopValid && sample.hdop <= config::kMaximumHdop;
  return sample;
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

void markSdFailed(uint32_t nowMs) {
  if (logFile) logFile.close();
  currentLogPath = "";
  sdMounted = false;
  lastSdAttemptMs = nowMs;
  SD.end();
}

bool mountSd(uint32_t nowMs, bool force = false) {
  if (sdMounted) return true;
  if (!force && nowMs - lastSdAttemptMs < config::kSdRetryIntervalMs) return false;
  lastSdAttemptMs = nowMs;
  SD.end();
  sdSpi.end();
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
  currentLogPath = requiredPath;
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

bool writeCsvRow(const GpsSnapshot& gpsSample, uint32_t nowMs, time_t nowUtc) {
  if (!clockIsReady() || !openDailyFile(nowUtc, nowMs)) return false;

  const ImuSample imuSample = imu.snapshotAndResetRoughness();
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
  line.field(imuSample.ax, 4, imuSample.valid);
  line.field(imuSample.ay, 4, imuSample.valid);
  line.field(imuSample.az, 4, imuSample.valid);
  line.field(imuSample.gx, 3, imuSample.valid);
  line.field(imuSample.gy, 3, imuSample.valid);
  line.field(imuSample.gz, 3, imuSample.valid);
  line.field(imuSample.pitch, 2, imuSample.valid);
  line.field(imuSample.roll, 2, imuSample.valid);
  line.field(imuSample.gTotal, 4, imuSample.valid);
  line.field(imuSample.roughness, 5, imuSample.valid);
  line.append(",%s,%u,", localTimestamp, gpsSample.fixValid ? 1U : 0U);
  if (gpsSample.fixAgeMs != UINT32_MAX) {
    line.append("%lu", static_cast<unsigned long>(gpsSample.fixAgeMs));
  }
  line.append(",%u,%s,%s,%llu", imuSample.available ? 1U : 0U, imu.typeName(),
              modeName(mode),
              static_cast<unsigned long long>(esp_timer_get_time() / 1000ULL));

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

void handleControls() {
  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) {
    return;
  }

  if (M5Cardputer.Keyboard.isKeyPressed('s')) {
    screenOn = !screenOn;
    if (screenOn) {
      M5Cardputer.Display.wakeup();
      M5Cardputer.Display.setBrightness(displayBrightness);
      lastDisplayMs = 0;
    } else {
      M5Cardputer.Display.sleep();
    }
  } else if (M5Cardputer.Keyboard.isKeyPressed('g')) {
    cycleGpsPreference();
  } else if (M5Cardputer.Keyboard.isKeyPressed('-')) {
    displayBrightness = displayBrightness >= 30 ? displayBrightness - 30 : 0;
    if (screenOn) M5Cardputer.Display.setBrightness(displayBrightness);
  } else if (M5Cardputer.Keyboard.isKeyPressed('=')) {
    displayBrightness = displayBrightness <= 225 ? displayBrightness + 30 : 255;
    if (screenOn) M5Cardputer.Display.setBrightness(displayBrightness);
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

void drawStatus(const GpsSnapshot& sample, uint32_t nowMs, time_t nowUtc) {
  if (!screenOn) return;
  if (nowMs - lastDisplayMs < config::kDisplayIntervalMs) return;
  lastDisplayMs = nowMs;

  auto& display = M5Cardputer.Display;
  display.startWrite();
  display.fillScreen(TFT_BLACK);
  display.setCursor(2, 2);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.printf("JP226 GPS/IMU  %s\n", boardName());
  display.setTextColor(sample.fixValid ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  display.printf("GPS: %s  sats:%lu  HDOP:", sample.fixValid ? "FIX" : "NO FIX",
                 static_cast<unsigned long>(sample.satellites));
  if (sample.hdopValid) display.printf("%.2f", sample.hdop);
  else display.print("-");
  display.println();
  display.printf("Src:%s %lu %s Pref:%s\n", activeGps->name(),
                 static_cast<unsigned long>(activeGps->baud()),
                 activeGps->live(nowMs) ? "OK" : "scan", gpsPreferenceName());
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  if (sample.positionFresh) {
    display.printf("Lat:%.6f\nLon:%.6f\n", sample.latitude, sample.longitude);
  } else {
    display.println("Lat:-\nLon:-");
  }
  display.printf("Speed: %.1f km/h\n", sample.speedFresh ? sample.speedKmh : 0.0);
  display.printf("State: %s\n", modeName(mode));
  display.printf("IMU:%s SD:%s rows:%llu\n", imu.available() ? imu.typeName() : "N/A",
                 sdMounted ? "OK" : "retry",
                 static_cast<unsigned long long>(rowsWritten));
  if (clockIsReady()) {
    struct tm local {};
    localtime_r(&nowUtc, &local);
    display.printf("Local: %04d-%02d-%02d %02d:%02d:%02d\n", local.tm_year + 1900,
                   local.tm_mon + 1, local.tm_mday, local.tm_hour, local.tm_min,
                   local.tm_sec);
  } else {
    display.println("Clock: waiting for GPS UTC");
  }
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.print("LOG:ON G:GPS S:screen -/+:bright");
  display.endWrite();
}

void loadPersistentState() {
  preferences.begin("gpsimu", false);
  const uint8_t savedGpsPreference = preferences.getUChar("gps_src", 0);
  if (savedGpsPreference <= static_cast<uint8_t>(GpsPreference::Cap) &&
      (savedGpsPreference != static_cast<uint8_t>(GpsPreference::Cap) ||
       isCardputerAdv())) {
    gpsPreference = static_cast<GpsPreference>(savedGpsPreference);
  }
  if (preferences.getUChar("state", 0) == 2) {
    persistedStopStartUtc =
        static_cast<time_t>(preferences.getULong64("stop_utc", 0));
    persistedNextDueUtc =
        static_cast<time_t>(preferences.getULong64("next_utc", 0));
    persistedStopPending = persistedStopStartUtc > 0 && persistedNextDueUtc > 0;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  setenv("TZ", config::kPosixTimezone, 1);
  tzset();

  auto m5Config = M5.config();
  M5Cardputer.begin(m5Config, true);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextWrap(false);
  M5Cardputer.Display.setBrightness(displayBrightness);
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  M5Cardputer.Display.setCursor(2, 2);
  M5Cardputer.Display.println("Starting GPS + IMU logger...");

  imu.begin();
  loadPersistentState();

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
  mountSd(millis(), true);
}

void loop() {
  M5Cardputer.update();
  handleControls();

  const uint32_t nowMs = millis();
  updateGpsReceivers(nowMs);
  imu.update(nowMs);
  syncClockFromGps(*activeGps, nowMs);

  const GpsSnapshot gpsSample = takeGpsSnapshot(*activeGps);
  const time_t nowUtc = time(nullptr);
  updateLoggingMode(gpsSample, nowMs, nowUtc);

  if (clockIsReady() && logIsDue(nowMs, nowUtc) &&
      writeCsvRow(gpsSample, nowMs, nowUtc)) {
    onLogSucceeded(nowMs, nowUtc);
  }

  mountSd(nowMs);
  drawStatus(gpsSample, nowMs, nowUtc);
  delay(2);
}
