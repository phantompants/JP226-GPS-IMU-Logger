#include "KmlExporter.h"

#include <cmath>
#include <cstdlib>

#include "Config.h"

namespace {

String baseName(const String& path) {
  const int slash = path.lastIndexOf('/');
  return slash >= 0 ? path.substring(slash + 1) : path;
}

bool validDateText(const String& value) {
  if (value.length() != 10 || value[4] != '-' || value[7] != '-') return false;
  for (size_t index = 0; index < value.length(); ++index) {
    if (index == 4 || index == 7) continue;
    if (value[index] < '0' || value[index] > '9') return false;
  }
  return true;
}

String csvField(const String& line, int wantedIndex) {
  if (wantedIndex < 0) return "";
  String result;
  int fieldIndex = 0;
  bool quoted = false;
  for (size_t index = 0; index <= line.length(); ++index) {
    const char character = index < line.length() ? line[index] : ',';
    if (character == '"') {
      if (quoted && index + 1 < line.length() && line[index + 1] == '"') {
        if (fieldIndex == wantedIndex) result += '"';
        ++index;
      } else {
        quoted = !quoted;
      }
    } else if (character == ',' && !quoted) {
      if (fieldIndex == wantedIndex) {
        result.trim();
        return result;
      }
      ++fieldIndex;
    } else if (fieldIndex == wantedIndex) {
      result += character;
    }
  }
  return "";
}

int findColumn(const String& header, const char* name) {
  for (int index = 0; index < 64; ++index) {
    const String field = csvField(header, index);
    if (field == name) return index;
    if (field.isEmpty() && index > 0 && header.endsWith(",") == false) break;
  }
  return -1;
}

bool parseNumber(String value, double& result) {
  value.trim();
  if (value.isEmpty()) return false;
  char* end = nullptr;
  result = strtod(value.c_str(), &end);
  if (end == value.c_str()) return false;
  while (*end == ' ' || *end == '\t' || *end == '\r') ++end;
  return *end == '\0' && std::isfinite(result);
}

}  // namespace

void KmlExporter::begin(fs::FS& storage, const char* logDirectory,
                        const char* filePrefix, const char* kmlDirectory) {
  storage_ = &storage;
  logDirectory_ = logDirectory;
  filePrefix_ = filePrefix;
  kmlDirectory_ = kmlDirectory;
}

void KmlExporter::requestScan() {
  scanRequested_ = true;
  completedCount_ = 0;
  failedCount_ = 0;
  if (phase_ == Phase::Done) phase_ = Phase::Idle;
  lastMessage_ = "Scan requested";
}

void KmlExporter::queueCompletedFile(const String& csvPath) {
  if (queuePath(csvPath)) {
    lastMessage_ = "Completed day queued";
    if (phase_ == Phase::Done) phase_ = Phase::Idle;
  }
}

void KmlExporter::invalidateForCsv(const String& csvPath) {
  if (storage_ == nullptr) return;
  const String name = baseName(csvPath);
  if (!name.startsWith(filePrefix_) || !name.endsWith(".csv")) return;
  const String stem = name.substring(0, name.length() - 4);
  const String kmlPath = kmlDirectory_ + "/" + stem + ".kml";
  const String temporaryPath = kmlPath + ".tmp";
  if (storage_->exists(kmlPath)) storage_->remove(kmlPath);
  if (storage_->exists(temporaryPath)) storage_->remove(temporaryPath);
}

const char* KmlExporter::phaseName() const {
  switch (phase_) {
    case Phase::Scanning:
      return "SCANNING";
    case Phase::Exporting:
      return "CONVERTING";
    case Phase::Done:
      return failedCount_ > 0 ? "DONE / ERRORS" : "DONE";
    default:
      return "IDLE";
  }
}

void KmlExporter::update(const String& activeCsvPath, const String& localDate) {
  if (storage_ == nullptr) return;

  if (phase_ == Phase::Exporting) {
    if (currentCsvPath_ == activeCsvPath) {
      failCurrent("CSV reopened; export deferred");
      return;
    }
    exportBatch();
    return;
  }
  if (phase_ == Phase::Scanning) {
    scanOne(activeCsvPath, localDate);
    return;
  }
  if (pendingCount_ > 0) {
    if (!beginNextExport(activeCsvPath, localDate) && pendingCount_ == 0) {
      finishPass();
    }
    return;
  }
  if (scanRequested_) beginScan();
}

void KmlExporter::beginScan() {
  scanRequested_ = false;
  if (scanDirectory_) scanDirectory_.close();
  scanDirectory_ = storage_->open(logDirectory_);
  if (!scanDirectory_ || !scanDirectory_.isDirectory()) {
    ++failedCount_;
    lastMessage_ = "Cannot open telemetry folder";
    phase_ = Phase::Done;
    return;
  }
  phase_ = Phase::Scanning;
  currentFile_ = "Checking daily CSV files";
  lastMessage_ = "Looking for missing KML files";
}

void KmlExporter::scanOne(const String& activeCsvPath,
                          const String& localDate) {
  File entry = scanDirectory_.openNextFile();
  if (!entry) {
    scanDirectory_.close();
    if (pendingCount_ == 0) {
      finishPass();
    } else {
      phase_ = Phase::Idle;
    }
    return;
  }

  const bool directory = entry.isDirectory();
  String entryName = entry.name();
  entry.close();
  if (directory) return;

  const String name = baseName(entryName);
  const String suffix = ".csv";
  if (!name.startsWith(filePrefix_) || !name.endsWith(suffix)) return;
  const size_t expectedLength = filePrefix_.length() + 10 + suffix.length();
  if (name.length() != expectedLength) return;
  const String fileDate = name.substring(filePrefix_.length(),
                                         filePrefix_.length() + 10);
  if (!validDateText(fileDate) || !validDateText(localDate) ||
      fileDate >= localDate) {
    return;
  }

  String fullPath = entryName;
  if (!fullPath.startsWith("/")) fullPath = logDirectory_ + "/" + name;
  if (fullPath == activeCsvPath) return;

  const String kmlPath = kmlDirectory_ + "/" +
                         name.substring(0, name.length() - suffix.length()) +
                         ".kml";
  if (!storage_->exists(kmlPath)) queuePath(fullPath);
}

bool KmlExporter::queuePath(const String& csvPath) {
  if (csvPath.isEmpty() || pendingCount_ >= kMaximumQueuedFiles) {
    if (pendingCount_ >= kMaximumQueuedFiles) {
      ++failedCount_;
      lastMessage_ = "Too many CSV files; run again";
    }
    return false;
  }
  if (csvPath == currentCsvPath_) return false;
  for (size_t index = 0; index < pendingCount_; ++index) {
    if (pending_[index] == csvPath) return false;
  }
  pending_[pendingCount_++] = csvPath;
  return true;
}

bool KmlExporter::beginNextExport(const String& activeCsvPath,
                                  const String& localDate) {
  while (pendingCount_ > 0) {
    const String csvPath = pending_[0];
    for (size_t index = 1; index < pendingCount_; ++index) {
      pending_[index - 1] = pending_[index];
    }
    pending_[--pendingCount_] = "";

    const String name = baseName(csvPath);
    const String suffix = ".csv";
    if (!name.startsWith(filePrefix_) || !name.endsWith(suffix)) continue;
    const String fileDate = name.substring(filePrefix_.length(),
                                           filePrefix_.length() + 10);
    if (csvPath == activeCsvPath || !validDateText(fileDate) ||
        !validDateText(localDate) || fileDate >= localDate) {
      continue;
    }

    if (!storage_->exists(kmlDirectory_) &&
        !storage_->mkdir(kmlDirectory_)) {
      currentFile_ = name;
      failCurrent("Cannot create KML folder");
      continue;
    }

    outputPath_ = kmlDirectory_ + "/" +
                  name.substring(0, name.length() - suffix.length()) + ".kml";
    temporaryPath_ = outputPath_ + ".tmp";
    if (storage_->exists(outputPath_)) continue;
    if (storage_->exists(temporaryPath_)) storage_->remove(temporaryPath_);

    input_ = storage_->open(csvPath, FILE_READ);
    if (!input_) {
      currentFile_ = name;
      failCurrent("Cannot read CSV");
      continue;
    }
    input_.setTimeout(20);
    String header = input_.readStringUntil('\n');
    header.trim();
    latitudeColumn_ = findColumn(header, "lat");
    longitudeColumn_ = findColumn(header, "lon");
    fixValidColumn_ = findColumn(header, "fix_valid");
    if (latitudeColumn_ < 0 || longitudeColumn_ < 0) {
      input_.close();
      currentFile_ = name;
      failCurrent("CSV has no lat/lon columns");
      continue;
    }

    output_ = storage_->open(temporaryPath_, FILE_WRITE);
    if (!output_) {
      input_.close();
      currentFile_ = name;
      failCurrent("Cannot create temporary KML");
      continue;
    }

    currentCsvPath_ = csvPath;
    currentFile_ = name;
    pointsWritten_ = 0;
    firstCoordinate_ = "";
    hadWriteFailure_ = false;
    const String documentName = "JP226 GPS track " + fileDate;
    const String headerText =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n"
        "<name>" + documentName + "</name>\n"
        "<Style id=\"track\"><LineStyle><color>ff00a5ff</color>"
        "<width>4</width></LineStyle></Style>\n"
        "<Placemark><name>" + fileDate + " GPS track</name>"
        "<styleUrl>#track</styleUrl><LineString><tessellate>1</tessellate>"
        "<altitudeMode>clampToGround</altitudeMode><coordinates>\n";
    if (output_.print(headerText) != headerText.length()) {
      failCurrent("KML write failed");
      continue;
    }
    phase_ = Phase::Exporting;
    lastMessage_ = "Streaming CSV from SD";
    return true;
  }
  return false;
}

void KmlExporter::exportBatch() {
  for (uint16_t lineIndex = 0;
       lineIndex < config::kKmlLinesPerUpdate && input_.available();
       ++lineIndex) {
    String line = input_.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith("timestamp,")) continue;
    double latitude = 0.0;
    double longitude = 0.0;
    if (!parseCoordinate(line, latitude, longitude)) continue;

    char coordinate[64]{};
    snprintf(coordinate, sizeof(coordinate), "%.7f,%.7f,0\n", longitude,
             latitude);
    if (pointsWritten_ == 0) firstCoordinate_ = coordinate;
    if (!writeText(coordinate)) {
      failCurrent("KML write failed");
      return;
    }
    ++pointsWritten_;
    if (pointsWritten_ % config::kKmlFlushEveryPoints == 0) output_.flush();
  }

  if (!input_.available()) finishExport();
}

void KmlExporter::finishExport() {
  input_.close();
  if (pointsWritten_ == 0) {
    output_.close();
    storage_->remove(temporaryPath_);
    if (!writeEmptyDocument()) {
      failCurrent("Cannot write empty KML");
      return;
    }
  } else {
    if (pointsWritten_ == 1 && output_.print(firstCoordinate_) !=
                                   firstCoordinate_.length()) {
      failCurrent("KML write failed");
      return;
    }
    if (!writeText("</coordinates></LineString></Placemark>\n"
                   "</Document>\n</kml>\n")) {
      failCurrent("KML write failed");
      return;
    }
    output_.flush();
    output_.close();
  }

  if (!storage_->rename(temporaryPath_, outputPath_)) {
    failCurrent("Cannot finalize KML");
    return;
  }
  ++completedCount_;
  lastMessage_ = pointsWritten_ == 0 ? "Saved (no valid GPS points)"
                                     : "KML saved safely";
  currentCsvPath_ = "";
  phase_ = Phase::Idle;
  if (pendingCount_ == 0 && !scanRequested_) finishPass();
}

void KmlExporter::failCurrent(const String& message) {
  if (input_) input_.close();
  if (output_) output_.close();
  if (storage_ != nullptr && !temporaryPath_.isEmpty() &&
      storage_->exists(temporaryPath_)) {
    storage_->remove(temporaryPath_);
  }
  ++failedCount_;
  lastMessage_ = message;
  currentCsvPath_ = "";
  phase_ = Phase::Idle;
}

void KmlExporter::finishPass() {
  phase_ = Phase::Done;
  if (completedCount_ == 0 && failedCount_ == 0) {
    lastMessage_ = "All finished days are current";
    currentFile_ = "No export needed";
  } else if (failedCount_ > 0) {
    lastMessage_ = "Finished with export errors";
  }
}

bool KmlExporter::writeText(const char* text) {
  const size_t length = strlen(text);
  return output_ && output_.write(
                        reinterpret_cast<const uint8_t*>(text), length) == length;
}

bool KmlExporter::writeEmptyDocument() {
  output_ = storage_->open(temporaryPath_, FILE_WRITE);
  if (!output_) return false;
  const String name = baseName(currentCsvPath_);
  const String text =
      "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      "<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>\n"
      "<name>" + name + "</name>\n"
      "<description>No valid GPS positions were present in this daily CSV."
      "</description>\n</Document></kml>\n";
  const bool ok = output_.print(text) == text.length();
  output_.flush();
  output_.close();
  return ok;
}

bool KmlExporter::parseCoordinate(const String& line, double& latitude,
                                  double& longitude) const {
  if (fixValidColumn_ >= 0) {
    String fix = csvField(line, fixValidColumn_);
    fix.trim();
    if (fix != "1" && !fix.equalsIgnoreCase("true")) return false;
  }
  if (!parseNumber(csvField(line, latitudeColumn_), latitude) ||
      !parseNumber(csvField(line, longitudeColumn_), longitude)) {
    return false;
  }
  return latitude >= -90.0 && latitude <= 90.0 && longitude >= -180.0 &&
         longitude <= 180.0;
}
