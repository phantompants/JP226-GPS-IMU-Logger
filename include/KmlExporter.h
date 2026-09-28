#pragma once

#include <Arduino.h>
#include <FS.h>

class KmlExporter {
 public:
  enum class Phase : uint8_t {
    Idle = 0,
    Scanning,
    Exporting,
    Done,
  };

  void begin(fs::FS& storage, const char* logDirectory,
             const char* filePrefix, const char* kmlDirectory);
  void requestScan();
  void queueCompletedFile(const String& csvPath);
  void invalidateForCsv(const String& csvPath);
  void update(const String& activeCsvPath, const String& localDate);

  Phase phase() const { return phase_; }
  const char* phaseName() const;
  const String& currentFile() const { return currentFile_; }
  const String& lastMessage() const { return lastMessage_; }
  uint32_t pointsWritten() const { return pointsWritten_; }
  uint16_t queuedCount() const { return pendingCount_; }
  uint16_t completedCount() const { return completedCount_; }
  uint16_t failedCount() const { return failedCount_; }

 private:
  static constexpr size_t kMaximumQueuedFiles = 64;

  void beginScan();
  void scanOne(const String& activeCsvPath, const String& localDate);
  bool queuePath(const String& csvPath);
  bool beginNextExport(const String& activeCsvPath, const String& localDate);
  void exportBatch();
  void finishExport();
  void failCurrent(const String& message);
  void finishPass();
  bool writeText(const char* text);
  bool writeEmptyDocument();
  bool parseCoordinate(const String& line, double& latitude,
                       double& longitude) const;

  fs::FS* storage_ = nullptr;
  String logDirectory_;
  String filePrefix_;
  String kmlDirectory_;
  File scanDirectory_;
  File input_;
  File output_;
  String pending_[kMaximumQueuedFiles];
  size_t pendingCount_ = 0;
  String currentCsvPath_;
  String currentFile_;
  String outputPath_;
  String temporaryPath_;
  String firstCoordinate_;
  String lastMessage_ = "Waiting";
  int latitudeColumn_ = -1;
  int longitudeColumn_ = -1;
  int fixValidColumn_ = -1;
  uint32_t pointsWritten_ = 0;
  uint16_t completedCount_ = 0;
  uint16_t failedCount_ = 0;
  bool scanRequested_ = false;
  bool hadWriteFailure_ = false;
  Phase phase_ = Phase::Idle;
};
