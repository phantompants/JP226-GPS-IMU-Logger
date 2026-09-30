#pragma once

#include <Arduino.h>
#include <FS.h>
#include <WebServer.h>

struct WebPortalStatus {
  bool sdMounted = false;
  bool stationary = false;
  bool fixValid = false;
  double speedKmh = 0.0;
  String logState;
  String activeCsvPath;
  String timezone;
  String kmlPhase;
  String kmlMessage;
  uint16_t kmlQueued = 0;
  uint16_t kmlCompleted = 0;
  uint16_t kmlFailed = 0;
  uint32_t kmlPoints = 0;
};

class WebPortal {
 public:
  void begin(fs::FS& storage, const char* logDirectory,
             const char* kmlDirectory, const char* hostname);
  void update(const WebPortalStatus& status);
  bool takeConversionRequest();

  bool running() const { return serverRunning_; }
  String address() const;

 private:
  struct FileItem {
    String path;
    String name;
    size_t size = 0;
    bool kml = false;
    bool active = false;
  };

  static constexpr size_t kMaximumListedFiles = 64;

  void registerRoutes();
  void startServer();
  void stopServer();
  void sendHomePage();
  void sendStatusJson();
  void handleConvert();
  void handleDownload();
  void addFilesFrom(const String& directory, const char* extension,
                    bool kml, FileItem* items, size_t& count);
  bool validDownloadPath(const String& path) const;
  void applyResponseHeaders();

  static String htmlEscape(const String& value);
  static String jsonEscape(const String& value);
  static String urlEncode(const String& value);
  static String baseName(const String& path);
  static String formatBytes(size_t bytes);
  static String localClockText();

  fs::FS* storage_ = nullptr;
  String logDirectory_;
  String kmlDirectory_;
  String hostname_;
  WebServer server_{80};
  WebPortalStatus status_;
  bool routesRegistered_ = false;
  bool serverRunning_ = false;
  bool mdnsRunning_ = false;
  bool conversionRequested_ = false;
};
