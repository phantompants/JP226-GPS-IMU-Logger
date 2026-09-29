#include "WebPortal.h"

#include <ESPmDNS.h>
#include <WiFi.h>
#include <time.h>

#include <cstring>

namespace {

constexpr char kPageHead[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0b1b16">
<title>JP226 Field Log</title>
<style>
:root{--bush:#0b1b16;--bush2:#142a22;--paper:#eef1e8;--mist:#a9b9ae;--track:#f3ad4f;--river:#68b9c8;--danger:#f27b67;--line:#315044;--shadow:rgba(0,0,0,.24)}
*{box-sizing:border-box}html{background:var(--bush)}body{margin:0;color:var(--paper);background:var(--bush);font-family:"Avenir Next","SF Pro Display",-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;-webkit-font-smoothing:antialiased}
body:before{content:"";position:fixed;inset:0;pointer-events:none;opacity:.17;background-image:url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='320' height='180' viewBox='0 0 320 180'%3E%3Cpath d='M-20 128C42 69 70 174 139 112S243 45 350 108M-30 84C38 31 82 127 151 73S263 10 350 62' fill='none' stroke='%2393b7a1' stroke-width='1'/%3E%3C/svg%3E")}
.shell{position:relative;width:min(1180px,100%);margin:auto;padding:calc(18px + env(safe-area-inset-top)) max(18px,env(safe-area-inset-right)) calc(32px + env(safe-area-inset-bottom)) max(18px,env(safe-area-inset-left))}
.hero{display:grid;grid-template-columns:1fr auto;gap:18px;align-items:end;padding:22px 0 24px;border-bottom:2px solid var(--track)}
.brand{margin:0;font-size:clamp(2rem,8vw,4.5rem);line-height:.9;letter-spacing:-.055em;font-weight:750;max-width:700px}.sub{color:var(--mist);margin:12px 0 0;font-size:1rem}.speed{text-align:right}.speed strong{font-size:clamp(3.2rem,13vw,7.2rem);line-height:.72;font-variant-numeric:tabular-nums;letter-spacing:-.08em}.speed span{display:block;color:var(--track);font-weight:700;margin-top:12px}
.status-rail{display:grid;grid-template-columns:repeat(4,1fr);border-bottom:1px solid var(--line)}.status{padding:18px 14px 18px 0}.status+ .status{padding-left:18px;border-left:1px solid var(--line)}.status span{display:block;color:var(--mist);font-size:.78rem;margin-bottom:4px}.status b{font-size:1.05rem}.good{color:#8bd6a5}.warn{color:var(--track)}.bad{color:var(--danger)}
.main{display:grid;grid-template-columns:minmax(0,1fr) 310px;gap:28px;margin-top:28px}.section-head{display:flex;justify-content:space-between;align-items:end;gap:16px;margin-bottom:14px}.section-head h2{font-size:1.55rem;margin:0;letter-spacing:-.025em}.section-head p{margin:0;color:var(--mist);font-size:.88rem;text-align:right}.ledger{border-top:1px solid var(--line)}.file{display:grid;grid-template-columns:54px minmax(0,1fr) auto;gap:14px;align-items:center;padding:15px 0;border-bottom:1px solid var(--line)}.stamp{width:46px;height:46px;display:grid;place-items:center;border:1px solid var(--line);border-radius:50%;color:var(--river);font-size:.68rem;font-weight:800}.stamp.kml{color:var(--track)}.filename{font-weight:650;overflow-wrap:anywhere}.meta{font-size:.82rem;color:var(--mist);margin-top:3px}.active{color:var(--track)}
.button{appearance:none;border:0;background:var(--track);color:#172016;border-radius:12px;min-height:50px;padding:0 18px;display:inline-flex;align-items:center;justify-content:center;text-decoration:none;font:inherit;font-weight:750;cursor:pointer}.button.secondary{background:transparent;color:var(--paper);border:1px solid var(--line)}.button[aria-disabled="true"]{opacity:.38;pointer-events:none}.button:focus-visible{outline:3px solid var(--river);outline-offset:3px}
.panel{background:var(--bush2);border-left:4px solid var(--track);padding:20px}.panel h2{font-size:1.35rem;margin:0 0 10px}.panel p{color:var(--mist);line-height:1.5;margin:0 0 18px}.panel .button{width:100%}.facts{margin-top:18px;border-top:1px solid var(--line)}.fact{display:flex;justify-content:space-between;gap:12px;padding:11px 0;border-bottom:1px solid var(--line);font-size:.86rem}.fact span{color:var(--mist)}.empty{padding:28px 0;color:var(--mist);line-height:1.5}.foot{margin-top:28px;color:var(--mist);font-size:.8rem;display:flex;justify-content:space-between;gap:14px}.toast{padding:12px 14px;border:1px solid var(--track);color:var(--track);margin-bottom:18px}
@media(max-width:760px){.shell{padding-left:max(15px,env(safe-area-inset-left));padding-right:max(15px,env(safe-area-inset-right))}.hero{grid-template-columns:1fr;align-items:start}.speed{text-align:left;display:flex;align-items:end;gap:12px}.speed span{margin:0 0 2px}.status-rail{grid-template-columns:1fr 1fr}.status:nth-child(3){border-left:0}.status:nth-child(n+3){border-top:1px solid var(--line)}.main{grid-template-columns:1fr}.side{order:-1}.section-head{align-items:start;flex-direction:column}.section-head p{text-align:left}.file{grid-template-columns:46px minmax(0,1fr)}.file .button{grid-column:1/-1;width:100%}.foot{flex-direction:column}}
@media(min-width:900px){.shell{padding-left:32px;padding-right:32px}.panel{position:sticky;top:22px}}
@media(prefers-reduced-motion:no-preference){.speed strong{transition:color .2s ease}}
</style></head><body><main class="shell">
)HTML";

constexpr char kPageTail[] = R"HTML(
<footer class="foot"><span>JP226 GPS + IMU Logger</span><span>Read-only file portal · local network only</span></footer>
</main><script>
const set=(id,v)=>{const e=document.getElementById(id);if(e)e.textContent=v};
async function live(){try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)return;const s=await r.json();set('speed',s.speed);set('clock',s.clock);set('gps',s.gps);set('log',s.log);set('sd',s.sd);set('kml',s.kml)}catch(e){}}
setInterval(live,5000);
</script></body></html>)HTML";

}  // namespace

void WebPortal::begin(fs::FS& storage, const char* logDirectory,
                      const char* kmlDirectory, const char* hostname) {
  storage_ = &storage;
  logDirectory_ = logDirectory;
  kmlDirectory_ = kmlDirectory;
  hostname_ = hostname;
  registerRoutes();
}

void WebPortal::update(const WebPortalStatus& status) {
  status_ = status;
  if (WiFi.status() == WL_CONNECTED) {
    if (!serverRunning_) startServer();
    server_.handleClient();
  } else if (serverRunning_) {
    stopServer();
  }
}

bool WebPortal::takeConversionRequest() {
  const bool requested = conversionRequested_;
  conversionRequested_ = false;
  return requested;
}

String WebPortal::address() const {
  if (!serverRunning_) return "Offline";
  return "http://" + hostname_ + ".local";
}

void WebPortal::registerRoutes() {
  if (routesRegistered_) return;
  server_.on("/", HTTP_GET, [this]() { sendHomePage(); });
  server_.on("/api/status", HTTP_GET, [this]() { sendStatusJson(); });
  server_.on("/convert", HTTP_POST, [this]() { handleConvert(); });
  server_.on("/download", HTTP_GET, [this]() { handleDownload(); });
  server_.onNotFound([this]() {
    applyResponseHeaders();
    server_.send(404, "text/plain", "Page not found");
  });
  routesRegistered_ = true;
}

void WebPortal::startServer() {
  server_.begin();
  serverRunning_ = true;
  mdnsRunning_ = MDNS.begin(hostname_);
  if (mdnsRunning_) {
    MDNS.setInstanceName("JP226 GPS IMU Logger");
    MDNS.addService("http", "tcp", 80);
  }
  Serial.printf("Web portal: http://%s.local or http://%s\n", hostname_.c_str(),
                WiFi.localIP().toString().c_str());
}

void WebPortal::stopServer() {
  server_.stop();
  if (mdnsRunning_) MDNS.end();
  mdnsRunning_ = false;
  serverRunning_ = false;
}

void WebPortal::sendHomePage() {
  applyResponseHeaders();
  server_.chunkResponseBegin("text/html; charset=utf-8");
  server_.chunkWrite(kPageHead, strlen(kPageHead));

  const String speed = status_.fixValid ? String(status_.speedKmh, 1) : "--";
  const String clock = localClockText();
  const String gpsClass = status_.fixValid ? "good" : "warn";
  const String sdClass = status_.sdMounted ? "good" : "bad";
  const String stopClass = status_.stationary ? "good" : "warn";
  String hero;
  hero.reserve(2300);
  hero += "<header class=\"hero\"><div><h1 class=\"brand\">Field log, ready when you are.</h1>";
  hero += "<p class=\"sub\"><span id=\"clock\">" + htmlEscape(clock) +
          "</span> · " + htmlEscape(status_.timezone) + " · " +
          htmlEscape(WiFi.SSID()) + "</p></div><div class=\"speed\"><strong id=\"speed\">" +
          speed + "</strong><span>km/h</span></div></header>";
  hero += "<section class=\"status-rail\" aria-label=\"Logger status\">";
  hero += "<div class=\"status\"><span>GPS</span><b id=\"gps\" class=\"" +
          gpsClass + "\">" + (status_.fixValid ? "Fix ready" : "Waiting") +
          "</b></div>";
  hero += "<div class=\"status\"><span>Logging</span><b id=\"log\">" +
          htmlEscape(status_.logState) + "</b></div>";
  hero += "<div class=\"status\"><span>Storage</span><b id=\"sd\" class=\"" +
          sdClass + "\">" + (status_.sdMounted ? "SD ready" : "SD unavailable") +
          "</b></div>";
  hero += "<div class=\"status\"><span>File work</span><b id=\"kml\" class=\"" +
          stopClass + "\">" +
          (status_.stationary ? htmlEscape(status_.kmlPhase) : "Paused while moving") +
          "</b></div></section><div class=\"main\"><section>";
  hero += "<div class=\"section-head\"><h2>Daily files</h2><p>Newest first · CSV and Google KML</p></div>";
  server_.chunkWrite(hero.c_str(), hero.length());

  FileItem items[kMaximumListedFiles];
  size_t count = 0;
  if (status_.sdMounted && storage_ != nullptr) {
    addFilesFrom(logDirectory_, ".csv", false, items, count);
    addFilesFrom(kmlDirectory_, ".kml", true, items, count);
  }
  for (size_t left = 0; left < count; ++left) {
    for (size_t right = left + 1; right < count; ++right) {
      if (items[right].name > items[left].name) {
        FileItem temporary = items[left];
        items[left] = items[right];
        items[right] = temporary;
      }
    }
  }

  constexpr char ledgerStart[] = "<div class=\"ledger\">";
  server_.chunkWrite(ledgerStart, strlen(ledgerStart));
  if (count == 0) {
    const char empty[] =
        "<div class=\"empty\">No downloadable files yet. The first CSV appears after the logger has a valid clock and a record is due.</div>";
    server_.chunkWrite(empty, strlen(empty));
  }
  for (size_t index = 0; index < count; ++index) {
    const FileItem& item = items[index];
    String row;
    row.reserve(900);
    row += "<article class=\"file\"><div class=\"stamp";
    if (item.kml) row += " kml";
    row += "\">" + String(item.kml ? "KML" : "CSV") + "</div><div>";
    row += "<div class=\"filename\">" + htmlEscape(item.name) + "</div>";
    row += "<div class=\"meta\">" + formatBytes(item.size);
    if (item.active) row += " · <span class=\"active\">today’s active file</span>";
    row += "</div></div>";
    if (status_.stationary && status_.sdMounted) {
      row += "<a class=\"button secondary\" download href=\"/download?path=" +
             urlEncode(item.path) + "\">Download</a>";
    } else {
      row += "<span class=\"button secondary\" aria-disabled=\"true\">Stop to download</span>";
    }
    row += "</article>";
    server_.chunkWrite(row.c_str(), row.length());
  }

  String side;
  side.reserve(1900);
  side += "</div></section><aside class=\"side\"><div class=\"panel\"><h2>Build missing KML routes</h2>";
  if (status_.stationary) {
    side += "<p>The vehicle is stationary. Conversion can run now without competing with moving telemetry.</p>";
  } else {
    side += "<p>The request can be queued now. Conversion will wait until GPS confirms the vehicle is stationary.</p>";
  }
  side += "<form method=\"post\" action=\"/convert\"><button class=\"button\" type=\"submit\">Convert finished days</button></form>";
  side += "<div class=\"facts\"><div class=\"fact\"><span>KML state</span><b>" +
          htmlEscape(status_.kmlPhase) + "</b></div>";
  side += "<div class=\"fact\"><span>Queued</span><b>" +
          String(status_.kmlQueued) + "</b></div>";
  side += "<div class=\"fact\"><span>Completed</span><b>" +
          String(status_.kmlCompleted) + "</b></div>";
  side += "<div class=\"fact\"><span>Errors</span><b>" +
          String(status_.kmlFailed) + "</b></div>";
  side += "<div class=\"fact\"><span>Address</span><b>" +
          htmlEscape(WiFi.localIP().toString()) + "</b></div></div></div></aside></div>";
  server_.chunkWrite(side.c_str(), side.length());
  server_.chunkWrite(kPageTail, strlen(kPageTail));
  server_.chunkResponseEnd();
}

void WebPortal::sendStatusJson() {
  applyResponseHeaders();
  const String speed = status_.fixValid ? String(status_.speedKmh, 1) : "--";
  String json;
  json.reserve(420);
  json = "{\"speed\":\"" + jsonEscape(speed) + "\",\"clock\":\"" +
         jsonEscape(localClockText()) + "\",\"gps\":\"" +
         String(status_.fixValid ? "Fix ready" : "Waiting") +
         "\",\"log\":\"" + jsonEscape(status_.logState) +
         "\",\"sd\":\"" +
         String(status_.sdMounted ? "SD ready" : "SD unavailable") +
         "\",\"kml\":\"" +
         jsonEscape(status_.stationary ? status_.kmlPhase
                                       : "Paused while moving") +
         "\"}";
  server_.send(200, "application/json", json);
}

void WebPortal::handleConvert() {
  conversionRequested_ = true;
  applyResponseHeaders();
  server_.sendHeader("Location", "/", true);
  server_.send(303, "text/plain", "KML conversion queued");
}

void WebPortal::handleDownload() {
  applyResponseHeaders();
  if (!status_.stationary) {
    server_.send(409, "text/plain",
                 "Downloads are paused while the vehicle is moving. Stop safely and try again.");
    return;
  }
  if (!status_.sdMounted || storage_ == nullptr) {
    server_.send(503, "text/plain", "The SD card is unavailable.");
    return;
  }
  const String path = server_.arg("path");
  if (!validDownloadPath(path) || !storage_->exists(path)) {
    server_.send(404, "text/plain", "File not found.");
    return;
  }
  File file = storage_->open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    if (file) file.close();
    server_.send(404, "text/plain", "File not found.");
    return;
  }
  const String name = baseName(path);
  server_.sendHeader("Content-Disposition",
                     "attachment; filename=\"" + name + "\"");
  const String contentType = path.endsWith(".kml")
                                 ? "application/vnd.google-earth.kml+xml"
                                 : "text/csv; charset=utf-8";
  server_.streamFile(file, contentType);
  file.close();
}

void WebPortal::addFilesFrom(const String& directory, const char* extension,
                             bool kml, FileItem* items, size_t& count) {
  if (count >= kMaximumListedFiles || !storage_->exists(directory)) return;
  File folder = storage_->open(directory);
  if (!folder || !folder.isDirectory()) return;
  while (count < kMaximumListedFiles) {
    File entry = folder.openNextFile();
    if (!entry) break;
    const bool isDirectory = entry.isDirectory();
    const String rawName = entry.name();
    const size_t fileSize = entry.size();
    entry.close();
    if (isDirectory) continue;
    const String name = baseName(rawName);
    if (!name.endsWith(extension) || !name.startsWith("telemetry_")) continue;
    String path = rawName;
    if (!path.startsWith("/")) path = directory + "/" + name;
    items[count].path = path;
    items[count].name = name;
    items[count].size = fileSize;
    items[count].kml = kml;
    items[count].active = path == status_.activeCsvPath;
    ++count;
  }
  folder.close();
}

bool WebPortal::validDownloadPath(const String& path) const {
  if (!path.startsWith(logDirectory_ + "/") || path.indexOf("..") >= 0 ||
      path.indexOf('\\') >= 0) {
    return false;
  }
  const String name = baseName(path);
  if (!name.startsWith("telemetry_")) return false;
  if (path.endsWith(".csv")) {
    return path.lastIndexOf('/') == static_cast<int>(logDirectory_.length());
  }
  return path.endsWith(".kml") && path.startsWith(kmlDirectory_ + "/") &&
         path.lastIndexOf('/') == static_cast<int>(kmlDirectory_.length());
}

void WebPortal::applyResponseHeaders() {
  server_.sendHeader("Cache-Control", "no-store");
  server_.sendHeader("X-Content-Type-Options", "nosniff");
  server_.sendHeader("X-Frame-Options", "DENY");
  server_.sendHeader("Referrer-Policy", "no-referrer");
  server_.sendHeader(
      "Content-Security-Policy",
      "default-src 'self'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; img-src data:");
}

String WebPortal::htmlEscape(const String& value) {
  String escaped;
  escaped.reserve(value.length() + 12);
  for (size_t index = 0; index < value.length(); ++index) {
    switch (value[index]) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      case '"':
        escaped += "&quot;";
        break;
      case '\'':
        escaped += "&#39;";
        break;
      default:
        escaped += value[index];
    }
  }
  return escaped;
}

String WebPortal::jsonEscape(const String& value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  for (size_t index = 0; index < value.length(); ++index) {
    const char character = value[index];
    if (character == '"' || character == '\\') escaped += '\\';
    if (character == '\n') {
      escaped += "\\n";
    } else if (character != '\r') {
      escaped += character;
    }
  }
  return escaped;
}

String WebPortal::urlEncode(const String& value) {
  constexpr char hexadecimal[] = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() + 12);
  for (size_t index = 0; index < value.length(); ++index) {
    const uint8_t character = static_cast<uint8_t>(value[index]);
    if ((character >= 'a' && character <= 'z') ||
        (character >= 'A' && character <= 'Z') ||
        (character >= '0' && character <= '9') || character == '-' ||
        character == '_' || character == '.' || character == '~') {
      encoded += static_cast<char>(character);
    } else {
      encoded += '%';
      encoded += hexadecimal[character >> 4];
      encoded += hexadecimal[character & 0x0F];
    }
  }
  return encoded;
}

String WebPortal::baseName(const String& path) {
  const int slash = path.lastIndexOf('/');
  return slash >= 0 ? path.substring(slash + 1) : path;
}

String WebPortal::formatBytes(size_t bytes) {
  if (bytes >= 1024U * 1024U) {
    return String(static_cast<double>(bytes) / (1024.0 * 1024.0), 1) + " MB";
  }
  if (bytes >= 1024U) {
    return String(static_cast<double>(bytes) / 1024.0, 1) + " KB";
  }
  return String(bytes) + " bytes";
}

String WebPortal::localClockText() {
  const time_t now = time(nullptr);
  struct tm local {};
  if (now <= 0 || localtime_r(&now, &local) == nullptr ||
      local.tm_year + 1900 < 2024) {
    return "Clock not ready";
  }
  char result[32]{};
  strftime(result, sizeof(result), "%a %d %b · %H:%M:%S", &local);
  return String(result);
}
