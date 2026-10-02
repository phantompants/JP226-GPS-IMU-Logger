// M5Stack Core2 touch UI. Included into main.cpp's anonymous namespace in
// place of the Cardputer keyboard UI, so it shares the logger state directly.
//
// Screen: 320x240 with a title bar and a footer. The three touch buttons
// under the screen are BtnA = previous page, BtnB = the page's action (shown
// in the footer) and BtnC = next page. Holding BtnB turns the screen off; any
// touch turns it back on. Some pages also have on-screen touch buttons.

enum class Core2Page : uint8_t {
  Drive = 0,
  Hud,
  Gps,
  Imu,
  Waypoint,
  Logger,
  Time,
  Setup,
  Weather,
  Version,
  About,
  Count,
};

constexpr const char* kCore2PageTitles[] = {
    "DRIVE",  "HUD",  "GPS",   "IMU",     "WAYPOINT",      "LOGGER",
    "TIME",   "SETUP", "WEATHER", "FOSSIL RECORD", "JP226PRINTS"};

// Same categories as the M5Dial; the labels are shortened to fit a button.
constexpr const char* kWaypointCategories[] = {
    "GENERIC", "PHOTO",  "CAMP",        "FUEL",      "LOOKOUT",
    "TRACK",   "HAZARD", "INTERESTING", "TEST POINT"};
constexpr const char* kWaypointLabels[] = {
    "GENERIC", "PHOTO",  "CAMP",     "FUEL",   "LOOKOUT",
    "TRACK",   "HAZARD", "INTEREST", "TEST PT"};
constexpr size_t kWaypointCategoryCount =
    sizeof(kWaypointCategories) / sizeof(kWaypointCategories[0]);

const lgfx::IFont* const kFontSmall = &fonts::FreeSans9pt7b;
const lgfx::IFont* const kFontBody = &fonts::FreeSansBold12pt7b;
const lgfx::IFont* const kFontLarge = &fonts::FreeSansBold18pt7b;
const lgfx::IFont* const kFontDigits = &fonts::Font7;

constexpr int32_t kScreenWidth = 320;
constexpr int32_t kTitleBottom = 28;
constexpr int32_t kFooterTop = 214;
constexpr int32_t kMargin = 8;
constexpr uint16_t kAxisColors[] = {TFT_RED, TFT_GREEN, 0x54BF};

struct TouchBox {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  bool contains(int32_t px, int32_t py) const {
    return px >= x && px < x + w && py >= y && py < y + h;
  }
};

constexpr TouchBox kSourceBox = {176, 32, 136, 34};
constexpr TouchBox kDimmerBox = {176, 170, 64, 36};
constexpr TouchBox kBrighterBox = {248, 170, 64, 36};

Core2Page core2Page = Core2Page::Drive;
bool swallowRelease = false;

void redrawSoon() {
  haveDashboardFrameHash = false;
  lastDisplayMs = 0;
}

void restoreCore2Page(uint8_t saved) {
  if (saved < static_cast<uint8_t>(Core2Page::Count)) {
    core2Page = static_cast<Core2Page>(saved);
  }
}

void selectCore2Page(Core2Page page) {
  core2Page = page;
  preferences.putUChar("page", static_cast<uint8_t>(page));
  redrawSoon();
}

void moveCore2Page(int direction) {
  const int count = static_cast<int>(Core2Page::Count);
  const int current = static_cast<int>(core2Page);
  selectCore2Page(
      static_cast<Core2Page>((current + direction + count) % count));
}

TouchBox waypointBox(size_t index) {
  const int16_t column = static_cast<int16_t>(index % 3);
  const int16_t row = static_cast<int16_t>(index / 3);
  return {static_cast<int16_t>(6 + column * 105),
          static_cast<int16_t>(66 + row * 48), 100, 44};
}

const char* core2ActionLabel() {
  switch (core2Page) {
    case Core2Page::Drive:
    case Core2Page::Hud:
      return "MARK";
    case Core2Page::Gps:
      return "SOURCE";
    case Core2Page::Waypoint:
      return waypointShowLast ? "GRID" : "LAST";
    case Core2Page::Logger:
      return "KML SCAN";
    case Core2Page::About:
      return dinoGame.state() == dino::Game::State::Auto ? "" : "JUMP";
    default:
      return "";
  }
}

void setScreen(bool on) {
  screenOn = on;
  if (on) {
    M5.Display.wakeup();
    M5.Display.setBrightness(displayBrightness);
    redrawSoon();
  } else {
    M5.Display.sleep();
  }
}

void changeBrightness(int delta) {
  displayBrightness = static_cast<uint8_t>(
      std::clamp(static_cast<int>(displayBrightness) + delta, 10, 255));
  M5.Display.setBrightness(displayBrightness);
  redrawSoon();
}

void markGenericWaypoint() {
  WaypointRecord created;
  createCurrentWaypoint("GENERIC", created);
}

void runPageAction() {
  switch (core2Page) {
    case Core2Page::Drive:
    case Core2Page::Hud:
      markGenericWaypoint();
      break;
    case Core2Page::Gps:
      cycleTelemetrySource();
      break;
    case Core2Page::About:
      dinoGame.press(millis());
      break;
    case Core2Page::Waypoint:
      waypointShowLast = !waypointShowLast;
      redrawSoon();
      break;
    case Core2Page::Logger:
      if (sdMounted && clockIsReady()) {
        kmlExporter.requestScan();
        setWaypointMessage("KML SCAN QUEUED");
      } else {
        setWaypointMessage(sdMounted ? "WAIT FOR CLOCK" : "SD UNAVAILABLE");
      }
      break;
    default:
      break;
  }
}

void handleContentTouch(int32_t x, int32_t y) {
  if (core2Page == Core2Page::Waypoint && !waypointShowLast) {
    for (size_t index = 0; index < kWaypointCategoryCount; ++index) {
      if (waypointBox(index).contains(x, y)) {
        WaypointRecord created;
        createCurrentWaypoint(kWaypointCategories[index], created);
        return;
      }
    }
  } else if (core2Page == Core2Page::Setup) {
    if (kSourceBox.contains(x, y)) {
      cycleTelemetrySource();
    } else if (kDimmerBox.contains(x, y)) {
      changeBrightness(-30);
    } else if (kBrighterBox.contains(x, y)) {
      changeBrightness(30);
    }
  }
}

bool anyInputHeld() {
  return M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed() ||
         M5.Touch.getCount() > 0;
}

void handleControls() {
  if (!screenOn) {
    if (anyInputHeld()) {
      setScreen(true);
      swallowRelease = true;
    }
    return;
  }
  // The press that woke the screen must not also act when it is released.
  if (swallowRelease) {
    if (!anyInputHeld()) swallowRelease = false;
    return;
  }
  if (M5.BtnB.wasHold()) {
    setScreen(false);
    return;
  }
  if (M5.BtnA.wasClicked()) {
    moveCore2Page(-1);
  } else if (M5.BtnC.wasClicked()) {
    moveCore2Page(1);
  } else if (M5.BtnB.wasClicked()) {
    runPageAction();
  }
  const auto touch = M5.Touch.getDetail();
  const bool inContent = touch.y > kTitleBottom && touch.y < kFooterTop;
  if (core2Page == Core2Page::About) {
    // Jump on touch-down rather than release, so it feels immediate.
    if (touch.wasPressed() && inContent) dinoGame.press(millis());
  } else if (touch.wasClicked() && inContent) {
    handleContentTouch(touch.x, touch.y);
  }
}

// Drawing helpers. Proportional fonts are drawn without a background colour
// because the canvas is cleared every frame.

String fitWidth(const String& value, const lgfx::IFont* font,
                int32_t maxWidth) {
  auto& display = dashboardCanvas;
  display.setFont(font);
  display.setTextSize(1);
  if (display.textWidth(value) <= maxWidth) return value;
  String clipped = value;
  while (clipped.length() > 1 &&
         display.textWidth(clipped + "..") > maxWidth) {
    clipped.remove(clipped.length() - 1);
  }
  return clipped + "..";
}

int32_t drawText(int32_t x, int32_t y, lgfx::textdatum_t datum,
                 const lgfx::IFont* font, uint16_t color, const char* format,
                 ...) {
  char buffer[96];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  auto& display = dashboardCanvas;
  display.setFont(font);
  display.setTextSize(1);
  display.setTextDatum(datum);
  display.setTextColor(color);
  return display.drawString(buffer, x, y);
}

void drawButton(const TouchBox& box, const char* label, uint16_t color) {
  auto& display = dashboardCanvas;
  display.fillRoundRect(box.x, box.y, box.w, box.h, 6, 0x2104);
  display.drawRoundRect(box.x, box.y, box.w, box.h, 6, color);
  drawText(box.x + box.w / 2, box.y + box.h / 2, middle_center, kFontSmall,
           color, "%s", label);
}

const char* shortModeName() {
  switch (mode) {
    case LogMode::Moving:
      return "MOVING";
    case LogMode::StoppedFirstHour:
      return "PARKED 15M";
    case LogMode::StoppedHourly:
      return "PARKED 1H";
    case LogMode::FixLost:
      return "FIX LOST";
    default:
      return "WAIT FIX";
  }
}

void drawCore2Header(uint32_t nowMs, time_t nowUtc) {
  auto& display = dashboardCanvas;
  drawText(kMargin, 4, top_left, kFontBody, TFT_CYAN, "%s",
           kCore2PageTitles[static_cast<uint8_t>(core2Page)]);

  // Status items run right to left: battery, clock, Dial, AtomS3.
  int32_t right = kScreenWidth - kMargin;
  if (batteryPercent >= 0) {
    right -= drawText(right, 6, top_right, kFontSmall,
                      batteryColor(batteryPercent), "%d%%%s", batteryPercent,
                      batteryCharging ? "+" : "") +
             10;
  }
  if (clockIsReady()) {
    struct tm local {};
    localtime_r(&nowUtc, &local);
    right -= drawText(right, 6, top_right, kFontSmall, TFT_WHITE, "%02d:%02d",
                      local.tm_hour, local.tm_min) +
             10;
  }
  if (remoteReceiver.dialConnected(nowMs)) {
    right -= drawText(right, 6, top_right, kFontSmall, TFT_CYAN, "DIAL") + 10;
  }
  if (usesRemoteImu(telemetrySource)) {
    drawText(right, 6, top_right, kFontSmall,
             activeTelemetry.metadata.remoteConnected ? TFT_GREEN : TFT_RED,
             "ATOM");
  }
  display.drawFastHLine(0, kTitleBottom, kScreenWidth, TFT_CYAN);
}

void drawCore2Footer() {
  auto& display = dashboardCanvas;
  display.drawFastHLine(0, kFooterTop, kScreenWidth, TFT_DARKGREY);
  constexpr int32_t kLabelY = 228;
  drawText(53, kLabelY, middle_center, kFontSmall, TFT_LIGHTGREY, "< PREV");
  drawText(267, kLabelY, middle_center, kFontSmall, TFT_LIGHTGREY, "NEXT >");
  const char* action = core2ActionLabel();
  if (action[0] != '\0') {
    drawText(160, kLabelY, middle_center, kFontSmall, TFT_YELLOW, "%s",
             action);
  } else {
    drawText(160, kLabelY, middle_center, kFontSmall, TFT_DARKGREY, "%u/%u",
             static_cast<unsigned>(core2Page) + 1,
             static_cast<unsigned>(Core2Page::Count));
  }
}

// Waypoint and action results appear over any page for five seconds.
void drawCore2Banner(uint32_t nowMs) {
  if (waypointMessage.isEmpty() || nowMs - waypointMessageStartedMs >= 5000) {
    return;
  }
  auto& display = dashboardCanvas;
  display.fillRoundRect(kMargin, 174, kScreenWidth - 2 * kMargin, 34, 6,
                        0x0320);
  const String message =
      fitWidth(waypointMessage, kFontBody, kScreenWidth - 4 * kMargin);
  drawText(kScreenWidth / 2, 191, middle_center, kFontBody, TFT_WHITE, "%s",
           message.c_str());
}

void drawCore2DrivePage(const GpsSnapshot& gps, uint32_t nowMs) {
  auto& display = dashboardCanvas;
  const uint16_t speedColor = gps.fixValid ? TFT_GREEN : TFT_ORANGE;
  display.setFont(kFontDigits);
  display.setTextSize(2);
  display.setTextDatum(middle_center);
  display.setTextColor(speedColor);
  if (gps.speedFresh) {
    char speed[8]{};
    snprintf(speed, sizeof(speed), "%d",
             std::clamp(static_cast<int>(std::lround(gps.speedKmh)), 0, 999));
    display.drawString(speed, 150, 84);
  } else {
    display.drawString("--", 150, 84);
  }
  drawText(kScreenWidth - kMargin, 130, bottom_right, kFontBody, TFT_WHITE,
           "km/h");

  drawText(kMargin, 140, top_left, kFontBody,
           gps.fixValid ? TFT_GREEN : TFT_RED, "%s %lu SAT",
           gps.fixValid ? "FIX" : "NO FIX",
           static_cast<unsigned long>(gps.satellites));
  drawText(kScreenWidth - kMargin, 140, top_right, kFontBody,
           logModeColor(mode), "%s", shortModeName());

  String poi;
  String poiSource;
  String waypointId;
  currentPoi(gps, poi, poiSource, waypointId);
  if (!poi.isEmpty()) {
    drawText(kMargin, 166, top_left, kFontBody, TFT_CYAN, "%s",
             fitWidth(poi, kFontBody, kScreenWidth - 2 * kMargin).c_str());
  } else if (gps.positionFresh) {
    drawText(kMargin, 168, top_left, kFontSmall, TFT_LIGHTGREY,
             "%.5f, %.5f", gps.latitude, gps.longitude);
  }

  const uint32_t tripS = tripStartedMs == 0 ? 0 : (nowMs - tripStartedMs) / 1000;
  drawText(kMargin, 192, top_left, kFontSmall, TFT_LIGHTGREY,
           "TRIP %02lu:%02lu", static_cast<unsigned long>(tripS / 3600),
           static_cast<unsigned long>((tripS / 60) % 60));
  drawText(kScreenWidth - kMargin, 192, top_right, kFontSmall, TFT_LIGHTGREY,
           "%s", telemetrySourceLabel(telemetrySource));
}

void drawCore2HudPage(const GpsSnapshot& gps) {
  auto& display = dashboardCanvas;
  char speed[8]{};
  if (gps.speedFresh) {
    snprintf(speed, sizeof(speed), "%d",
             std::clamp(static_cast<int>(std::lround(gps.speedKmh)), 0, 999));
  } else {
    strcpy(speed, "--");
  }
  constexpr int kDigitWidth = 80;
  constexpr int kDigitHeight = 150;
  constexpr int kDigitSpacing = 12;
  constexpr int kDigitTop = 36;
  const int length = strlen(speed);
  const int totalWidth = length * kDigitWidth + (length - 1) * kDigitSpacing;
  const int normalStart = (kScreenWidth - totalWidth) / 2;
  const uint16_t color = gps.fixValid ? TFT_GREEN : TFT_ORANGE;
  // Mirrored so the windscreen reflection reads normally.
  for (int index = 0; index < length; ++index) {
    const int normalX = normalStart + index * (kDigitWidth + kDigitSpacing);
    const int mirroredX = kScreenWidth - normalX - kDigitWidth;
    drawMirroredSevenSegmentDigit(display, speed[index], mirroredX, kDigitTop,
                                  kDigitWidth, kDigitHeight, 12, color);
  }
  drawText(kScreenWidth / 2, 198, middle_center, kFontSmall, TFT_DARKGREY,
           "MIRRORED FOR WINDSCREEN");
}

void drawCore2GpsPage(const GpsSnapshot& gps, uint32_t nowMs) {
  drawText(kMargin, 34, top_left, kFontBody,
           gps.fixValid ? TFT_GREEN : TFT_RED, "%s  %lu SAT",
           gps.fixValid ? "FIX" : "NO FIX",
           static_cast<unsigned long>(gps.satellites));
  if (gps.positionFresh) {
    drawText(kMargin, 60, top_left, kFontBody, TFT_WHITE, "LAT %.6f",
             gps.latitude);
    drawText(kMargin, 86, top_left, kFontBody, TFT_WHITE, "LON %.6f",
             gps.longitude);
  } else {
    drawText(kMargin, 60, top_left, kFontBody, TFT_DARKGREY, "LAT --");
    drawText(kMargin, 86, top_left, kFontBody, TFT_DARKGREY, "LON --");
  }
  char altitude[16] = "--";
  if (gps.altitudeFresh) snprintf(altitude, sizeof(altitude), "%.0fm", gps.altitudeM);
  char hdop[12] = "--";
  if (gps.hdopValid) snprintf(hdop, sizeof(hdop), "%.1f", gps.hdop);
  char heading[12] = "--";
  if (gps.courseFresh) snprintf(heading, sizeof(heading), "%.0f", gps.courseDeg);
  drawText(kMargin, 112, top_left, kFontBody, TFT_WHITE, "ALT %s", altitude);
  drawText(170, 112, top_left, kFontBody, TFT_WHITE, "HDG %s", heading);
  drawText(kMargin, 138, top_left, kFontBody, TFT_WHITE, "HDOP %s", hdop);

  if (telemetrySource == TelemetrySource::AtomS3Remote) {
    const bool live = activeTelemetry.metadata.remoteConnected;
    drawText(kMargin, 164, top_left, kFontBody, live ? TFT_GREEN : TFT_RED,
             "ATOM %s", live ? "LIVE" : "LOST");
  } else {
    const bool live = activeGps->live(nowMs);
    drawText(kMargin, 164, top_left, kFontBody, live ? TFT_GREEN : TFT_ORANGE,
             "GROVE %lu %s", static_cast<unsigned long>(activeGps->baud()),
             live ? "LIVE" : "SCAN");
    drawText(kScreenWidth - kMargin, 168, top_right, kFontSmall,
             TFT_DARKGREY, "RX G%d", activeGps->rxPin());
  }
  drawText(kMargin, 192, top_left, kFontSmall, TFT_LIGHTGREY, "TZ %s",
           fitWidth(locationTime.zoneName(), kFontSmall, 270).c_str());
}

void drawCore2ImuPage() {
  const ImuSample& value = activeTelemetry.imu;
  const bool remote = usesRemoteImu(telemetrySource);
  if (!value.available) {
    drawText(kMargin, 40, top_left, kFontBody, TFT_RED, "IMU UNAVAILABLE");
    drawText(kMargin, 72, top_left, kFontSmall, TFT_WHITE, "%s",
             remote ? "No data from the AtomS3 yet."
                    : "No IMU in this Core2.");
    if (!remote) {
      drawText(kMargin, 96, top_left, kFontSmall, TFT_WHITE,
               "Use SETUP > SOURCE: GPS+ATOM IMU");
    }
    return;
  }
  drawText(kMargin, 34, top_left, kFontSmall, TFT_LIGHTGREY, "AXIS");
  drawText(190, 34, top_right, kFontSmall, TFT_LIGHTGREY, "ACCEL m/s2");
  drawText(kScreenWidth - kMargin, 34, top_right, kFontSmall, TFT_LIGHTGREY,
           "GYRO dps");
  const float accel[] = {value.axMps2, value.ayMps2, value.azMps2};
  const float gyro[] = {value.gxDps, value.gyDps, value.gzDps};
  const char* names[] = {"X", "Y", "Z"};
  for (int axis = 0; axis < 3; ++axis) {
    const int32_t y = 56 + axis * 30;
    drawText(kMargin + 6, y, top_left, kFontBody, kAxisColors[axis], "%s",
             names[axis]);
    drawText(190, y, top_right, kFontBody, kAxisColors[axis], "%+.2f",
             accel[axis]);
    drawText(kScreenWidth - kMargin, y, top_right, kFontBody,
             kAxisColors[axis], "%+.1f", gyro[axis]);
  }
  drawText(kMargin, 150, top_left, kFontBody, TFT_WHITE, "PITCH %+.1f",
           value.pitchDeg);
  drawText(kScreenWidth - kMargin, 150, top_right, kFontBody, TFT_WHITE,
           "ROLL %+.1f", value.rollDeg);
  drawText(kMargin, 184, top_left, kFontSmall, TFT_LIGHTGREY, "VIB %.2f m/s2",
           value.vibrationRmsMps2);
  drawText(kScreenWidth - kMargin, 184, top_right, kFontSmall,
           value.valid ? TFT_GREEN : TFT_ORANGE, "%s %s",
           remote ? "ATOM MPU6886" : imu.typeName(),
           value.valid ? "LIVE" : "WAIT");
}

void drawCore2WaypointPage(const GpsSnapshot& gps) {
  const bool haveLast = waypointStore.hasLast();
  if (waypointShowLast) {
    if (!haveLast) {
      drawText(kMargin, 40, top_left, kFontBody, TFT_DARKGREY,
               "NO WAYPOINTS YET");
      return;
    }
    const auto& last = waypointStore.last();
    const int32_t width = kScreenWidth - 2 * kMargin;
    drawText(kMargin, 34, top_left, kFontBody, TFT_CYAN, "%s",
             fitWidth(last.id + "  " + last.name, kFontBody, width).c_str());
    drawText(kMargin, 62, top_left, kFontSmall, TFT_WHITE, "TYPE %s",
             last.category.c_str());
    if (gps.positionFresh) {
      drawText(kScreenWidth - kMargin, 62, top_right, kFontSmall, TFT_WHITE,
               "%.0f m away",
               waypointStore.distanceFromLastM(gps.latitude, gps.longitude));
    }
    drawText(kMargin, 86, top_left, kFontSmall, TFT_LIGHTGREY, "%s",
             fitWidth(last.timestamp, kFontSmall, width).c_str());
    drawText(kMargin, 110, top_left, kFontSmall, TFT_WHITE, "%s",
             fitWidth("NOTE " + last.note, kFontSmall, width).c_str());
    drawText(kMargin, 134, top_left, kFontSmall, TFT_WHITE, "%s",
             fitWidth("PHOTO " + last.photoReference, kFontSmall, width)
                 .c_str());
    drawText(kMargin, 158, top_left, kFontSmall, TFT_LIGHTGREY,
             "%.6f, %.6f", last.latitude, last.longitude);
    return;
  }

  if (!gps.fixValid) {
    drawText(kMargin, 38, top_left, kFontSmall, TFT_ORANGE,
             "WAITING FOR GPS FIX");
  } else if (haveLast) {
    const auto& last = waypointStore.last();
    drawText(kMargin, 38, top_left, kFontSmall, TFT_LIGHTGREY, "%s",
             fitWidth("LAST " + last.id + " " + last.category + "  " +
                          String(waypointStore.distanceFromLastM(
                                     gps.latitude, gps.longitude),
                                 0) +
                          " m",
                      kFontSmall, kScreenWidth - 2 * kMargin)
                 .c_str());
  } else {
    drawText(kMargin, 38, top_left, kFontSmall, TFT_LIGHTGREY,
             "TAP A TYPE TO SAVE THIS SPOT");
  }
  const uint16_t color = gps.fixValid ? TFT_WHITE : TFT_DARKGREY;
  for (size_t index = 0; index < kWaypointCategoryCount; ++index) {
    drawButton(waypointBox(index), kWaypointLabels[index], color);
  }
}

void drawCore2LoggerPage(time_t nowUtc) {
  drawText(kMargin, 34, top_left, kFontBody,
           sdMounted ? TFT_GREEN : TFT_RED, "SD %s",
           sdMounted ? "READY" : "RETRYING");
  drawText(kScreenWidth - kMargin, 34, top_right, kFontBody,
           logModeColor(mode), "%s", shortModeName());
  drawText(kMargin, 62, top_left, kFontBody, TFT_WHITE, "ROWS %llu",
           static_cast<unsigned long long>(rowsWritten));
  if (!currentLogPath.isEmpty()) {
    const int slash = currentLogPath.lastIndexOf('/');
    drawText(kMargin, 92, top_left, kFontSmall, TFT_LIGHTGREY, "%s",
             currentLogPath.substring(slash + 1).c_str());
  } else {
    drawText(kMargin, 92, top_left, kFontSmall, TFT_ORANGE,
             "Waiting for clock");
  }
  if (nextStoppedDueUtc > nowUtc) {
    drawText(kMargin, 114, top_left, kFontSmall, TFT_LIGHTGREY,
             "Next parked row in %lld min",
             static_cast<long long>((nextStoppedDueUtc - nowUtc + 59) / 60));
  }
  const bool stationary = stationaryForFileWork();
  drawText(kMargin, 140, top_left, kFontSmall,
           kmlExporter.failedCount() == 0 && stationary ? TFT_GREEN
                                                        : TFT_ORANGE,
           "KML %s", stationary ? kmlExporter.phaseName() : "WAITING TO STOP");
  drawText(kMargin, 162, top_left, kFontSmall, TFT_WHITE,
           "Done %u  Errors %u  Queue %u", kmlExporter.completedCount(),
           kmlExporter.failedCount(), kmlExporter.queuedCount());
  drawText(kMargin, 184, top_left, kFontSmall, TFT_DARKGREY, "%s",
           fitWidth(kmlExporter.lastMessage(), kFontSmall,
                    kScreenWidth - 2 * kMargin)
               .c_str());
}

void drawCore2TimePage(time_t nowUtc) {
  if (clockIsReady()) {
    struct tm local {};
    localtime_r(&nowUtc, &local);
    auto& display = dashboardCanvas;
    char clockText[12]{};
    snprintf(clockText, sizeof(clockText), "%02d:%02d:%02d", local.tm_hour,
             local.tm_min, local.tm_sec);
    display.setFont(kFontDigits);
    display.setTextSize(1);
    display.setTextDatum(middle_center);
    display.setTextColor(TFT_GREEN);
    display.drawString(clockText, kScreenWidth / 2, 64);
    drawText(kScreenWidth / 2, 110, middle_center, kFontLarge, TFT_WHITE,
             "%04d-%02d-%02d", local.tm_year + 1900, local.tm_mon + 1,
             local.tm_mday);
    drawText(kScreenWidth / 2, 140, middle_center, kFontSmall, TFT_LIGHTGREY,
             "%s (%s)", locationTime.zoneName(), locationTime.zoneSource());
  } else {
    drawText(kScreenWidth / 2, 70, middle_center, kFontBody, TFT_ORANGE,
             "WAITING FOR TIME");
    drawText(kScreenWidth / 2, 100, middle_center, kFontSmall, TFT_WHITE,
             "From GPS, the RTC or Wi-Fi NTP");
  }
  if (locationTime.wifiConnected()) {
    drawText(kMargin, 162, top_left, kFontSmall, TFT_GREEN, "WI-FI %s",
             fitWidth(WiFi.SSID(), kFontSmall, 220).c_str());
    drawText(kMargin, 184, top_left, kFontSmall, TFT_CYAN, "%s  %s.local",
             WiFi.localIP().toString().c_str(), config::kWebHostname);
  } else {
    const String saved = locationTime.wifiSsid();
    drawText(kMargin, 162, top_left, kFontSmall,
             saved.isEmpty() ? TFT_LIGHTGREY : TFT_RED, "%s",
             saved.isEmpty() ? "WI-FI OFF: set it in logger.cfg"
                             : ("WI-FI OFFLINE  " + saved).c_str());
    drawText(kMargin, 184, top_left, kFontSmall, TFT_DARKGREY, "MAC %s",
             WiFi.macAddress().c_str());
  }
}

void drawCore2SetupPage(uint32_t nowMs) {
  const bool remote = usesRemoteImu(telemetrySource);
  drawText(kMargin, 40, top_left, kFontBody, TFT_WHITE, "SOURCE");
  drawButton(kSourceBox, telemetrySourceLabel(telemetrySource), TFT_YELLOW);
  drawText(kMargin, 76, top_left, kFontSmall, TFT_LIGHTGREY, "ATOMS3");
  drawText(96, 76, top_left, kFontSmall,
           remote && activeTelemetry.metadata.remoteConnected ? TFT_GREEN
                                                              : TFT_WHITE,
           "%s", remote ? remoteReceiver.statusText(nowMs) : "not in use");
  const bool dial = remoteReceiver.dialConnected(nowMs);
  drawText(kMargin, 100, top_left, kFontSmall, TFT_LIGHTGREY, "M5DIAL");
  drawText(96, 100, top_left, kFontSmall, dial ? TFT_GREEN : TFT_WHITE, "%s",
           dial ? "CONNECTED" : "not seen");
  drawText(kMargin, 124, top_left, kFontSmall, TFT_LIGHTGREY, "GPS");
  drawText(96, 124, top_left, kFontSmall,
           groveGps.live(nowMs) ? TFT_GREEN : TFT_ORANGE, "GROVE %lu RX G%d %s",
           static_cast<unsigned long>(groveGps.baud()), groveGps.rxPin(),
           groveGps.live(nowMs) ? "LIVE" : "SCANNING");
  drawText(kMargin, 148, top_left, kFontSmall, TFT_LIGHTGREY, "IMU");
  drawText(96, 148, top_left, kFontSmall, TFT_WHITE, "%s", imu.typeName());
  drawText(kMargin, 180, top_left, kFontBody, TFT_WHITE, "BRIGHT");
  drawButton(kDimmerBox, "-", TFT_WHITE);
  drawButton(kBrighterBox, "+", TFT_WHITE);
}

void drawCore2WeatherPage(uint32_t nowMs) {
  for (int i = 0; i < 3; ++i) {
    const float c = locationCelsius(static_cast<ProbeLocation>(i), nowMs);
    const int32_t y = 36 + i * 30;
    drawText(kMargin, y, top_left, kFontBody, TFT_WHITE, "%s",
             kProbeLocationNames[i]);
    if (std::isnan(c)) {
      drawText(kScreenWidth - kMargin, y, top_right, kFontBody, TFT_DARKGREY,
               "--");
    } else {
      drawText(kScreenWidth - kMargin, y, top_right, kFontBody, TFT_CYAN,
               "%.1f C", c);
    }
  }
  WeatherReading wx;
  if (weather.latest(wx)) {
    drawText(kMargin, 130, top_left, kFontSmall, TFT_YELLOW,
             "Net %.0f C  %s  %.0f%%  %.0f hPa", wx.temperatureC,
             WeatherService::describe(wx.code), wx.humidityPct,
             wx.pressureHpa);
    drawText(kMargin, 150, top_left, kFontSmall, TFT_YELLOW,
             "Wind %.0f km/h from %.0f  (%lu min ago)", wx.windKmh,
             wx.windDirectionDeg,
             static_cast<unsigned long>((nowMs - wx.fetchedMs) / 60000));
  } else {
    drawText(kMargin, 130, top_left, kFontSmall, TFT_DARKGREY, "%s",
             locationTime.wifiConnected() ? "Internet weather: fetching"
                                          : "Internet weather: needs Wi-Fi");
  }
  ProbeEntry entries[8];
  const int count = collectProbes(entries, 8, nowMs);
  if (count == 0) {
    drawText(kMargin, 176, top_left, kFontSmall, TFT_DARKGREY,
             "No probes found");
  }
  for (int i = 0; i < count && i < 2; ++i) {
    drawText(kMargin, 176 + i * 18, top_left, kFontSmall, TFT_LIGHTGREY,
             "%s %04X  %s", probeSourceName(entries[i].source), entries[i].id,
             std::isnan(entries[i].celsius)
                 ? "--"
                 : String(entries[i].celsius, 1).c_str());
  }
}

void drawCore2VersionPage(uint32_t nowMs) {
  auto& display = dashboardCanvas;
  dino::draw(display, kScreenWidth - dino::kWidth * 3 - kMargin, 40, 3,
             (nowMs / 4000) % dino::kFrameCount);
  drawText(kMargin, 40, top_left, kFontSmall, TFT_LIGHTGREY, "SPECIES");
  drawText(kMargin, 58, top_left, kFontBody, TFT_CYAN, "%s", version::kNumber);
  drawText(kMargin, 90, top_left, kFontSmall, TFT_LIGHTGREY, "DNA");
  drawText(kMargin, 108, top_left, kFontBody, TFT_YELLOW, "%s", version::kGit);
  drawText(kMargin, 140, top_left, kFontSmall, TFT_LIGHTGREY, "HATCHED");
  drawText(kMargin, 158, top_left, kFontBody, TFT_GREEN, "%s", version::kDate);
  drawText(kScreenWidth / 2, 198, middle_center, kFontBody, TFT_ORANGE, "%s",
           version::jokeAt(nowMs));
}

void drawCore2AboutPage(uint32_t nowMs) {
  auto& display = dashboardCanvas;
  constexpr int kScale = 4;
  constexpr int kGroundY = 40 + dino::kHeight * kScale + 1;
  dinoGame.update(nowMs, dinoGameSpeedKmh(), kScreenWidth / kScale);
  saveDinoBest();
  dinoGame.draw(display, 0, kGroundY, kScreenWidth, kScale, nowMs);
  switch (dinoGame.state()) {
    case dino::Game::State::Ready:
      drawText(kScreenWidth - kMargin, 34, top_right, kFontSmall, TFT_YELLOW,
               "TAP TO PLAY");
      break;
    case dino::Game::State::Playing:
    case dino::Game::State::Over: {
      char text[32];
      dinoGame.statusText(text, sizeof(text));
      drawText(kScreenWidth - kMargin, 34, top_right, kFontSmall,
               dinoGame.state() == dino::Game::State::Playing ? TFT_WHITE
               : dinoGame.beatBest()                         ? TFT_GREEN
                                                             : TFT_RED,
               "%s", text);
      break;
    }
    default:
      break;
  }
  drawText(kScreenWidth / 2, 132, middle_center, kFontSmall, TFT_WHITE, "%s",
           dino::kCredit1);
  display.setFont(kFontBody);
  display.setTextSize(2);
  display.setTextDatum(middle_center);
  display.setTextColor(TFT_CYAN);
  display.drawString(dino::kCredit2, kScreenWidth / 2, 172);
  display.setTextSize(1);
}

void drawStatus(const GpsSnapshot& sample, uint32_t nowMs, time_t nowUtc) {
  if (!screenOn) return;
  // The dinosaur page animates; everything else refreshes twice a second.
  const uint32_t interval =
      core2Page == Core2Page::About ? 60 : config::kDisplayIntervalMs;
  if (nowMs - lastDisplayMs < interval) return;
  lastDisplayMs = nowMs;

  if (!dashboardCanvasReady) {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.setCursor(8, 100);
    M5.Display.println("Display buffer unavailable");
    return;
  }

  auto& display = dashboardCanvas;
  display.startWrite();
  display.fillScreen(TFT_BLACK);
  display.setTextWrap(false);
  drawCore2Header(nowMs, nowUtc);
  switch (core2Page) {
    case Core2Page::Drive:
      drawCore2DrivePage(sample, nowMs);
      break;
    case Core2Page::Hud:
      drawCore2HudPage(sample);
      break;
    case Core2Page::Gps:
      drawCore2GpsPage(sample, nowMs);
      break;
    case Core2Page::Imu:
      drawCore2ImuPage();
      break;
    case Core2Page::Waypoint:
      drawCore2WaypointPage(sample);
      break;
    case Core2Page::Logger:
      drawCore2LoggerPage(nowUtc);
      break;
    case Core2Page::Time:
      drawCore2TimePage(nowUtc);
      break;
    case Core2Page::Setup:
      drawCore2SetupPage(nowMs);
      break;
    case Core2Page::Weather:
      drawCore2WeatherPage(nowMs);
      break;
    case Core2Page::Version:
      drawCore2VersionPage(nowMs);
      break;
    case Core2Page::About:
      drawCore2AboutPage(nowMs);
      break;
    default:
      break;
  }
  drawCore2Banner(nowMs);
  drawCore2Footer();
  display.setFont(&fonts::Font0);
  display.setTextSize(1);
  display.setTextDatum(top_left);
  display.endWrite();
  const uint32_t frameHash = frameBufferHash(display);
  if (!haveDashboardFrameHash || frameHash != lastDashboardFrameHash) {
    display.pushSprite(0, 0);
    lastDashboardFrameHash = frameHash;
    haveDashboardFrameHash = true;
  }
}
