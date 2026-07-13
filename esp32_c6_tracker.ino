#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <FS.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "soc/usb_serial_jtag_struct.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Waveshare ESP32-C6-Touch-LCD-1.47, landscape 320x172.
static constexpr int LCD_SCK = 1;
static constexpr int LCD_MOSI = 2;
static constexpr int LCD_CS = 14;
static constexpr int LCD_DC = 15;
static constexpr int LCD_RST = 22;
static constexpr int LCD_BL = 23;
static constexpr int LCD_W = 320;
static constexpr int LCD_H = 172;

static constexpr int SD_SCK = 1;
static constexpr int SD_MISO = 3;
static constexpr int SD_MOSI = 2;
static constexpr int SD_CS = 4;

static constexpr int BAT_PIN = 0;
static constexpr float BAT_SCALE = 3.00f;
static constexpr float BAT_EMPTY_V = 3.30f;
static constexpr float BAT_FULL_V = 4.20f;

static constexpr int GPS_RX = 17;  // GPS TX -> ESP32-C6 RX.
static constexpr int GPS_TX = 16;  // GPS RX -> ESP32-C6 TX.
static constexpr uint32_t GPS_BAUD = 38400;
static constexpr int WAKE_BUTTON_PIN = 5;  // Button between GPIO5 and GND, external 10k pull-up.

static constexpr uint32_t TRACK_SAVE_MS = 1000;
static constexpr uint32_t BATTERY_SAMPLE_MS = 1000;
static constexpr uint32_t DISPLAY_UPDATE_MS = 250;
static constexpr uint32_t BLE_UPDATE_MS = 1000;
static constexpr uint32_t SERVICE_LOG_MS = 30000;
static constexpr uint32_t HISTORY_PACKET_MS = 30;
static constexpr uint32_t BLE_CONNECT_WINDOW_MS = 30000;
static constexpr uint32_t BLE_CYCLE_ACK_WINDOW_MS = 10000;
static constexpr uint32_t USB_ACTIVITY_HOLD_MS = 3000;
static constexpr uint64_t SD_LOW_SPACE_BYTES = 16ULL * 1024ULL * 1024ULL;
static constexpr uint8_t HISTORY_CAPACITY = 100;

static constexpr char BLE_DEVICE_NAME[] = "C6 Tracker";
static constexpr char TRACKER_SERVICE_UUID[] = "9f6d1000-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char TELEMETRY_CHAR_UUID[] = "9f6d1001-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char NAV_CHAR_UUID[] = "9f6d1002-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char SETTINGS_CHAR_UUID[] = "9f6d1003-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char COMMAND_CHAR_UUID[] = "9f6d1004-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char RESPONSE_CHAR_UUID[] = "9f6d1005-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char HISTORY_CHAR_UUID[] = "9f6d1006-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char TRACK_TRANSFER_CHAR_UUID[] = "9f6d1007-2b9e-4a5f-8c7d-6f3e4b2a1000";
static constexpr char CYCLE_STATUS_CHAR_UUID[] = "9f6d1008-2b9e-4a5f-8c7d-6f3e4b2a1000";

struct Settings {
  uint16_t gpsTimeoutSec = 120;
  uint8_t minSats = 4;
  float maxHdop = 5.0f;
  uint32_t sleepTimeSec = 60;
  bool screenEnabled = true;
  bool bleEnabled = true;
  uint8_t minPointsPerWake = 1;
  bool filterEnabled = false;
  bool sleepWhileBleConnected = false;
};

struct BatteryState {
  uint16_t raw = 0;
  float pinVoltage = 0.0f;
  float voltage = 0.0f;
  uint8_t percent = 0;
};

struct BleCommand {
  char text[96];
};

struct HistoryPoint {
  int32_t latE7 = 0;
  int32_t lonE7 = 0;
  int16_t altitudeM = 0;
};

struct BatteryHistorySample {
  uint32_t sequence = 0;
  uint16_t millivolts = 0;
  uint32_t epoch = 0;
  uint16_t cycleId = 0;
};

enum HistoryStreamPhase : uint8_t {
  HISTORY_IDLE,
  HISTORY_TRACK_START,
  HISTORY_TRACK_POINTS,
  HISTORY_BATTERY_START,
  HISTORY_BATTERY_CHUNKS,
  HISTORY_USER_START,
  HISTORY_USER_POINTS,
  HISTORY_END,
};

enum TrackTransferPhase : uint8_t {
  TRACK_TRANSFER_IDLE,
  TRACK_LIST_START,
  TRACK_LIST_ITEMS,
  TRACK_LIST_NAME,
  TRACK_LIST_META,
  TRACK_LOAD_START,
  TRACK_LOAD_POINTS,
  TRACK_LOAD_POINT_DETAIL,
  TRACK_LOAD_POINT_STATUS,
  TRACK_LOAD_MARK_NAME,
  TRACK_LOAD_MARK_COMMENT,
  TRACK_LOAD_END,
  TRACK_LOG_START,
  TRACK_LOG_CHUNKS,
  TRACK_LOG_END,
};

RTC_DATA_ATTR float totalDistanceKm = 0.0f;
RTC_DATA_ATTR double lastLat = 0.0;
RTC_DATA_ATTR double lastLon = 0.0;
RTC_DATA_ATTR float startAltitudeM = 0.0f;
RTC_DATA_ATTR bool hasLastPoint = false;
RTC_DATA_ATTR uint32_t pointCount = 0;
RTC_DATA_ATTR bool recording = false;
RTC_DATA_ATTR uint32_t routeStartGpsEpoch = 0;
RTC_DATA_ATTR uint32_t lastRouteElapsedSec = 0;
RTC_DATA_ATTR uint32_t routeMarkCount = 0;
RTC_DATA_ATTR uint32_t batterySampleSequence = 0;
RTC_DATA_ATTR uint16_t wakeCycleSequence = 0;

static Arduino_DataBus *lcdBus =
    new Arduino_HWSPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, SD_MISO, &SPI, true);
static Arduino_GFX *gfx = new Arduino_ST7789(lcdBus, LCD_RST, 0, false, 172, 320, 34, 0, 34, 0);
static HardwareSerial gpsSerial(1);
static TinyGPSPlus gps;
static Preferences preferences;
static Settings settings;
static BatteryState battery;

static bool sdOk = false;
static bool sdWriteError = false;
static bool sdLowSpace = false;
static bool sdTailRepaired = false;
static bool firstFixHandled = false;
static bool gpsRawUsb = false;
static uint32_t routeNumber = 1;
static char routePath[32] = "/track_001.geojsonl";
static uint32_t bootMs = 0;
static uint32_t gpsWaitStartMs = 0;
static uint32_t lastTrackSaveMs = 0;
static uint32_t lastBatterySampleMs = 0;
static uint32_t lastDisplayUpdateMs = 0;
static uint32_t lastBleUpdateMs = 0;
static uint32_t lastServiceLogMs = 0;
static uint32_t lastSdRetryMs = 0;
static uint32_t recordingSegmentStartMs = 0;
static uint32_t accumulatedRecordingMs = 0;
static uint16_t lastUsbFrame = 0;
static uint32_t lastUsbActivityMs = 0;
static char serialCommand[96] = {};
static uint8_t serialCommandLength = 0;
static HistoryPoint trackHistory[HISTORY_CAPACITY] = {};
static HistoryPoint userPointHistory[HISTORY_CAPACITY] = {};
static BatteryHistorySample batteryHistory[HISTORY_CAPACITY] = {};
static uint8_t trackHistoryHead = 0;
static uint8_t trackHistoryCount = 0;
static uint8_t userPointHistoryHead = 0;
static uint8_t userPointHistoryCount = 0;
static uint8_t batteryHistoryHead = 0;
static uint8_t batteryHistoryCount = 0;
static bool cycleBatteryRecorded = false;
static bool sdHistoryRestored = false;
static HistoryStreamPhase historyStreamPhase = HISTORY_IDLE;
static uint8_t historyStreamIndex = 0;
static uint32_t lastHistoryPacketMs = 0;
static TrackTransferPhase trackTransferPhase = TRACK_TRANSFER_IDLE;
static File trackTransferDirectory;
static File trackTransferFile;
static uint16_t trackTransferRoute = 0;
static uint32_t trackTransferSize = 0;
static uint32_t trackTransferPointCount = 0;
static uint32_t trackTransferMarkCount = 0;
static uint32_t trackTransferSkipPoints = 0;
static uint32_t trackTransferSkipMarks = 0;
static bool trackTransferIncremental = false;
static uint32_t lastTrackTransferPacketMs = 0;
static String pendingMarkName;
static String pendingMarkComment;
static uint16_t pendingMarkIndex = 0;
static uint16_t pendingPointIndex = 0;
static uint16_t pendingPointHdop = 0;
static uint16_t pendingPointRecordId = 0;
static uint16_t pendingPointCycleId = 0;
static uint8_t pendingPointType = 0;
static uint8_t pendingPointSats = 0;
static String pendingPointStatus;
static uint16_t pendingListRoute = 0;
static String pendingListName;
static uint32_t pendingMetaPoints = 0, pendingMetaMarks = 0, pendingMetaCreated = 0, pendingMetaUpdated = 0;

static lv_disp_draw_buf_t lvDrawBuffer;
static lv_disp_drv_t lvDisplayDriver;
static lv_color_t *lvBuffer = nullptr;
static lv_obj_t *distanceValueLabel = nullptr;
static lv_obj_t *altitudeValueLabel = nullptr;
static lv_obj_t *batteryValueLabel = nullptr;
static lv_obj_t *batteryFill = nullptr;
static bool displayReady = false;

static BLECharacteristic *telemetryCharacteristic = nullptr;
static BLECharacteristic *navCharacteristic = nullptr;
static BLECharacteristic *settingsCharacteristic = nullptr;
static BLECharacteristic *responseCharacteristic = nullptr;
static BLECharacteristic *historyCharacteristic = nullptr;
static BLECharacteristic *trackTransferCharacteristic = nullptr;
static BLECharacteristic *cycleStatusCharacteristic = nullptr;
static BLECharacteristic *batteryCharacteristic = nullptr;
static QueueHandle_t bleCommandQueue = nullptr;
static volatile bool bleConnected = false;
static bool bleStarted = false;
static bool bleWasConnected = false;
static bool cycleComplete = false;
static bool configurationWake = false;
static bool runtimeScreenEnabled = true;
static bool runtimeBleEnabled = true;
static uint8_t cyclePointCount = 0;
static uint32_t bleConnectWindowStartMs = 0;
static float filteredAltitudeM = NAN;
static double filteredLastLat = 0.0;
static double filteredLastLon = 0.0;
static uint32_t filteredLastEpoch = 0;
static bool filteredHasPoint = false;
static bool sleepAnnounced = false;
static uint32_t sleepAnnouncedMs = 0;
static uint32_t cycleCompleteMs = 0;
static bool cycleAcknowledged = false;
static esp_sleep_wakeup_cause_t bootWakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;

static void publishSettings();
static void publishTelemetry(bool notifyClient);
static void serviceLog(const char *event, const char *status, const char *message);
static bool gpsFixValid();
static void sendBleResponse(const char *message);
static void serviceHistoryStream(uint32_t now);
static void serviceTrackTransfer(uint32_t now);
static void cancelTrackTransfer();
static void saveCurrentTrackMeta();

static uint16_t clampU16(uint32_t value) {
  return value > 65535U ? 65535U : static_cast<uint16_t>(value);
}

static int16_t clampI16(int32_t value) {
  if (value < -32768) return -32768;
  if (value > 32767) return 32767;
  return static_cast<int16_t>(value);
}

static void markCycleComplete() {
  if (cycleComplete) return;
  cycleComplete = true;
  cycleCompleteMs = millis();
  char message[48] = {};
  snprintf(message, sizeof(message), "points=%u;gps_wait=%lu", cyclePointCount,
           static_cast<unsigned long>((millis() - gpsWaitStartMs) / 1000));
  serviceLog("CYCLE", gpsFixValid() ? "GPS_OK" : "TIMEOUT", message);
}

static bool usbHostConnected() {
  const uint16_t frame = USB_SERIAL_JTAG.fram_num.sof_frame_index;
  if (USB_SERIAL_JTAG.bus_reset_st.usb_bus_reset_st && frame != lastUsbFrame) {
    lastUsbFrame = frame;
    lastUsbActivityMs = millis();
  }
  return lastUsbActivityMs != 0 && millis() - lastUsbActivityMs < USB_ACTIVITY_HOLD_MS;
}

static void putU16(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value);
  buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
}

static void putI16(uint8_t *buffer, size_t offset, int16_t value) {
  putU16(buffer, offset, static_cast<uint16_t>(value));
}

static void putU32(uint8_t *buffer, size_t offset, uint32_t value) {
  buffer[offset] = static_cast<uint8_t>(value);
  buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
  buffer[offset + 2] = static_cast<uint8_t>(value >> 16);
  buffer[offset + 3] = static_cast<uint8_t>(value >> 24);
}

static void putI32(uint8_t *buffer, size_t offset, int32_t value) {
  putU32(buffer, offset, static_cast<uint32_t>(value));
}

static void lcdRegisterInit() {
  static const uint8_t operations[] = {
      BEGIN_WRITE,
      WRITE_COMMAND_8, 0x11,
      END_WRITE,
      DELAY, 120,
      BEGIN_WRITE,
      WRITE_C8_D16, 0xDF, 0x98, 0x53,
      WRITE_C8_D8, 0xB2, 0x23,
      WRITE_COMMAND_8, 0xB7,
      WRITE_BYTES, 4, 0x00, 0x47, 0x00, 0x6F,
      WRITE_COMMAND_8, 0xBB,
      WRITE_BYTES, 6, 0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,
      WRITE_C8_D16, 0xC0, 0x44, 0xA4,
      WRITE_C8_D8, 0xC1, 0x16,
      WRITE_COMMAND_8, 0xC3,
      WRITE_BYTES, 8, 0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,
      WRITE_COMMAND_8, 0xC4,
      WRITE_BYTES, 12, 0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,
      WRITE_COMMAND_8, 0xC8,
      WRITE_BYTES, 32,
      0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28,
      0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
      0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28,
      0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
      WRITE_COMMAND_8, 0xD0,
      WRITE_BYTES, 5, 0x04, 0x06, 0x6B, 0x0F, 0x00,
      WRITE_C8_D16, 0xD7, 0x00, 0x30,
      WRITE_C8_D8, 0xE6, 0x14,
      WRITE_C8_D8, 0xDE, 0x01,
      WRITE_COMMAND_8, 0xB7,
      WRITE_BYTES, 5, 0x03, 0x13, 0xEF, 0x35, 0x35,
      WRITE_COMMAND_8, 0xC1,
      WRITE_BYTES, 3, 0x14, 0x15, 0xC0,
      WRITE_C8_D16, 0xC2, 0x06, 0x3A,
      WRITE_C8_D16, 0xC4, 0x72, 0x12,
      WRITE_C8_D8, 0xBE, 0x00,
      WRITE_C8_D8, 0xDE, 0x02,
      WRITE_COMMAND_8, 0xE5,
      WRITE_BYTES, 3, 0x00, 0x02, 0x00,
      WRITE_COMMAND_8, 0xE5,
      WRITE_BYTES, 3, 0x01, 0x02, 0x00,
      WRITE_C8_D8, 0xDE, 0x00,
      WRITE_C8_D8, 0x35, 0x00,
      WRITE_C8_D8, 0x3A, 0x05,
      WRITE_COMMAND_8, 0x2A,
      WRITE_BYTES, 4, 0x00, 0x22, 0x00, 0xCD,
      WRITE_COMMAND_8, 0x2B,
      WRITE_BYTES, 4, 0x00, 0x00, 0x01, 0x3F,
      WRITE_C8_D8, 0xDE, 0x02,
      WRITE_COMMAND_8, 0xE5,
      WRITE_BYTES, 3, 0x00, 0x02, 0x00,
      WRITE_C8_D8, 0xDE, 0x00,
      WRITE_C8_D8, 0x36, 0x00,
      WRITE_COMMAND_8, 0x21,
      END_WRITE,
      DELAY, 10,
      BEGIN_WRITE,
      WRITE_COMMAND_8, 0x29,
      END_WRITE,
  };
  lcdBus->batchOperation(operations, sizeof(operations));
}

static void lvglFlush(lv_disp_drv_t *driver, const lv_area_t *area, lv_color_t *pixels) {
  const uint32_t width = area->x2 - area->x1 + 1;
  const uint32_t height = area->y2 - area->y1 + 1;
#if LV_COLOR_16_SWAP != 0
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, reinterpret_cast<uint16_t *>(&pixels->full), width, height);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, reinterpret_cast<uint16_t *>(&pixels->full), width, height);
#endif
  lv_disp_flush_ready(driver);
}

static lv_obj_t *makeLabel(lv_obj_t *parent, const char *text, int x, int y,
                           const lv_font_t *font, lv_color_t color) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_pos(label, x, y);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0);
  return label;
}

static lv_obj_t *makeSeparator(lv_obj_t *parent, int y) {
  lv_obj_t *line = lv_obj_create(parent);
  lv_obj_remove_style_all(line);
  lv_obj_set_pos(line, 10, y);
  lv_obj_set_size(line, 300, 1);
  lv_obj_set_style_bg_color(line, lv_color_hex(0x244834), 0);
  lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
  return line;
}

static void createTrackerScreen() {
  lv_obj_t *root = lv_scr_act();
  lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(root, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(root, 0, 0);
  lv_obj_set_style_pad_all(root, 0, 0);

  makeLabel(root, "DISTANCE", 12, 7, &lv_font_montserrat_14, lv_color_hex(0xB8BBC3));
  makeLabel(root, "BAT", 188, 7, &lv_font_montserrat_16, lv_color_hex(0xF1D20A));
  batteryValueLabel = makeLabel(root, "--%", 225, 3, &lv_font_montserrat_20, lv_color_white());

  lv_obj_t *batteryOutline = lv_obj_create(root);
  lv_obj_remove_style_all(batteryOutline);
  lv_obj_set_pos(batteryOutline, 275, 7);
  lv_obj_set_size(batteryOutline, 34, 17);
  lv_obj_set_style_border_width(batteryOutline, 2, 0);
  lv_obj_set_style_border_color(batteryOutline, lv_color_hex(0xF1D20A), 0);
  lv_obj_set_style_border_opa(batteryOutline, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(batteryOutline, 3, 0);
  lv_obj_set_style_bg_color(batteryOutline, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(batteryOutline, LV_OPA_COVER, 0);

  batteryFill = lv_obj_create(batteryOutline);
  lv_obj_remove_style_all(batteryFill);
  lv_obj_set_pos(batteryFill, 3, 3);
  lv_obj_set_size(batteryFill, 2, 7);
  lv_obj_set_style_bg_color(batteryFill, lv_color_hex(0xF1D20A), 0);
  lv_obj_set_style_bg_opa(batteryFill, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(batteryFill, 1, 0);

  lv_obj_t *batteryNub = lv_obj_create(root);
  lv_obj_remove_style_all(batteryNub);
  lv_obj_set_pos(batteryNub, 309, 11);
  lv_obj_set_size(batteryNub, 3, 9);
  lv_obj_set_style_bg_color(batteryNub, lv_color_hex(0xF1D20A), 0);
  lv_obj_set_style_bg_opa(batteryNub, LV_OPA_COVER, 0);

  makeSeparator(root, 31);

  distanceValueLabel = makeLabel(root, "0.00", 28, 35, &lv_font_montserrat_48, lv_color_white());
  lv_obj_set_width(distanceValueLabel, 210);
  lv_obj_set_style_text_align(distanceValueLabel, LV_TEXT_ALIGN_RIGHT, 0);
  makeLabel(root, "KM", 246, 69, &lv_font_montserrat_20, lv_color_hex(0xB8BBC3));

  makeSeparator(root, 111);
  makeLabel(root, "ALT", 18, 129, &lv_font_montserrat_20, lv_color_hex(0xB8BBC3));
  altitudeValueLabel = makeLabel(root, "---", 98, 119, &lv_font_montserrat_32, lv_color_hex(0x43D13E));
  lv_obj_set_width(altitudeValueLabel, 165);
  lv_obj_set_style_text_align(altitudeValueLabel, LV_TEXT_ALIGN_RIGHT, 0);
  makeLabel(root, "M", 276, 133, &lv_font_montserrat_20, lv_color_hex(0x43D13E));
}

static bool initDisplay() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  if (!gfx->begin()) {
    Serial.println("ERR display begin");
    return false;
  }
  lcdRegisterInit();
  gfx->setRotation(1);
  gfx->fillScreen(RGB565_BLACK);

  lv_init();
  constexpr uint32_t bufferPixels = LCD_W * 40;
  lvBuffer = static_cast<lv_color_t *>(
      heap_caps_malloc(bufferPixels * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!lvBuffer) {
    lvBuffer = static_cast<lv_color_t *>(heap_caps_malloc(bufferPixels * sizeof(lv_color_t), MALLOC_CAP_8BIT));
  }
  if (!lvBuffer) {
    Serial.println("ERR LVGL buffer allocation");
    return false;
  }

  lv_disp_draw_buf_init(&lvDrawBuffer, lvBuffer, nullptr, bufferPixels);
  lv_disp_drv_init(&lvDisplayDriver);
  lvDisplayDriver.hor_res = LCD_W;
  lvDisplayDriver.ver_res = LCD_H;
  lvDisplayDriver.flush_cb = lvglFlush;
  lvDisplayDriver.draw_buf = &lvDrawBuffer;
  lv_disp_drv_register(&lvDisplayDriver);

  createTrackerScreen();
  lv_timer_handler();

  ledcAttach(LCD_BL, 5000, 10);
  ledcWrite(LCD_BL, 680);
  displayReady = true;
  return true;
}

static float currentAltitudeM() {
  return gps.altitude.isValid() ? static_cast<float>(gps.altitude.meters()) : 0.0f;
}

static void updateDisplayValues() {
  if (!displayReady) return;

  static char previousDistance[16] = {};
  static char previousAltitude[16] = {};
  static int previousBattery = -1;
  char value[16];

  if (totalDistanceKm < 1000.0f) {
    snprintf(value, sizeof(value), "%.2f", totalDistanceKm);
  } else {
    snprintf(value, sizeof(value), "%.1f", totalDistanceKm);
  }
  if (strcmp(value, previousDistance) != 0) {
    strncpy(previousDistance, value, sizeof(previousDistance) - 1);
    lv_label_set_text(distanceValueLabel, value);
  }

  if (gps.altitude.isValid() && gps.altitude.age() < 5000) {
    snprintf(value, sizeof(value), "%.0f", gps.altitude.meters());
  } else {
    strcpy(value, "---");
  }
  if (strcmp(value, previousAltitude) != 0) {
    strncpy(previousAltitude, value, sizeof(previousAltitude) - 1);
    lv_label_set_text(altitudeValueLabel, value);
  }

  if (previousBattery != battery.percent) {
    previousBattery = battery.percent;
    lv_label_set_text_fmt(batteryValueLabel, "%u%%", battery.percent);
    const int fillWidth = max(2, static_cast<int>(25 * battery.percent / 100));
    lv_obj_set_width(batteryFill, fillWidth);
  }
}

static void updateRoutePath() {
  snprintf(routePath, sizeof(routePath), "/track_%03lu.geojsonl", static_cast<unsigned long>(routeNumber));
}

static void readSettings() {
  preferences.begin("c6track", false);
  settings.gpsTimeoutSec = static_cast<uint16_t>(
      constrain(static_cast<int>(preferences.getUInt("gps_to", settings.gpsTimeoutSec)), 30, 3600));
  settings.minSats = static_cast<uint8_t>(
      constrain(static_cast<int>(preferences.getUChar("min_sat", settings.minSats)), 1, 20));
  settings.maxHdop = constrain(preferences.getFloat("max_hdop", settings.maxHdop), 0.5f, 20.0f);
  settings.sleepTimeSec = constrain(preferences.getUInt("sleep_s", settings.sleepTimeSec), 10U, 86400U);
  settings.screenEnabled = preferences.getBool("screen", settings.screenEnabled);
  settings.bleEnabled = preferences.getBool("ble", settings.bleEnabled);
  settings.minPointsPerWake = static_cast<uint8_t>(
      constrain(static_cast<int>(preferences.getUChar("min_pts", settings.minPointsPerWake)), 1, 100));
  settings.filterEnabled = preferences.getBool("filter", settings.filterEnabled);
  settings.sleepWhileBleConnected = preferences.getBool("sleep_bt", settings.sleepWhileBleConnected);
  routeNumber = max<uint32_t>(1, preferences.getUInt("route", routeNumber));
  if (routeStartGpsEpoch == 0) routeStartGpsEpoch = preferences.getUInt("route_t0", 0);
  updateRoutePath();
}

static void saveSettings() {
  preferences.putUInt("gps_to", settings.gpsTimeoutSec);
  preferences.putUChar("min_sat", settings.minSats);
  preferences.putFloat("max_hdop", settings.maxHdop);
  preferences.putUInt("sleep_s", settings.sleepTimeSec);
  preferences.putBool("screen", settings.screenEnabled);
  preferences.putBool("ble", settings.bleEnabled);
  preferences.putUChar("min_pts", settings.minPointsPerWake);
  preferences.putBool("filter", settings.filterEnabled);
  preferences.putBool("sleep_bt", settings.sleepWhileBleConnected);
  if (sdOk && !sdWriteError && !sdLowSpace) {
    File file = SD.open("/settings.csv", FILE_APPEND);
    if (file) {
      const size_t written = file.printf("%lu,%u,%.2f,%lu,%u,%u,%u,%u,%u\n", static_cast<unsigned long>(settings.gpsTimeoutSec),
                                         settings.minSats, settings.maxHdop, static_cast<unsigned long>(settings.sleepTimeSec),
                                         settings.screenEnabled, settings.bleEnabled, settings.minPointsPerWake,
                                         settings.filterEnabled, settings.sleepWhileBleConnected);
      file.flush();
      if (written == 0 || file.getWriteError()) sdWriteError = true;
      file.close();
    } else sdWriteError = true;
  }
  publishSettings();
}

static void clearTrackHistory() {
  trackHistoryHead = 0;
  trackHistoryCount = 0;
}

static void clearBatteryHistory() {
  batteryHistoryHead = 0;
  batteryHistoryCount = 0;
}

static void clearUserPointHistory() {
  userPointHistoryHead = 0;
  userPointHistoryCount = 0;
}

static void pushTrackHistory(double lat, double lon, float altitude) {
  HistoryPoint &point = trackHistory[trackHistoryHead];
  point.latE7 = static_cast<int32_t>(lround(lat * 10000000.0));
  point.lonE7 = static_cast<int32_t>(lround(lon * 10000000.0));
  point.altitudeM = clampI16(lroundf(altitude));
  trackHistoryHead = (trackHistoryHead + 1) % HISTORY_CAPACITY;
  if (trackHistoryCount < HISTORY_CAPACITY) ++trackHistoryCount;
}

static void pushUserPointHistory(double lat, double lon, float altitude) {
  HistoryPoint &point = userPointHistory[userPointHistoryHead];
  point.latE7 = static_cast<int32_t>(lround(lat * 10000000.0));
  point.lonE7 = static_cast<int32_t>(lround(lon * 10000000.0));
  point.altitudeM = clampI16(lroundf(altitude));
  userPointHistoryHead = (userPointHistoryHead + 1) % HISTORY_CAPACITY;
  if (userPointHistoryCount < HISTORY_CAPACITY) ++userPointHistoryCount;
}

static void pushBatteryHistory(uint32_t sequence, uint16_t millivolts, uint32_t epoch = 0,
                               uint16_t cycleId = 0) {
  batteryHistory[batteryHistoryHead] = {sequence, millivolts, epoch, cycleId};
  batteryHistoryHead = (batteryHistoryHead + 1) % HISTORY_CAPACITY;
  if (batteryHistoryCount < HISTORY_CAPACITY) ++batteryHistoryCount;
  if (sequence > batterySampleSequence) batterySampleSequence = sequence;
}

static const HistoryPoint &trackHistoryAt(uint8_t index) {
  const uint8_t start = (trackHistoryHead + HISTORY_CAPACITY - trackHistoryCount) % HISTORY_CAPACITY;
  return trackHistory[(start + index) % HISTORY_CAPACITY];
}

static const HistoryPoint &userPointHistoryAt(uint8_t index) {
  const uint8_t start =
      (userPointHistoryHead + HISTORY_CAPACITY - userPointHistoryCount) % HISTORY_CAPACITY;
  return userPointHistory[(start + index) % HISTORY_CAPACITY];
}

static const BatteryHistorySample &batteryHistoryAt(uint8_t index) {
  const uint8_t start = (batteryHistoryHead + HISTORY_CAPACITY - batteryHistoryCount) % HISTORY_CAPACITY;
  return batteryHistory[(start + index) % HISTORY_CAPACITY];
}

static void restoreTrackHistoryFromSd() {
  if (trackHistoryCount != 0 || !SD.exists(routePath)) return;
  File file = SD.open(routePath, FILE_READ);
  if (!file) return;
  const size_t fileSize = file.size();
  const size_t start = fileSize > 65536 ? fileSize - 65536 : 0;
  file.seek(start);
  if (start > 0) file.readStringUntil('\n');
  while (file.available()) {
    const String line = file.readStringUntil('\n');
    const bool isTrack = line.indexOf("\"point_type\":\"track\"") >= 0;
    const bool isUserPoint = line.indexOf("\"point_type\":\"user_point\"") >= 0;
    if (!isTrack && !isUserPoint) continue;
    const int marker = line.indexOf("\"coordinates\":[");
    if (marker < 0) continue;
    double lon = 0.0;
    double lat = 0.0;
    float altitude = 0.0f;
    if (sscanf(line.c_str() + marker + 15, "%lf,%lf,%f", &lon, &lat, &altitude) == 3) {
      if (isTrack) pushTrackHistory(lat, lon, altitude);
      if (isUserPoint) pushUserPointHistory(lat, lon, altitude);
    }
  }
  file.close();
}

static void restoreBatteryHistoryFromSd() {
  if (batteryHistoryCount > 1 || !SD.exists("/battery.csv")) return;
  File file = SD.open("/battery.csv", FILE_READ);
  if (!file) return;
  clearBatteryHistory();
  const size_t fileSize = file.size();
  const size_t start = fileSize > 16384 ? fileSize - 16384 : 0;
  file.seek(start);
  if (start > 0) file.readStringUntil('\n');
  uint32_t legacySequence = 0;
  while (file.available()) {
    const String line = file.readStringUntil('\n');
    unsigned long sequenceOrUptime = 0;
    float voltage = 0.0f;
    unsigned long explicitSequence = 0, epoch = 0;
    unsigned int cycleId = 0;
    const int parsed = sscanf(line.c_str(), "%lu,%f,%*f,%*u,%*u,%lu,%lu,%u", &sequenceOrUptime,
                              &voltage, &explicitSequence, &epoch, &cycleId);
    if (parsed >= 2 && voltage > 0.0f) {
      const uint32_t sequence = parsed >= 3 ? static_cast<uint32_t>(explicitSequence)
                                            : ++legacySequence;
      pushBatteryHistory(sequence, clampU16(lroundf(voltage * 1000.0f)),
                         parsed >= 4 ? static_cast<uint32_t>(epoch) : 0,
                         parsed >= 5 ? static_cast<uint16_t>(cycleId) : 0);
    }
  }
  file.close();
}

static void restoreHistoryFromSd() {
  if (sdHistoryRestored || !sdOk) return;
  restoreTrackHistoryFromSd();
  restoreBatteryHistoryFromSd();
  sdHistoryRestored = true;
}

static void ensureLogFiles() {
  if (!sdOk) return;
  if (!SD.exists("/service.csv")) {
    File file = SD.open("/service.csv", FILE_WRITE);
    if (file) {
      file.println("uptime_ms,gps_epoch,cycle_id,event,status,battery_v,battery_pct,sats,hdop,message");
      file.close();
    }
  }
  if (!SD.exists("/battery.csv")) {
    File file = SD.open("/battery.csv", FILE_WRITE);
    if (file) {
      file.println("uptime_ms,voltage,pin_v,raw,percent,sample_id,gps_epoch,cycle_id");
      file.close();
    }
  }
  if (!SD.exists("/settings.csv")) {
    File file = SD.open("/settings.csv", FILE_WRITE);
    if (file) {
      file.println("gps_timeout_s,min_sats,max_hdop,sleep_s,screen,ble,min_points_per_wake,filter,sleep_while_ble");
      file.close();
    }
  }
}

static void refreshSdHealth() {
  if (!sdOk) {
    sdLowSpace = false;
    return;
  }
  const uint64_t total = SD.totalBytes();
  const uint64_t used = SD.usedBytes();
  sdLowSpace = total > 0 && total - min(total, used) < SD_LOW_SPACE_BYTES;
}

static bool validateSdWrite() {
  static constexpr char testPath[] = "/.__c6_write_test";
  SD.remove(testPath);
  File file = SD.open(testPath, FILE_WRITE);
  if (!file) return false;
  const size_t written = file.print("ok\n");
  file.flush();
  const bool ok = written == 3 && file.getWriteError() == 0;
  file.close();
  SD.remove(testPath);
  return ok;
}

static bool repairGeoJsonTail(const char *path) {
  if (!path || !SD.exists(path)) return false;
  File source = SD.open(path, FILE_READ);
  if (!source) return false;
  const size_t size = source.size();
  if (size == 0) { source.close(); return false; }
  source.seek(size - 1);
  if (source.read() == '\n') { source.close(); return false; }

  size_t validSize = 0;
  for (size_t position = size; position > 0; --position) {
    source.seek(position - 1);
    if (source.read() == '\n') { validSize = position; break; }
  }
  source.seek(0);
  static constexpr char tempPath[] = "/.__c6_repair.tmp";
  static constexpr char backupPath[] = "/.__c6_repair.bak";
  SD.remove(tempPath);
  SD.remove(backupPath);
  File repaired = SD.open(tempPath, FILE_WRITE);
  if (!repaired) { source.close(); return false; }
  uint8_t buffer[512];
  size_t remaining = validSize;
  bool copyOk = true;
  while (remaining > 0) {
    const size_t wanted = min(remaining, sizeof(buffer));
    const size_t received = source.read(buffer, wanted);
    if (received == 0 || repaired.write(buffer, received) != received) { copyOk = false; break; }
    remaining -= received;
  }
  repaired.flush();
  copyOk = copyOk && repaired.getWriteError() == 0 && remaining == 0;
  repaired.close();
  source.close();
  if (!copyOk || !SD.rename(path, backupPath)) { SD.remove(tempPath); return false; }
  if (!SD.rename(tempPath, path)) {
    SD.rename(backupPath, path);
    SD.remove(tempPath);
    return false;
  }
  SD.remove(backupPath);
  return true;
}

static bool initSd() {
  sdOk = false;
  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD mount failed");
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println("SD card absent");
    SD.end();
    return false;
  }
  sdOk = true;
  sdWriteError = !validateSdWrite();
  refreshSdHealth();
  ensureLogFiles();
  if (repairGeoJsonTail(routePath)) sdTailRepaired = true;
  restoreHistoryFromSd();
  Serial.printf("SD OK: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}

static uint32_t currentGpsEpoch();

static void serviceLog(const char *event, const char *status, const char *message) {
  if (!sdOk || sdWriteError || sdLowSpace) return;
  File file = SD.open("/service.csv", FILE_APPEND);
  if (!file) { sdWriteError = true; return; }
  const size_t written = file.printf("%lu,%lu,%u,%s,%s,%.3f,%u,%lu,%.2f,%s\n",
                                     static_cast<unsigned long>(millis()),
                                     static_cast<unsigned long>(currentGpsEpoch()), wakeCycleSequence,
                                     event, status, battery.voltage, battery.percent,
                                     static_cast<unsigned long>(gps.satellites.isValid() ? gps.satellites.value() : 0),
                                     gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, message);
  file.flush();
  if (written == 0 || file.getWriteError()) sdWriteError = true;
  file.close();
}

static void sampleBattery(bool recordHistory) {
  uint32_t rawSum = 0;
  uint32_t millivoltsSum = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    rawSum += analogRead(BAT_PIN);
    millivoltsSum += analogReadMilliVolts(BAT_PIN);
  }
  battery.raw = rawSum / 8;
  battery.pinVoltage = (millivoltsSum / 8) / 1000.0f;
  const float measured = battery.pinVoltage * BAT_SCALE;
  battery.voltage = battery.voltage < 0.1f ? measured : battery.voltage * 0.85f + measured * 0.15f;
  const int percent = lroundf((battery.voltage - BAT_EMPTY_V) * 100.0f / (BAT_FULL_V - BAT_EMPTY_V));
  battery.percent = static_cast<uint8_t>(constrain(percent, 0, 100));
  if (recordHistory) {
    ++batterySampleSequence;
    pushBatteryHistory(batterySampleSequence,
                       clampU16(lroundf(max(0.0f, battery.voltage) * 1000.0f)),
                       currentGpsEpoch(), wakeCycleSequence);
  }

  if (recordHistory && sdOk && !sdWriteError && !sdLowSpace) {
    File file = SD.open("/battery.csv", FILE_APPEND);
    if (file) {
      const size_t written = file.printf("%lu,%.3f,%.3f,%u,%u,%lu,%lu,%u\n", static_cast<unsigned long>(millis()), battery.voltage,
                                         battery.pinVoltage, battery.raw, battery.percent,
                                         static_cast<unsigned long>(batterySampleSequence),
                                         static_cast<unsigned long>(currentGpsEpoch()), wakeCycleSequence);
      file.flush();
      if (written == 0 || file.getWriteError()) sdWriteError = true;
      file.close();
    } else sdWriteError = true;
  }
}

static double distanceKm(double lat1, double lon1, double lat2, double lon2) {
  constexpr double earthRadiusKm = 6371.0088;
  constexpr double radians = 0.017453292519943295;
  const double deltaLat = (lat2 - lat1) * radians;
  const double deltaLon = (lon2 - lon1) * radians;
  const double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
                   cos(lat1 * radians) * cos(lat2 * radians) *
                       sin(deltaLon / 2.0) * sin(deltaLon / 2.0);
  return earthRadiusKm * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

static bool gpsFixValid() {
  return gps.location.isValid() && gps.location.age() < 5000 && gps.satellites.isValid() &&
         gps.satellites.value() >= settings.minSats && gps.hdop.isValid() &&
         gps.hdop.hdop() <= settings.maxHdop;
}

static int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
  const unsigned dayOfYear = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
  return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(dayOfEra) - 719468;
}

static uint32_t currentGpsEpoch() {
  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.age() >= 5000 || gps.time.age() >= 5000) {
    return 0;
  }
  const int year = gps.date.year();
  const unsigned month = gps.date.month();
  const unsigned day = gps.date.day();
  if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31) return 0;
  const int64_t seconds = daysFromCivil(year, month, day) * 86400LL +
                          static_cast<int64_t>(gps.time.hour()) * 3600LL +
                          static_cast<int64_t>(gps.time.minute()) * 60LL + gps.time.second();
  return seconds > 0 && seconds <= UINT32_MAX ? static_cast<uint32_t>(seconds) : 0;
}

static uint32_t recordingSeconds() {
  const uint32_t nowEpoch = currentGpsEpoch();
  if (routeStartGpsEpoch != 0 && nowEpoch >= routeStartGpsEpoch) {
    lastRouteElapsedSec = nowEpoch - routeStartGpsEpoch;
  }
  return lastRouteElapsedSec;
}

static void startRecording() {
  if (recording) return;
  recording = true;
  recordingSegmentStartMs = millis();
  hasLastPoint = false;  // Do not join a stopped interval into the route distance.
}

static void stopRecording() {
  if (!recording) return;
  accumulatedRecordingMs += millis() - recordingSegmentStartMs;
  recording = false;
  hasLastPoint = false;
}

static String jsonEscape(const char *text) {
  String result;
  if (!text) return result;
  while (*text) {
    const char value = *text++;
    if (value == '"' || value == '\\') result += '\\';
    if (static_cast<uint8_t>(value) >= 0x20) result += value;
  }
  return result;
}

static bool writeGeoPoint(double lat, double lon, float altitude, const char *kind, const char *status,
                          uint32_t recordId, const char *name = "", const char *comment = "") {
  if (!sdOk || sdWriteError || sdLowSpace) return false;
  File file = SD.open(routePath, FILE_APPEND);
  if (!file) { sdWriteError = true; return false; }
  const size_t written = file.printf(
      "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.7f,%.7f,%.1f]},"
      "\"properties\":{\"point_type\":\"%s\",\"uptime_ms\":%lu,\"dist_km\":%.4f,\"sats\":%lu,"
      "\"hdop\":%.2f,\"battery\":%.3f,\"gps_epoch\":%lu,\"speed_kmh\":%.1f,"
      "\"record_id\":%lu,\"cycle_id\":%u,\"status\":\"%s\",\"name\":\"%s\",\"comment\":\"%s\"}}\n",
      lon, lat, altitude, kind, static_cast<unsigned long>(millis()), totalDistanceKm,
      static_cast<unsigned long>(gps.satellites.isValid() ? gps.satellites.value() : 0),
      gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, battery.voltage,
      static_cast<unsigned long>(currentGpsEpoch()), gps.speed.isValid() ? gps.speed.kmph() : 0.0,
      static_cast<unsigned long>(recordId), wakeCycleSequence, status,
      jsonEscape(name).c_str(), jsonEscape(comment).c_str());
  file.flush();
  const bool ok = written > 0 && file.getWriteError() == 0;
  file.close();
  sdWriteError = !ok;
  if (ok) refreshSdHealth();
  return ok;
}

static void saveGpsPoint(const char *status) {
  if (!recording || !gps.location.isValid()) return;
  if (cyclePointCount < 255) ++cyclePointCount;
  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  const uint32_t epoch = currentGpsEpoch();
  const float rawAltitude = currentAltitudeM();
  if (!isfinite(filteredAltitudeM)) filteredAltitudeM = rawAltitude;
  else filteredAltitudeM = settings.filterEnabled ? filteredAltitudeM * 0.7f + rawAltitude * 0.3f : rawAltitude;
  const float altitude = filteredAltitudeM;

  if (settings.filterEnabled && filteredHasPoint) {
    const double segmentKm = distanceKm(filteredLastLat, filteredLastLon, lat, lon);
    const double reportedSpeed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    double impliedSpeed = 0.0;
    if (epoch > filteredLastEpoch && filteredLastEpoch != 0) impliedSpeed = segmentKm * 3600.0 / (epoch - filteredLastEpoch);
    if ((segmentKm < 0.002 && reportedSpeed < 1.0) || segmentKm > 0.25 || impliedSpeed > 120.0) return;
    if (fabsf(rawAltitude - filteredAltitudeM) > 50.0f) return;
  }
  filteredLastLat = lat;
  filteredLastLon = lon;
  filteredLastEpoch = epoch;
  filteredHasPoint = true;
  if (routeStartGpsEpoch == 0) {
    const uint32_t pointEpoch = currentGpsEpoch();
    if (pointEpoch != 0) {
      routeStartGpsEpoch = pointEpoch;
      lastRouteElapsedSec = 0;
      preferences.putUInt("route_t0", routeStartGpsEpoch);
    }
  }

  if (hasLastPoint) {
    const double segmentKm = distanceKm(lastLat, lastLon, lat, lon);
    if (segmentKm < 0.25) totalDistanceKm += static_cast<float>(segmentKm);
  } else {
    startAltitudeM = altitude;
  }

  lastLat = lat;
  lastLon = lon;
  hasLastPoint = true;
  const uint32_t nextPoint = pointCount + 1;
  if (!writeGeoPoint(lat, lon, altitude, "track", status, nextPoint)) return;
  pointCount = nextPoint;
  pushTrackHistory(lat, lon, altitude);
  saveCurrentTrackMeta();
  if (cyclePointCount >= settings.minPointsPerWake) markCycleComplete();
}

static bool saveUserPoint(const char *name = "", const char *comment = "") {
  if (!gps.location.isValid() || !sdOk) return false;
  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  const float altitude = currentAltitudeM();
  const uint32_t nextMark = routeMarkCount + 1;
  if (!writeGeoPoint(lat, lon, altitude, "user_point",
                     gpsFixValid() ? "USER_MARK" : "USER_MARK_WEAK", nextMark, name, comment)) return false;
  routeMarkCount = nextMark;
  pushUserPointHistory(lat, lon, altitude);
  saveCurrentTrackMeta();
  serviceLog("POINT", "SAVED", routePath);
  return true;
}

static void newRoute() {
  stopRecording();
  ++routeNumber;
  preferences.putUInt("route", routeNumber);
  updateRoutePath();
  totalDistanceKm = 0.0f;
  lastLat = 0.0;
  lastLon = 0.0;
  startAltitudeM = 0.0f;
  hasLastPoint = false;
  pointCount = 0;
  routeMarkCount = 0;
  filteredAltitudeM = NAN;
  filteredHasPoint = false;
  routeStartGpsEpoch = 0;
  lastRouteElapsedSec = 0;
  preferences.putUInt("route_t0", 0);
  clearTrackHistory();
  clearUserPointHistory();
  historyStreamPhase = HISTORY_IDLE;
  accumulatedRecordingMs = 0;
  firstFixHandled = false;
  gpsWaitStartMs = millis();
  startRecording();
  if (gpsFixValid()) {
    firstFixHandled = true;
    saveGpsPoint("NEW_ROUTE");
    lastTrackSaveMs = millis();
  }
  serviceLog("REC", "NEW_FILE", routePath);
}

class TrackerServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer *server) override {
    (void)server;
    bleConnected = true;
    bleWasConnected = true;
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    bleConnected = false;
    historyStreamPhase = HISTORY_IDLE;
    cancelTrackTransfer();
  }
};

class TrackerCommandCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic *characteristic) override {
    if (!bleCommandQueue) return;
    const String value = characteristic->getValue();
    if (value.length() == 0) return;
    BleCommand command = {};
    value.substring(0, sizeof(command.text) - 1).toCharArray(command.text, sizeof(command.text));
    xQueueSend(bleCommandQueue, &command, 0);
  }
};

static TrackerServerCallbacks trackerServerCallbacks;
static TrackerCommandCallbacks trackerCommandCallbacks;

static void publishSettings() {
  if (!settingsCharacteristic) return;
  uint8_t packet[14] = {};
  packet[0] = 5;
  putU16(packet, 1, settings.gpsTimeoutSec);
  packet[3] = settings.minSats;
  putU16(packet, 4, clampU16(lroundf(settings.maxHdop * 100.0f)));
  putU32(packet, 6, settings.sleepTimeSec);
  if (settings.screenEnabled) packet[10] |= 0x01;
  if (settings.bleEnabled) packet[10] |= 0x02;
  packet[11] = settings.minPointsPerWake;
  packet[12] = settings.filterEnabled ? 1 : 0;
  packet[13] = settings.sleepWhileBleConnected ? 1 : 0;
  settingsCharacteristic->setValue(packet, sizeof(packet));
}

static void sendBleResponse(const char *message) {
  Serial.printf("BLE %s\n", message);
  if (!responseCharacteristic) return;
  char shortMessage[20] = {};
  strncpy(shortMessage, message, sizeof(shortMessage) - 1);
  responseCharacteristic->setValue(reinterpret_cast<const uint8_t *>(shortMessage), strlen(shortMessage));
  if (bleConnected) responseCharacteristic->notify();
}

static void startHistoryStream() {
  historyStreamIndex = 0;
  lastHistoryPacketMs = 0;
  historyStreamPhase = HISTORY_TRACK_START;
}

static void serviceHistoryStream(uint32_t now) {
  if (historyStreamPhase == HISTORY_IDLE || !bleConnected || !historyCharacteristic) return;
  if (lastHistoryPacketMs != 0 && now - lastHistoryPacketMs < HISTORY_PACKET_MS) return;
  lastHistoryPacketMs = now;

  uint8_t packet[20] = {};
  packet[0] = 1;

  size_t packetSize = 4;

  if (historyStreamPhase == HISTORY_TRACK_START) {
    packet[1] = 1;
    packet[2] = trackHistoryCount;
    historyStreamIndex = 0;
    historyStreamPhase = trackHistoryCount ? HISTORY_TRACK_POINTS : HISTORY_BATTERY_START;
  } else if (historyStreamPhase == HISTORY_TRACK_POINTS) {
    packet[1] = 2;
    packet[2] = historyStreamIndex;
    const HistoryPoint &point = trackHistoryAt(historyStreamIndex);
    putI32(packet, 4, point.latE7);
    putI32(packet, 8, point.lonE7);
    putI16(packet, 12, point.altitudeM);
    packetSize = 14;
    if (++historyStreamIndex >= trackHistoryCount) {
      historyStreamPhase = HISTORY_BATTERY_START;
    }
  } else if (historyStreamPhase == HISTORY_BATTERY_START) {
    packet[1] = 3;
    packet[2] = batteryHistoryCount;
    packet[3] = 3;  // sequence, millivolts, GPS epoch and wake-cycle id.
    historyStreamIndex = 0;
    historyStreamPhase = batteryHistoryCount ? HISTORY_BATTERY_CHUNKS : HISTORY_USER_START;
  } else if (historyStreamPhase == HISTORY_BATTERY_CHUNKS) {
    packet[1] = 4;
    packet[2] = historyStreamIndex;
    const uint8_t remaining = batteryHistoryCount - historyStreamIndex;
    const uint8_t count = min<uint8_t>(1, remaining);
    packet[3] = count;
    for (uint8_t i = 0; i < count; ++i) {
      const BatteryHistorySample &sample = batteryHistoryAt(historyStreamIndex + i);
      putU32(packet, 4, sample.sequence);
      putU16(packet, 8, sample.millivolts);
      putU32(packet, 10, sample.epoch);
      putU16(packet, 14, sample.cycleId);
    }
    packetSize = 16;
    historyStreamIndex += count;
    if (historyStreamIndex >= batteryHistoryCount) historyStreamPhase = HISTORY_USER_START;
  } else if (historyStreamPhase == HISTORY_USER_START) {
    packet[1] = 6;
    packet[2] = userPointHistoryCount;
    historyStreamIndex = 0;
    historyStreamPhase = userPointHistoryCount ? HISTORY_USER_POINTS : HISTORY_END;
  } else if (historyStreamPhase == HISTORY_USER_POINTS) {
    packet[1] = 7;
    packet[2] = historyStreamIndex;
    const HistoryPoint &point = userPointHistoryAt(historyStreamIndex);
    putI32(packet, 4, point.latE7);
    putI32(packet, 8, point.lonE7);
    putI16(packet, 12, point.altitudeM);
    packetSize = 14;
    if (++historyStreamIndex >= userPointHistoryCount) historyStreamPhase = HISTORY_END;
  } else if (historyStreamPhase == HISTORY_END) {
    packet[1] = 5;
    packet[2] = trackHistoryCount;
    packet[3] = userPointHistoryCount;
    historyStreamPhase = HISTORY_IDLE;
  }

  historyCharacteristic->setValue(packet, packetSize);
  historyCharacteristic->notify();
}

static bool routeFromFilename(const char *name, uint16_t &number) {
  if (!name) return false;
  const char *base = strrchr(name, '/');
  base = base ? base + 1 : name;
  unsigned long parsed = 0;
  char tail = 0;
  if (sscanf(base, "track_%lu.geojsonl%c", &parsed, &tail) != 1 || parsed == 0 || parsed > 65535) {
    return false;
  }
  number = static_cast<uint16_t>(parsed);
  return true;
}

static String trackDisplayName(uint16_t number) {
  char path[32] = {};
  snprintf(path, sizeof(path), "/track_%03u.name", number);
  File file = SD.open(path, FILE_READ);
  if (!file) return String();
  String value = file.readStringUntil('\n');
  file.close();
  value.trim();
  return value;
}

static bool saveTrackDisplayName(uint16_t number, const char *name) {
  char path[32] = {};
  snprintf(path, sizeof(path), "/track_%03u.name", number);
  File file = SD.open(path, FILE_WRITE);
  if (!file) return false;
  file.println(name ? name : "");
  file.close();
  return true;
}

static void saveCurrentTrackMeta() {
  if (!sdOk || sdWriteError || sdLowSpace) return;
  char path[32] = {};
  char tempPath[32] = {};
  char backupPath[32] = {};
  snprintf(path, sizeof(path), "/track_%03lu.meta", static_cast<unsigned long>(routeNumber));
  snprintf(tempPath, sizeof(tempPath), "/track_%03lu.meta.tmp", static_cast<unsigned long>(routeNumber));
  snprintf(backupPath, sizeof(backupPath), "/track_%03lu.meta.bak", static_cast<unsigned long>(routeNumber));
  SD.remove(tempPath);
  SD.remove(backupPath);
  File file = SD.open(tempPath, FILE_WRITE);
  if (!file) { sdWriteError = true; return; }
  const size_t written = file.printf("%lu,%lu,%lu,%lu\n", static_cast<unsigned long>(pointCount),
                                     static_cast<unsigned long>(routeMarkCount),
                                     static_cast<unsigned long>(routeStartGpsEpoch),
                                     static_cast<unsigned long>(currentGpsEpoch()));
  file.flush();
  const bool writeOk = written > 0 && file.getWriteError() == 0;
  file.close();
  if (!writeOk) { SD.remove(tempPath); sdWriteError = true; return; }
  const bool hadOriginal = SD.exists(path);
  if (hadOriginal && !SD.rename(path, backupPath)) { SD.remove(tempPath); sdWriteError = true; return; }
  if (!SD.rename(tempPath, path)) {
    if (hadOriginal) SD.rename(backupPath, path);
    sdWriteError = true;
    return;
  }
  if (hadOriginal) SD.remove(backupPath);
}

static void loadTrackMeta(uint16_t number) {
  pendingMetaPoints = pendingMetaMarks = pendingMetaCreated = pendingMetaUpdated = 0;
  char path[32] = {};
  snprintf(path, sizeof(path), "/track_%03u.meta", number);
  File file = SD.open(path, FILE_READ);
  if (!file) return;
  const String line = file.readStringUntil('\n');
  file.close();
  sscanf(line.c_str(), "%lu,%lu,%lu,%lu", &pendingMetaPoints, &pendingMetaMarks,
         &pendingMetaCreated, &pendingMetaUpdated);
}

static void cancelTrackTransfer() {
  if (trackTransferFile) trackTransferFile.close();
  if (trackTransferDirectory) trackTransferDirectory.close();
  trackTransferPhase = TRACK_TRANSFER_IDLE;
}

static void startTrackList() {
  cancelTrackTransfer();
  if (!sdOk || !trackTransferCharacteristic) return;
  trackTransferDirectory = SD.open("/");
  trackTransferPhase = trackTransferDirectory ? TRACK_LIST_START : TRACK_TRANSFER_IDLE;
  lastTrackTransferPacketMs = 0;
}

static bool startTrackLoad(uint16_t number, uint32_t skipPoints = 0, uint32_t skipMarks = 0) {
  cancelTrackTransfer();
  if (!sdOk || !trackTransferCharacteristic || number == 0) return false;
  char path[32] = {};
  snprintf(path, sizeof(path), "/track_%03u.geojsonl", number);
  trackTransferFile = SD.open(path, FILE_READ);
  if (!trackTransferFile) return false;
  trackTransferRoute = number;
  trackTransferSize = trackTransferFile.size();
  trackTransferPointCount = 0;
  trackTransferMarkCount = 0;
  trackTransferSkipPoints = skipPoints;
  trackTransferSkipMarks = skipMarks;
  trackTransferIncremental = skipPoints > 0 || skipMarks > 0;
  trackTransferPhase = TRACK_LOAD_START;
  lastTrackTransferPacketMs = 0;
  return true;
}

static bool startServiceLogTransfer() {
  cancelTrackTransfer();
  if (!sdOk || !trackTransferCharacteristic || !SD.exists("/service.csv")) return false;
  trackTransferFile = SD.open("/service.csv", FILE_READ);
  if (!trackTransferFile) return false;
  trackTransferSize = trackTransferFile.size();
  // Keep the diagnostic transfer bounded on long-running cards.
  if (trackTransferSize > 32768) {
    trackTransferFile.seek(trackTransferSize - 32768);
    trackTransferFile.readStringUntil('\n');
  }
  trackTransferPhase = TRACK_LOG_START;
  lastTrackTransferPacketMs = 0;
  return true;
}

static void notifyTrackTransfer(uint8_t *packet, size_t size) {
  trackTransferCharacteristic->setValue(packet, size);
  trackTransferCharacteristic->notify();
}

static double jsonNumber(const String &line, const char *key) {
  const int marker = line.indexOf(key);
  return marker >= 0 ? strtod(line.c_str() + marker + strlen(key), nullptr) : 0.0;
}

static String jsonText(const String &line, const char *key) {
  const int marker = line.indexOf(key);
  if (marker < 0) return String();
  const int start = marker + strlen(key);
  const int end = line.indexOf('"', start);
  return end > start ? line.substring(start, end) : String();
}

static void serviceTrackTransfer(uint32_t now) {
  if (trackTransferPhase == TRACK_TRANSFER_IDLE || !bleConnected || !trackTransferCharacteristic) return;
  if (lastTrackTransferPacketMs && now - lastTrackTransferPacketMs < HISTORY_PACKET_MS) return;
  lastTrackTransferPacketMs = now;
  uint8_t packet[20] = {};
  packet[0] = 1;

  if (trackTransferPhase == TRACK_LOG_START) {
    packet[1] = 20;
    putU32(packet, 2, trackTransferFile.available());
    notifyTrackTransfer(packet, 6);
    trackTransferPhase = TRACK_LOG_CHUNKS;
    return;
  }
  if (trackTransferPhase == TRACK_LOG_CHUNKS) {
    packet[1] = 21;
    const int count = trackTransferFile.read(packet + 2, 18);
    if (count > 0) {
      notifyTrackTransfer(packet, 2 + static_cast<size_t>(count));
      return;
    }
    trackTransferFile.close();
    trackTransferPhase = TRACK_LOG_END;
  }
  if (trackTransferPhase == TRACK_LOG_END) {
    packet[1] = 22;
    notifyTrackTransfer(packet, 2);
    trackTransferPhase = TRACK_TRANSFER_IDLE;
    return;
  }

  if (trackTransferPhase == TRACK_LOAD_POINT_DETAIL) {
    packet[1] = 12;
    packet[2] = pendingPointType;
    putU16(packet, 3, pendingPointIndex);
    putU16(packet, 5, pendingPointHdop);
    packet[7] = pendingPointSats;
    putU16(packet, 8, pendingPointRecordId);
    putU16(packet, 10, pendingPointCycleId);
    notifyTrackTransfer(packet, 12);
    trackTransferPhase = TRACK_LOAD_POINT_STATUS;
    return;
  }
  if (trackTransferPhase == TRACK_LOAD_POINT_STATUS) {
    packet[1] = 13;
    packet[2] = pendingPointType;
    putU16(packet, 3, pendingPointIndex);
    const uint8_t length = min<uint8_t>(14, pendingPointStatus.length());
    packet[5] = length;
    memcpy(packet + 6, pendingPointStatus.c_str(), length);
    notifyTrackTransfer(packet, 6 + length);
    trackTransferPhase = pendingPointType ? TRACK_LOAD_MARK_NAME : TRACK_LOAD_POINTS;
    return;
  }

  if (trackTransferPhase == TRACK_LOAD_MARK_NAME || trackTransferPhase == TRACK_LOAD_MARK_COMMENT) {
    const String &text = trackTransferPhase == TRACK_LOAD_MARK_NAME ? pendingMarkName : pendingMarkComment;
    packet[1] = trackTransferPhase == TRACK_LOAD_MARK_NAME ? 8 : 9;
    putU16(packet, 2, pendingMarkIndex);
    const uint8_t length = min<uint8_t>(15, text.length());
    packet[4] = length;
    memcpy(packet + 5, text.c_str(), length);
    notifyTrackTransfer(packet, 5 + length);
    if (trackTransferPhase == TRACK_LOAD_MARK_NAME) trackTransferPhase = TRACK_LOAD_MARK_COMMENT;
    else trackTransferPhase = TRACK_LOAD_POINTS;
    return;
  }

  if (trackTransferPhase == TRACK_LIST_START) {
    packet[1] = 1;
    putU16(packet, 2, clampU16(routeNumber));
    putU32(packet, 4, static_cast<uint32_t>(SD.totalBytes() / (1024ULL * 1024ULL)));
    putU32(packet, 8, static_cast<uint32_t>((SD.totalBytes() - SD.usedBytes()) / (1024ULL * 1024ULL)));
    notifyTrackTransfer(packet, 12);
    trackTransferPhase = TRACK_LIST_ITEMS;
    return;
  }

  if (trackTransferPhase == TRACK_LIST_NAME) {
    packet[1] = 10;
    putU16(packet, 2, pendingListRoute);
    const uint8_t length = min<uint8_t>(15, pendingListName.length());
    packet[4] = length;
    memcpy(packet + 5, pendingListName.c_str(), length);
    notifyTrackTransfer(packet, 5 + length);
    trackTransferPhase = TRACK_LIST_META;
    return;
  }

  if (trackTransferPhase == TRACK_LIST_META) {
    packet[1] = 11;
    putU16(packet, 2, pendingListRoute);
    putU32(packet, 4, pendingMetaPoints);
    putU32(packet, 8, pendingMetaMarks);
    putU32(packet, 12, pendingMetaCreated);
    putU32(packet, 16, pendingMetaUpdated);
    notifyTrackTransfer(packet, 20);
    trackTransferPhase = TRACK_LIST_ITEMS;
    return;
  }

  if (trackTransferPhase == TRACK_LIST_ITEMS) {
    while (true) {
      File item = trackTransferDirectory.openNextFile();
      if (!item) {
        packet[1] = 3;
        notifyTrackTransfer(packet, 2);
        trackTransferDirectory.close();
        trackTransferPhase = TRACK_TRANSFER_IDLE;
        return;
      }
      uint16_t number = 0;
      const bool valid = !item.isDirectory() && routeFromFilename(item.name(), number);
      const uint32_t size = item.size();
      item.close();
      if (!valid) continue;
      packet[1] = 2;
      putU16(packet, 2, number);
      putU32(packet, 4, size);
      packet[8] = number == routeNumber ? 0x01 : 0x00;
      notifyTrackTransfer(packet, 9);
      pendingListRoute = number;
      pendingListName = trackDisplayName(number);
      loadTrackMeta(number);
      trackTransferPhase = TRACK_LIST_NAME;
      return;
    }
  }

  if (trackTransferPhase == TRACK_LOAD_START) {
    packet[1] = 4;
    putU16(packet, 2, trackTransferRoute);
    putU32(packet, 4, trackTransferSize);
    packet[8] = trackTransferRoute == routeNumber ? 0x01 : 0x00;
    packet[9] = trackTransferIncremental ? 0x01 : 0x00;
    putU32(packet, 10, trackTransferSkipPoints);
    putU32(packet, 14, trackTransferSkipMarks);
    notifyTrackTransfer(packet, 18);
    trackTransferPhase = TRACK_LOAD_POINTS;
    return;
  }

  if (trackTransferPhase == TRACK_LOAD_POINTS) {
    while (trackTransferFile.available()) {
      const String line = trackTransferFile.readStringUntil('\n');
      const bool isTrack = line.indexOf("\"point_type\":\"track\"") >= 0;
      const bool isMark = line.indexOf("\"point_type\":\"user_point\"") >= 0;
      if (!isTrack && !isMark) continue;
      if (isMark) {
        ++trackTransferMarkCount;
        if (trackTransferMarkCount <= trackTransferSkipMarks) continue;
      } else {
        ++trackTransferPointCount;
        if (trackTransferPointCount <= trackTransferSkipPoints) continue;
      }
      const int marker = line.indexOf("\"coordinates\":[");
      if (marker < 0) continue;
      double lon = 0.0;
      double lat = 0.0;
      float altitude = 0.0f;
      if (sscanf(line.c_str() + marker + 15, "%lf,%lf,%f", &lon, &lat, &altitude) != 3) continue;
      packet[1] = 5;
      packet[2] = isMark ? 1 : 0;
      putI32(packet, 4, static_cast<int32_t>(lround(lat * 10000000.0)));
      putI32(packet, 8, static_cast<int32_t>(lround(lon * 10000000.0)));
      putI16(packet, 12, clampI16(lroundf(altitude)));
      putU16(packet, 14, clampU16(lround(jsonNumber(line, "\"speed_kmh\":") * 10.0)));
      putU32(packet, 16, static_cast<uint32_t>(jsonNumber(line, "\"gps_epoch\":")));
      pendingPointType = isMark ? 1 : 0;
      pendingPointIndex = static_cast<uint16_t>(min<uint32_t>(65535,
          (isMark ? trackTransferMarkCount : trackTransferPointCount) - 1));
      pendingPointHdop = clampU16(lround(jsonNumber(line, "\"hdop\":") * 100.0));
      pendingPointSats = min<uint32_t>(255, lround(jsonNumber(line, "\"sats\":")));
      pendingPointRecordId = clampU16(lround(jsonNumber(line, "\"record_id\":")));
      pendingPointCycleId = clampU16(lround(jsonNumber(line, "\"cycle_id\":")));
      pendingPointStatus = jsonText(line, "\"status\":\"");
      if (isMark) {
        pendingMarkIndex = static_cast<uint16_t>(min<uint32_t>(65535, trackTransferMarkCount - 1));
        pendingMarkName = jsonText(line, "\"name\":\"");
        pendingMarkComment = jsonText(line, "\"comment\":\"");
      }
      trackTransferPhase = TRACK_LOAD_POINT_DETAIL;
      notifyTrackTransfer(packet, 20);
      return;
    }
    trackTransferFile.close();
    trackTransferPhase = TRACK_LOAD_END;
  }

  if (trackTransferPhase == TRACK_LOAD_END) {
    packet[1] = 7;
    putU16(packet, 2, trackTransferRoute);
    putU32(packet, 4, trackTransferPointCount);
    putU32(packet, 8, trackTransferMarkCount);
    notifyTrackTransfer(packet, 12);
    trackTransferPhase = TRACK_TRANSFER_IDLE;
  }
}

static void publishTelemetry(bool notifyClient) {
  if (!telemetryCharacteristic || !navCharacteristic) return;

  uint8_t telemetry[20] = {};
  telemetry[0] = 1;
  if (gpsFixValid()) telemetry[1] |= 0x01;
  if (recording) telemetry[1] |= 0x02;
  if (sdOk) telemetry[1] |= 0x04;
  if (gps.altitude.isValid() && gps.altitude.age() < 5000) telemetry[1] |= 0x08;
  if (sdWriteError) telemetry[1] |= 0x10;
  if (sdLowSpace) telemetry[1] |= 0x20;
  if (sdTailRepaired) telemetry[1] |= 0x40;
  if (usbHostConnected()) telemetry[1] |= 0x80;
  putU32(telemetry, 2, static_cast<uint32_t>(max(0.0f, totalDistanceKm) * 1000.0f + 0.5f));
  putI16(telemetry, 6, clampI16(lroundf(currentAltitudeM())));
  putU16(telemetry, 8, clampU16(lroundf(max(0.0f, battery.voltage) * 1000.0f)));
  telemetry[10] = battery.percent;
  telemetry[11] = static_cast<uint8_t>(
      min<uint32_t>(255, gps.satellites.isValid() ? gps.satellites.value() : 0));
  putU16(telemetry, 12,
         clampU16(lround(gps.speed.isValid() ? gps.speed.kmph() * 10.0 : 0.0)));
  putU16(telemetry, 14,
         clampU16(lround(gps.hdop.isValid() ? gps.hdop.hdop() * 100.0 : 0.0)));
  putU16(telemetry, 16, clampU16(routeNumber));
  putU16(telemetry, 18, clampU16(pointCount));
  telemetryCharacteristic->setValue(telemetry, sizeof(telemetry));

  uint8_t nav[20] = {};
  putI32(nav, 0, gps.location.isValid() ? static_cast<int32_t>(lround(gps.location.lat() * 10000000.0)) : 0);
  putI32(nav, 4, gps.location.isValid() ? static_cast<int32_t>(lround(gps.location.lng() * 10000000.0)) : 0);
  putI16(nav, 8, clampI16(lroundf((gps.altitude.isValid() && hasLastPoint)
                                      ? currentAltitudeM() - startAltitudeM
                                      : 0.0f)));
  putU16(nav, 10,
         clampU16(lround(gps.course.isValid() ? gps.course.deg() * 10.0 : 0.0)));
  putU32(nav, 12, recordingSeconds());
  putU32(nav, 16, (millis() - bootMs) / 1000);
  navCharacteristic->setValue(nav, sizeof(nav));

  if (notifyClient && bleConnected) {
    telemetryCharacteristic->notify();
    navCharacteristic->notify();
  }

  if (batteryCharacteristic) {
    batteryCharacteristic->setValue(&battery.percent, 1);
    if (notifyClient && bleConnected) batteryCharacteristic->notify();
  }
}

static void publishCycleStatus(bool notifyClient) {
  if (!cycleStatusCharacteristic) return;
  uint8_t packet[20] = {};
  packet[0] = 1;
  uint8_t state = 1;  // GPS wait / point collection.
  if (usbHostConnected() && cycleComplete) state = 7;
  else if (sleepAnnounced) state = 6;
  else if (trackTransferPhase != TRACK_TRANSFER_IDLE) state = 2;
  else if (cycleComplete && bleConnected) state = 3;
  else if (cycleComplete && configurationWake && !bleWasConnected &&
           (bleConnectWindowStartMs == 0 || millis() - bleConnectWindowStartMs < BLE_CONNECT_WINDOW_MS)) state = 4;
  else if (cycleComplete) state = 5;
  packet[1] = state;
  packet[2] = bootWakeCause == ESP_SLEEP_WAKEUP_TIMER ? 1 :
              bootWakeCause == ESP_SLEEP_WAKEUP_GPIO ? 2 : 3;
  if (gpsFixValid()) packet[3] |= 0x01;
  if (bleConnected) packet[3] |= 0x02;
  if (settings.filterEnabled) packet[3] |= 0x04;
  if (cycleComplete) packet[3] |= 0x08;
  if (cycleAcknowledged) packet[3] |= 0x10;
  if (usbHostConnected()) packet[3] |= 0x20;
  packet[4] = cyclePointCount;
  packet[5] = settings.minPointsPerWake;
  putU16(packet, 6, clampU16((millis() - gpsWaitStartMs) / 1000));
  putU16(packet, 8, settings.gpsTimeoutSec);
  putU32(packet, 10, settings.sleepTimeSec);
  putU16(packet, 14, wakeCycleSequence);
  putU32(packet, 16, batterySampleSequence);
  cycleStatusCharacteristic->setValue(packet, sizeof(packet));
  if (notifyClient && bleConnected) cycleStatusCharacteristic->notify();
}

static bool initBle() {
  bleCommandQueue = xQueueCreate(8, sizeof(BleCommand));
  if (!bleCommandQueue) {
    Serial.println("ERR BLE command queue");
    return false;
  }
  if (!BLEDevice::init(BLE_DEVICE_NAME)) {
    Serial.println("ERR BLE init");
    return false;
  }
  BLEDevice::setMTU(64);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(&trackerServerCallbacks);
  server->advertiseOnDisconnect(true);

  BLEService *trackerService = server->createService(BLEUUID(String(TRACKER_SERVICE_UUID)), 32);
  telemetryCharacteristic = trackerService->createCharacteristic(
      TELEMETRY_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  navCharacteristic = trackerService->createCharacteristic(
      NAV_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  settingsCharacteristic = trackerService->createCharacteristic(SETTINGS_CHAR_UUID,
                                                                 BLECharacteristic::PROPERTY_READ);
  BLECharacteristic *commandCharacteristic = trackerService->createCharacteristic(
      COMMAND_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  commandCharacteristic->setCallbacks(&trackerCommandCallbacks);
  responseCharacteristic = trackerService->createCharacteristic(
      RESPONSE_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  historyCharacteristic = trackerService->createCharacteristic(
      HISTORY_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  uint8_t initialHistoryPacket[4] = {1, 5, 0, 0};
  historyCharacteristic->setValue(initialHistoryPacket, sizeof(initialHistoryPacket));
  trackTransferCharacteristic = trackerService->createCharacteristic(
      TRACK_TRANSFER_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  uint8_t initialTrackPacket[2] = {1, 3};
  trackTransferCharacteristic->setValue(initialTrackPacket, sizeof(initialTrackPacket));
  cycleStatusCharacteristic = trackerService->createCharacteristic(
      CYCLE_STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  trackerService->start();

  BLEService *batteryService = server->createService(BLEUUID(static_cast<uint16_t>(0x180F)));
  batteryCharacteristic = batteryService->createCharacteristic(
      BLEUUID(static_cast<uint16_t>(0x2A19)),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  batteryService->start();

  publishSettings();
  publishTelemetry(false);
  publishCycleStatus(false);

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(TRACKER_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  bleStarted = true;
  Serial.println("BLE advertising as C6 Tracker");
  return true;
}

static bool parseIntegerSetting(const char *command, const char *prefix, long &value) {
  const size_t prefixLength = strlen(prefix);
  if (strncmp(command, prefix, prefixLength) != 0) return false;
  char *end = nullptr;
  value = strtol(command + prefixLength, &end, 10);
  return end && *end == '\0';
}

static void processBleCommand(const char *command) {
  if (strcmp(command, "rec=1") == 0) {
    startRecording();
    if (gpsFixValid()) {
      firstFixHandled = true;
      saveGpsPoint("REC_START");
      lastTrackSaveMs = millis();
    } else {
      firstFixHandled = false;
      gpsWaitStartMs = millis();
    }
    serviceLog("REC", "START", routePath);
    sendBleResponse("OK REC START");
  } else if (strcmp(command, "rec=0") == 0) {
    stopRecording();
    firstFixHandled = true;
    serviceLog("REC", "STOP", routePath);
    sendBleResponse("OK REC STOP");
  } else if (strcmp(command, "mark") == 0 || strncmp(command, "mark=", 5) == 0) {
    if (!sdOk) {
      sendBleResponse("ERR NO SD");
    } else if (!gps.location.isValid()) {
      sendBleResponse("ERR NO GPS");
    } else {
      char markData[80] = {};
      const char *name = "";
      const char *comment = "";
      if (strncmp(command, "mark=", 5) == 0) {
        strncpy(markData, command + 5, sizeof(markData) - 1);
        name = markData;
        char *separator = strchr(markData, '|');
        if (separator) {
          *separator = '\0';
          comment = separator + 1;
        }
      }
      if (saveUserPoint(name, comment)) {
      sendBleResponse("OK MARK SAVED");
      }
    }
  } else if (strcmp(command, "new") == 0) {
    newRoute();
    sendBleResponse("OK NEW ROUTE");
  } else if (strcmp(command, "settings?") == 0) {
    publishSettings();
    sendBleResponse("OK SETTINGS");
  } else if (strcmp(command, "history") == 0) {
    startHistoryStream();
    sendBleResponse("OK HISTORY");
  } else if (strcmp(command, "tracks") == 0) {
    if (!sdOk) sendBleResponse("ERR NO SD");
    else {
      startTrackList();
      sendBleResponse("OK TRACK LIST");
    }
  } else if (strcmp(command, "logs") == 0) {
    if (startServiceLogTransfer()) sendBleResponse("OK LOGS");
    else sendBleResponse("ERR NO LOGS");
  } else if (strcmp(command, "cancel") == 0) {
    cancelTrackTransfer();
    sendBleResponse("OK CANCEL");
  } else if (strncmp(command, "rename=", 7) == 0) {
    char *separator = strchr(const_cast<char *>(command) + 7, '|');
    if (!separator) sendBleResponse("ERR BAD NAME");
    else {
      *separator = '\0';
      const long number = strtol(command + 7, nullptr, 10);
      if (number < 1 || number > 65535 || !saveTrackDisplayName(number, separator + 1)) sendBleResponse("ERR RENAME");
      else sendBleResponse("OK RENAME");
    }
  } else if (strncmp(command, "delete=", 7) == 0) {
    const long number = strtol(command + 7, nullptr, 10);
    if (number < 1 || number > 65535 || static_cast<uint32_t>(number) == routeNumber) {
      sendBleResponse("ERR DELETE");
    } else {
      char path[32] = {};
      snprintf(path, sizeof(path), "/track_%03ld.geojsonl", number);
      const bool removed = SD.remove(path);
      snprintf(path, sizeof(path), "/track_%03ld.name", number);
      SD.remove(path);
      snprintf(path, sizeof(path), "/track_%03ld.meta", number);
      SD.remove(path);
      refreshSdHealth();
      sendBleResponse(removed ? "OK DELETE" : "ERR DELETE");
    }
  } else {
    long value = 0;
    long syncRoute = 0;
    unsigned long syncPoints = 0, syncMarks = 0;
    if (sscanf(command, "sync=%ld|%lu|%lu", &syncRoute, &syncPoints, &syncMarks) == 3 &&
        syncRoute >= 1 && syncRoute <= 65535) {
      if (startTrackLoad(static_cast<uint16_t>(syncRoute), syncPoints, syncMarks)) {
        sendBleResponse("OK TRACK SYNC");
      } else sendBleResponse("ERR NO TRACK");
    } else if (parseIntegerSetting(command, "ack=", value) && value >= 0 && value <= 65535) {
      if (static_cast<uint16_t>(value) == wakeCycleSequence) {
        cycleAcknowledged = true;
        sendBleResponse("OK CYCLE ACK");
        publishCycleStatus(true);
      } else sendBleResponse("ERR OLD CYCLE");
    } else if (parseIntegerSetting(command, "load=", value) && value >= 1 && value <= 65535) {
      if (startTrackLoad(static_cast<uint16_t>(value))) sendBleResponse("OK TRACK LOAD");
      else sendBleResponse("ERR NO TRACK");
    } else if (parseIntegerSetting(command, "gpsto=", value) && value >= 30 && value <= 3600) {
      settings.gpsTimeoutSec = static_cast<uint16_t>(value);
      saveSettings();
      sendBleResponse("OK GPS TIMEOUT");
    } else if (parseIntegerSetting(command, "sats=", value) && value >= 1 && value <= 20) {
      settings.minSats = static_cast<uint8_t>(value);
      saveSettings();
      sendBleResponse("OK MIN SATS");
    } else if (parseIntegerSetting(command, "hdop=", value) && value >= 50 && value <= 2000) {
      settings.maxHdop = value / 100.0f;
      saveSettings();
      sendBleResponse("OK MAX HDOP");
    } else if (parseIntegerSetting(command, "sleep=", value) && value >= 10 && value <= 86400) {
      settings.sleepTimeSec = static_cast<uint32_t>(value);
      saveSettings();
      sendBleResponse("OK SLEEP TIME");
    } else if (parseIntegerSetting(command, "screen=", value) && (value == 0 || value == 1)) {
      settings.screenEnabled = value != 0;
      saveSettings();
      sendBleResponse("OK SCREEN");
    } else if (parseIntegerSetting(command, "ble=", value) && (value == 0 || value == 1)) {
      settings.bleEnabled = value != 0;
      saveSettings();
      sendBleResponse("OK BLE");
    } else if (parseIntegerSetting(command, "minpts=", value) && value >= 1 && value <= 100) {
      settings.minPointsPerWake = static_cast<uint8_t>(value);
      saveSettings();
      sendBleResponse("OK MIN POINTS");
    } else if (parseIntegerSetting(command, "filter=", value) && (value == 0 || value == 1)) {
      settings.filterEnabled = value != 0;
      saveSettings();
      sendBleResponse("OK FILTER");
    } else if (parseIntegerSetting(command, "sleepbt=", value) && (value == 0 || value == 1)) {
      settings.sleepWhileBleConnected = value != 0;
      saveSettings();
      sendBleResponse("OK SLEEP BT");
    } else {
      sendBleResponse("ERR BAD COMMAND");
    }
  }
  publishTelemetry(true);
}

static void processBleQueue() {
  if (!bleCommandQueue) return;
  BleCommand command = {};
  while (xQueueReceive(bleCommandQueue, &command, 0) == pdTRUE) {
    processBleCommand(command.text);
  }
}

static void updateGps() {
  bool updated = false;
  while (gpsSerial.available()) {
    const char byte = static_cast<char>(gpsSerial.read());
    if (gpsRawUsb && Serial) Serial.write(byte);
    if (gps.encode(byte) &&
        (gps.location.isUpdated() || gps.satellites.isUpdated() || gps.hdop.isUpdated() ||
         gps.speed.isUpdated() || gps.altitude.isUpdated() || gps.course.isUpdated())) {
      updated = true;
    }
  }

  if (updated && recording && firstFixHandled && gpsFixValid() &&
      millis() - lastTrackSaveMs >= TRACK_SAVE_MS) {
    lastTrackSaveMs = millis();
    saveGpsPoint("TRACK");
  }
}

static void updateInitialFix() {
  if (!cycleComplete && millis() - gpsWaitStartMs > settings.gpsTimeoutSec * 1000UL) {
    firstFixHandled = true;
    serviceLog("GPS", "CYCLE_TIMEOUT", "sleep_with_available_points");
    markCycleComplete();
    return;
  }
  if (firstFixHandled) return;
  if (gpsFixValid()) {
    firstFixHandled = true;
    startRecording();
    saveGpsPoint("START_FIX");
    lastTrackSaveMs = millis();
    serviceLog("REC", "START", routePath);
  }
}

static void sendUbx(uint8_t messageClass, uint8_t messageId, const uint8_t *payload, uint16_t length) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  auto writeChecked = [&](uint8_t value) {
    gpsSerial.write(value);
    ckA += value;
    ckB += ckA;
  };
  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  writeChecked(messageClass);
  writeChecked(messageId);
  writeChecked(static_cast<uint8_t>(length));
  writeChecked(static_cast<uint8_t>(length >> 8));
  for (uint16_t index = 0; index < length; ++index) writeChecked(payload[index]);
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
  gpsSerial.flush();
}

static void sleepGpsM10() {
  uint8_t payload[16] = {};
  payload[8] = 0x06;   // backup + force minimum consumption.
  payload[12] = 0x08;  // wake on an edge at UART RX.
  sendUbx(0x02, 0x41, payload, sizeof(payload));
  serviceLog("GPS", "SLEEP", "UBX-RXM-PMREQ");
  delay(100);
  gpsSerial.end();
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_TX, HIGH);
  gpio_hold_en(static_cast<gpio_num_t>(GPS_TX));
}

static void enterDeepSleep() {
  if (digitalRead(WAKE_BUTTON_PIN) == LOW) return;
  serviceLog("SLEEP", "ENTER", "timer_or_button");
  sleepGpsM10();
  if (displayReady) {
    lv_timer_handler();
    digitalWrite(LCD_BL, LOW);
  }
  if (bleStarted) BLEDevice::deinit(true);
  if (sdOk) {
    SD.end();
    sdOk = false;
  }
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(settings.sleepTimeSec) * 1000000ULL);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << WAKE_BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  Serial.flush();
  esp_deep_sleep_start();
}

static void serviceSleepCycle() {
  if (!cycleComplete) return;
  if (!cycleBatteryRecorded) {
    sampleBattery(true);
    cycleBatteryRecorded = true;
    publishTelemetry(true);
    publishCycleStatus(true);
  }
  if (usbHostConnected()) return;
  if (runtimeBleEnabled && !configurationWake && !cycleAcknowledged &&
      millis() - cycleCompleteMs < BLE_CYCLE_ACK_WINDOW_MS) return;
  if (bleConnected) {
    if (!settings.sleepWhileBleConnected) return;
    if (!sleepAnnounced) {
      sleepAnnounced = true;
      sleepAnnouncedMs = millis();
      publishCycleStatus(true);
      sendBleResponse("SLEEPING");
      return;
    }
    if (millis() - sleepAnnouncedMs < 500) return;
  }
  if (configurationWake && !bleWasConnected) {
    if (bleConnectWindowStartMs == 0) bleConnectWindowStartMs = millis();
    if (millis() - bleConnectWindowStartMs < BLE_CONNECT_WINDOW_MS) return;
  }
  enterDeepSleep();
}

static void printStatus() {
  Serial.printf(
      "STAT fix=%d rec=%d sd=%d ble=%d file=%s dist=%.3f bat=%.2fV %u%% sats=%lu hdop=%.2f "
      "lat=%.7f lon=%.7f alt=%.1f spd=%.1f\n",
      gpsFixValid(), recording, sdOk, bleConnected, routePath, totalDistanceKm, battery.voltage,
      battery.percent,
      static_cast<unsigned long>(gps.satellites.isValid() ? gps.satellites.value() : 0),
      gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
      gps.location.isValid() ? gps.location.lat() : 0.0,
      gps.location.isValid() ? gps.location.lng() : 0.0,
      gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
      gps.speed.isValid() ? gps.speed.kmph() : 0.0);
}

static void processSerialCommand(char *command) {
  while (*command == ' ') ++command;
  if (strcmp(command, "status") == 0) {
    printStatus();
  } else if (strcmp(command, "rec start") == 0) {
    processBleCommand("rec=1");
  } else if (strcmp(command, "rec stop") == 0) {
    processBleCommand("rec=0");
  } else if (strcmp(command, "new file") == 0) {
    processBleCommand("new");
  } else if (strcmp(command, "mark") == 0) {
    processBleCommand("mark");
  } else if (strcmp(command, "gps raw on") == 0) {
    gpsRawUsb = true;
    Serial.println("OK GPS RAW ON");
  } else if (strcmp(command, "gps raw off") == 0) {
    gpsRawUsb = false;
    Serial.println("OK GPS RAW OFF");
  } else if (strcmp(command, "help") == 0) {
    Serial.println("status | rec start/stop | new file | mark | gps raw on/off");
  } else if (*command) {
    Serial.println("ERR unknown command");
  }
}

static void pollSerial() {
  while (Serial.available()) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\r') continue;
    if (value == '\n') {
      serialCommand[serialCommandLength] = '\0';
      processSerialCommand(serialCommand);
      serialCommandLength = 0;
    } else if (serialCommandLength < sizeof(serialCommand) - 1) {
      serialCommand[serialCommandLength++] = value;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);
  bootMs = millis();
  if (++wakeCycleSequence == 0) ++wakeCycleSequence;
  lastUsbFrame = USB_SERIAL_JTAG.fram_num.sof_frame_index;
  gpsWaitStartMs = bootMs;
  pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_TX, HIGH);
  gpio_hold_dis(static_cast<gpio_num_t>(GPS_TX));

  bootWakeCause = esp_sleep_get_wakeup_cause();
  configurationWake = bootWakeCause != ESP_SLEEP_WAKEUP_TIMER;

  analogReadResolution(12);
  analogSetPinAttenuation(BAT_PIN, ADC_11db);
  readSettings();
  runtimeScreenEnabled = configurationWake || settings.screenEnabled;
  runtimeBleEnabled = configurationWake || settings.bleEnabled;
  if (runtimeScreenEnabled) {
    initDisplay();
  } else {
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, LOW);
  }
  initSd();
  sampleBattery(false);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  gpsSerial.write(0xFF);  // UART edge wakes an M10 receiver from software backup.
  gpsSerial.flush();
  if (runtimeBleEnabled) initBle();

  serviceLog("BOOT", "OK", "c6_tracker_ble");
  updateDisplayValues();
  Serial.println("C6 Tracker ready: LVGL + GPS + SD + BLE");
  printStatus();
}

void loop() {
  const uint32_t now = millis();
  pollSerial();
  processBleQueue();
  serviceHistoryStream(now);
  serviceTrackTransfer(now);
  updateGps();
  updateInitialFix();
  serviceSleepCycle();

  if (now - lastBatterySampleMs >= BATTERY_SAMPLE_MS) {
    lastBatterySampleMs = now;
    sampleBattery(false);
  }

  if ((!sdOk || sdWriteError) && now - lastSdRetryMs >= 10000) {
    lastSdRetryMs = now;
    if (!sdOk) initSd();
    else sdWriteError = !validateSdWrite();
    refreshSdHealth();
  }

  if (now - lastServiceLogMs >= SERVICE_LOG_MS) {
    lastServiceLogMs = now;
    serviceLog("STATUS", recording ? "RECORDING" : "STOPPED", routePath);
  }

  if (now - lastBleUpdateMs >= BLE_UPDATE_MS) {
    lastBleUpdateMs = now;
    publishTelemetry(true);
    publishCycleStatus(true);
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdateMs = now;
    updateDisplayValues();
  }

  if (displayReady) lv_timer_handler();
  delay(2);
}
