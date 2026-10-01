#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>

#include <Adafruit_HDC1000.h>
#include <DFRobot_ENS160.h>
#include <GxEPD2_BW.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <Update.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WebServer.h>
#include <PubSubClient.h>

#include "config.h"

DFRobot_ENS160_I2C ens160(&Wire, SENSOR_I2C_ADDRESS);
Adafruit_HDC1000 hdc1008;
Adafruit_NeoPixel aqiPixels(WS2812_LED_COUNT, WS2812_DATA_PIN,
                            NEO_RGB + NEO_KHZ800);
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(
  GxEPD2_270(EPD_CS_PIN, EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN));
WebServer webServer(80);
WiFiClient mqttTransport;
PubSubClient mqttClient(mqttTransport);
U8G2_FOR_ADAFRUIT_GFX textRenderer;

const uint8_t* const FONT_SMALL = u8g2_font_helvB08_tf;
const uint8_t* const FONT_MEDIUM = u8g2_font_helvB10_tf;
const uint8_t* const FONT_RATING = u8g2_font_helvB14_tf;
const uint8_t* const FONT_LARGE = u8g2_font_helvB18_tf;

struct SensorReading {
  bool ens160Available;
  bool climateAvailable;
  uint8_t status;
  uint8_t aqi;
  uint16_t tvoc;
  uint16_t eco2;
  float temperature;
  float humidity;
};

bool ens160Ready = false;
bool hdc1008Ready = false;
uint32_t lastRefreshMillis = 0;
uint32_t lastFullRefreshMillis = 0;
uint8_t displayedAqi = 0;
bool displayedAqiAvailable = false;
bool aqiAlertVisible = true;
uint32_t lastAqiBlinkMillis = 0;
uint32_t dashboardRefreshIntervalMs = 5000;
uint8_t aqiLedBrightness = WS2812_BRIGHTNESS;
String configuredWifiSsid;
String configuredWifiPassword;
bool wifiReconnectPending = false;
bool otaUploadSucceeded = false;
bool mqttReconnectPending = false;
bool mqttEnabled = false;
String mqttHost;
String mqttUsername;
String mqttPassword;
String mqttBaseTopic;
uint16_t mqttPort = MQTT_DEFAULT_PORT;
float fallbackTemperatureC = ENS160_FALLBACK_TEMPERATURE_C;
float fallbackHumidityPercent = ENS160_FALLBACK_HUMIDITY_PERCENT;
float temperatureOffsetC = 0.0f;   // Kompensation, wird zur Messung addiert
uint32_t lastMqttConnectAttemptMillis = 0;
bool simulateSensorData = SIMULATE_SENSOR_DATA != 0;
bool simulateAqiTest = SIMULATE_AQI_TEST != 0;
bool simulateValuesAutomatically = true;
float simulatedTemperatureC = 22.0f;
float simulatedHumidityPercent = 45.0f;
uint8_t simulatedAqi = 3;
uint16_t simulatedTvoc = 120;
uint16_t simulatedEco2 = 540;
SensorReading latestReading = {};
SensorReading displayedReading = {};   // was zuletzt auf dem E-Paper steht
struct tm displayedTime = {};
bool displayedTimeValid = false;
const uint32_t LED_UPDATE_INTERVAL_MS = 500;
uint32_t lastLedUpdateMillis = 0;
const uint32_t AQI_BLINK_INTERVAL_MS = 500;
const uint32_t GAS_VALUE_UPDATE_INTERVAL_MS = 1000;
const int16_t AQI_BAR_FIRST_BOX_X = 148;
const int16_t AQI_BAR_Y = 89;
const int16_t AQI_BOX_WIDTH = 17;
const int16_t AQI_BOX_HEIGHT = 11;
const int16_t AQI_BOX_GAP = 4;
uint32_t lastGasValueUpdateMillis = 0;
uint16_t cachedTvoc = 0;
uint16_t cachedEco2 = 0;
bool gasValuesInitialized = false;

const uint8_t MEDIAN_MAX_WINDOW = 15;
bool displayPortrait = false;          // true = Hochformat (90 Grad gedreht)
bool displayLayoutChanged = false;
bool sensorRestartPending = false;
const int16_t P_AQI_BAR_FIRST_BOX_X = 10;
const int16_t P_AQI_BAR_Y = 152;
const int16_t P_AQI_BOX_WIDTH = 28;
const int16_t P_AQI_BOX_HEIGHT = 12;
const int16_t P_AQI_BOX_GAP = 4;

uint8_t displayRotation() {
  return displayPortrait ? 0 : 1;
}

struct MedianFilter {
  uint16_t samples[MEDIAN_MAX_WINDOW];
  uint8_t count;
  uint8_t next;
};

MedianFilter tvocMedian = {};
MedianFilter eco2Median = {};
uint8_t tvocMedianWindow = 10;   // 1 = Filter aus
uint8_t eco2MedianWindow = 10;   // 1 = Filter aus

void resetMedian(MedianFilter& filter) {
  filter.count = 0;
  filter.next = 0;
}

uint16_t medianPush(MedianFilter& filter, uint16_t value, uint8_t window) {
  window = constrain(window, 1, MEDIAN_MAX_WINDOW);
  if (window == 1) {
    resetMedian(filter);
    return value;
  }
  if (filter.next >= window) filter.next = 0;
  filter.samples[filter.next] = value;
  filter.next = (filter.next + 1) % window;
  if (filter.count < window) ++filter.count;

  uint16_t sorted[MEDIAN_MAX_WINDOW];
  memcpy(sorted, filter.samples, filter.count * sizeof(uint16_t));
  for (uint8_t i = 1; i < filter.count; ++i) {        // Insertion Sort
    const uint16_t key = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = key;
  }
  if (filter.count % 2 == 1) return sorted[filter.count / 2];
  return static_cast<uint16_t>(
      (uint32_t(sorted[filter.count / 2 - 1]) + sorted[filter.count / 2]) / 2);
}

bool gasSampleDue() {
  return !gasValuesInitialized ||
         millis() - lastGasValueUpdateMillis >= GAS_VALUE_UPDATE_INTERVAL_MS;
}

void storeGasSample(uint16_t rawTvoc, uint16_t rawEco2) {
  cachedTvoc = medianPush(tvocMedian, rawTvoc, tvocMedianWindow);
  cachedEco2 = medianPush(eco2Median, rawEco2, eco2MedianWindow);
  lastGasValueUpdateMillis = millis();
  gasValuesInitialized = true;
}

uint8_t simulatedAqiTestValue() {
  return static_cast<uint8_t>((millis() / 10000UL) % 5 + 1);
}

const char* aqiLabel(uint8_t aqi) {
  switch (aqi) {
    case 1: return "Ausgezeichnet";
    case 2: return "Gut";
    case 3: return "M\xC3\xA4" "\xC3\x9F" "ig";
    case 4: return "Schlecht";
    case 5: return "Ungesund";
    default: return "Keine Daten";
  }
}

const uint16_t ECO2_GREEN_PPM = 600;    // eCO2: Grün bis hier
const uint16_t ECO2_RED_PPM   = 1200;   // eCO2: Rot ab hier (Gelb bei 1000)
const uint16_t TVOC_GREEN_PPB = 100;    // TVOC: Grün bis hier
const uint16_t TVOC_RED_PPB   = 660;    // TVOC: Rot ab hier (Gelb bei ~380)

float levelFromRange(uint16_t value, uint16_t green, uint16_t red) {
  const uint16_t v = constrain(value, green, red);
  return float(v - green) / float(red - green);   // 0 = grün, 1 = rot
}

void updateAqiPixels(const SensorReading& reading) {
  uint32_t color = 0;
  if (reading.ens160Available) {
    const float eco2Level = levelFromRange(reading.eco2, ECO2_GREEN_PPM, ECO2_RED_PPM);
    const float tvocLevel = levelFromRange(reading.tvoc, TVOC_GREEN_PPB, TVOC_RED_PPB);
    const float t = max(eco2Level, tvocLevel);     // schlechterer Wert gewinnt

    uint8_t r, g;
    if (t < 0.5f) { r = uint8_t(255 * t * 2.0f); g = 255; }
    else          { r = 255; g = uint8_t(255 * (1.0f - (t - 0.5f) * 2.0f)); }
    color = aqiPixels.Color(r, g, 0);
  }

  aqiPixels.fill(color);
  aqiPixels.show();
}

void drawText(const char* text, int16_t x, int16_t baseline,
              const uint8_t* font, uint16_t color) {
  textRenderer.setFont(font);
  textRenderer.setFontMode(1);
  textRenderer.setForegroundColor(color);
  textRenderer.drawUTF8(x, baseline, text);
}

void drawCenteredText(const char* text, int16_t centerX, int16_t baseline,
                      const uint8_t* font, uint16_t color) {
  textRenderer.setFont(font);
  const int16_t width = textRenderer.getUTF8Width(text);
  drawText(text, centerX - width / 2, baseline, font, color);
}

void drawCenteredValueWithUnit(const char* value, const char* unit,
                               int16_t centerX, int16_t baseline) {
  textRenderer.setFont(FONT_LARGE);
  const int16_t valueWidth = textRenderer.getUTF8Width(value);
  textRenderer.setFont(FONT_MEDIUM);
  const int16_t unitWidth = textRenderer.getUTF8Width(unit);
  const int16_t gap = 5;
  const int16_t startX = centerX - (valueWidth + gap + unitWidth) / 2;
  drawText(value, startX, baseline, FONT_LARGE, GxEPD_BLACK);
  drawText(unit, startX + valueWidth + gap, baseline, FONT_MEDIUM, GxEPD_BLACK);
}

void drawAqiAlertBoxes(uint8_t aqi, bool aqiAvailable, bool alertVisible) {
  if (displayPortrait) {
    display.setPartialWindow(104, 150, 64, 16);
    display.firstPage();
    do {
      for (uint8_t index = 3; index < 5; ++index) {
        const int16_t boxX = P_AQI_BAR_FIRST_BOX_X +
                             index * (P_AQI_BOX_WIDTH + P_AQI_BOX_GAP);
        const bool active = aqiAvailable && aqi >= index + 1;
        display.fillRect(boxX, P_AQI_BAR_Y, P_AQI_BOX_WIDTH, P_AQI_BOX_HEIGHT,
                         GxEPD_WHITE);
        if (active && alertVisible) {
          display.fillRect(boxX, P_AQI_BAR_Y, P_AQI_BOX_WIDTH, P_AQI_BOX_HEIGHT,
                           GxEPD_BLACK);
        } else {
          display.drawRect(boxX, P_AQI_BAR_Y, P_AQI_BOX_WIDTH, P_AQI_BOX_HEIGHT,
                           GxEPD_BLACK);
        }
      }
    } while (display.nextPage());
    return;
  }
  display.setPartialWindow(208, 88, 48, 16);
  display.firstPage();
  do {
    for (uint8_t index = 3; index < 5; ++index) {
      const int16_t boxX = AQI_BAR_FIRST_BOX_X +
                           index * (AQI_BOX_WIDTH + AQI_BOX_GAP);
      const bool active = aqiAvailable && aqi >= index + 1;
      if (active && alertVisible) {
        display.fillRect(boxX, AQI_BAR_Y, AQI_BOX_WIDTH, AQI_BOX_HEIGHT,
                         GxEPD_BLACK);
      } else {
        display.drawRect(boxX, AQI_BAR_Y, AQI_BOX_WIDTH, AQI_BOX_HEIGHT,
                         GxEPD_BLACK);
      }
    }
  } while (display.nextPage());
}

uint8_t wifiStrengthArcs() {
  if (WiFi.status() != WL_CONNECTED) return 3;
  const int32_t rssi = WiFi.RSSI();
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  return 1;
}

// Windows-Stil: 90°-Fächer aus dicken Bögen über einem Punkt.
// Aktive Bögen voll, inaktive gerastert (wirkt auf E-Paper grau).
const int16_t WIFI_ICON_WIDTH = 25;

void drawWifiStatusIcon(int16_t x, int16_t y, bool connected) {
  const int16_t centerX = x + 12;
  const int16_t centerY = y + 20;
  const uint8_t arcs = connected ? wifiStrengthArcs() : 0;
  const float bands[3][2] = {{4.5f, 7.5f}, {9.5f, 12.5f}, {14.5f, 17.5f}};

  for (int16_t dy = -18; dy <= 0; ++dy) {
    for (int16_t dx = -13; dx <= 13; ++dx) {
      if (abs(dx) > -dy) continue;                    // nur 90°-Fächer nach oben
      const float r = sqrtf(float(dx * dx + dy * dy));
      for (uint8_t band = 0; band < 3; ++band) {
        if (r < bands[band][0] || r > bands[band][1]) continue;
        const bool active = band < arcs;
        if (active || ((dx + dy) & 1) == 0) {
          display.drawPixel(centerX + dx, centerY + dy, GxEPD_WHITE);
        }
      }
    }
  }
  display.fillCircle(centerX, centerY, 2, GxEPD_WHITE);

  if (!connected) {                                   // kleines X unten rechts
    const int16_t crossX = x + 18;
    const int16_t crossY = y + 15;
    display.drawLine(crossX, crossY, crossX + 5, crossY + 5, GxEPD_WHITE);
    display.drawLine(crossX + 1, crossY, crossX + 6, crossY + 5, GxEPD_WHITE);
    display.drawLine(crossX + 5, crossY, crossX, crossY + 5, GxEPD_WHITE);
    display.drawLine(crossX + 6, crossY, crossX + 1, crossY + 5, GxEPD_WHITE);
  }
}

bool initializeENS160() {
  if (ens160.begin() != NO_ERR) {
    Serial.println("SEN0515 not found; will retry on the next refresh.");
    return false;
  }

  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(fallbackTemperatureC, fallbackHumidityPercent);
  Serial.println("SEN0515 ready.");
  return true;
}

bool initializeHDC1008();

// ENS160 per Opmode 0xF0 zuruecksetzen, dann beide Sensoren neu initialisieren.
void restartSensors() {
  Serial.println("Restarting sensors...");
  if (ens160Ready) {
    ens160.setPWRMode(0xF0);   // ENS160 Reset
    delay(10);
  }
  ens160Ready = false;
  hdc1008Ready = false;
  resetMedian(tvocMedian);
  resetMedian(eco2Median);
  gasValuesInitialized = false;
  if (!simulateSensorData) {
    hdc1008Ready = initializeHDC1008();
    ens160Ready = initializeENS160();
  }
}

bool initializeHDC1008() {
  if (!hdc1008.begin(HDC1008_I2C_ADDRESS, &Wire)) {
    Serial.println("HDC1008 not found; will retry on the next refresh.");
    return false;
  }

  Serial.println("HDC1008 ready.");
  return true;
}

SensorReading readSensors() {
  SensorReading reading = {false, false, 3, 0, 0, 0, 0.0f, 0.0f};

  if (simulateSensorData) {
    const float phase = millis() / 10000.0f;
    const float gasPhase =
        static_cast<float>(millis() / GAS_VALUE_UPDATE_INTERVAL_MS);
    reading.climateAvailable = true;
    reading.ens160Available = true;
    reading.status = 0;
    if (simulateAqiTest) {
      reading.aqi = simulatedAqiTestValue();
    } else if (simulateValuesAutomatically) {
      reading.aqi = 1 + (static_cast<uint32_t>(phase / 5.0f) % 5);
    } else {
      reading.aqi = simulatedAqi;
    }
    reading.temperature = simulateValuesAutomatically
        ? 22.0f + 1.5f * sinf(phase / 8.0f) : simulatedTemperatureC;
    reading.humidity = simulateValuesAutomatically
        ? 45.0f + 8.0f * sinf(phase / 10.0f) : simulatedHumidityPercent;
    if (gasSampleDue()) {
      const uint16_t rawTvoc = simulateValuesAutomatically
          ? static_cast<uint16_t>(180.0f + 90.0f * (1.0f + sinf(gasPhase / 4.0f)))
          : simulatedTvoc;
      const uint16_t rawEco2 = simulateValuesAutomatically
          ? static_cast<uint16_t>(500.0f + 600.0f * (1.0f + sinf(gasPhase / 6.0f)))
          : simulatedEco2;
      storeGasSample(rawTvoc, rawEco2);
    }
    reading.tvoc = cachedTvoc;
    reading.eco2 = cachedEco2;
    return reading;
  }

  if (!hdc1008Ready) {
    hdc1008Ready = initializeHDC1008();
  }

  if (hdc1008Ready) {
    const float temperature = hdc1008.readTemperature();
    const float humidity = hdc1008.readHumidity();
    if (isfinite(temperature) && isfinite(humidity)) {
      reading.climateAvailable = true;
      reading.temperature = temperature + temperatureOffsetC;
      reading.humidity = humidity;
    }
  }

  if (!ens160Ready) {
    ens160Ready = initializeENS160();
  }

  if (!ens160Ready) {
    if (simulateAqiTest) {
      reading.ens160Available = true;
      reading.status = 0;
      reading.aqi = simulatedAqiTestValue();
    }
    return reading;
  }

  if (reading.climateAvailable) {
    ens160.setTempAndHum(reading.temperature, reading.humidity);
  }

  reading.ens160Available = true;
  reading.status = ens160.getENS160Status();
  reading.aqi = ens160.getAQI();
  if (gasSampleDue()) {
    storeGasSample(ens160.getTVOC(), ens160.getECO2());
  }
  reading.tvoc = cachedTvoc;
  reading.eco2 = cachedEco2;
  if (simulateAqiTest) {
    reading.status = 0;
    reading.aqi = simulatedAqiTestValue();
  }
  return reading;
}

bool readClock(struct tm& currentTime) {
  return getLocalTime(&currentTime, 20);
}

void logStatus(const SensorReading& reading, const struct tm* currentTime) {
  char timeText[24] = "not synchronized";
  if (currentTime != nullptr) {
    strftime(timeText, sizeof(timeText), "%Y-%m-%d %H:%M:%S", currentTime);
  }

  const char* const sensorSource = simulateSensorData ? "simulated" : "hardware";

  Serial.printf("Clock: %s | Sensors (%s): ", timeText, sensorSource);
  if (reading.climateAvailable) {
    Serial.printf("temperature=%.1f C, humidity=%.0f%%", reading.temperature,
                  reading.humidity);
  } else {
    Serial.print("temperature/humidity unavailable");
  }

  if (reading.ens160Available) {
    Serial.printf(", AQI=%u (%s), TVOC=%u ppb, eCO2=%u ppm\n", reading.aqi,
                  aqiLabel(reading.aqi), reading.tvoc, reading.eco2);
  } else {
    Serial.println(", air-quality sensor unavailable");
  }
}

const char* ratingText(const SensorReading& reading) {
  if (!reading.ens160Available) return "Kein Sensor";
  if (reading.status == 1) return "Aufw\xC3\xA4" "rmen...";
  if (reading.status == 2) return "Anlaufphase";
  if (reading.status == 0) return aqiLabel(reading.aqi);
  return "Messfehler";
}

void drawRightAlignedText(const char* text, int16_t rightX, int16_t baseline,
                          const uint8_t* font) {
  textRenderer.setFont(font);
  drawText(text, rightX - textRenderer.getUTF8Width(text), baseline, font,
           GxEPD_BLACK);
}

// Hochformat 176 x 264
void drawPortraitPage(const SensorReading& reading, const char* dateText,
                      const char* timeText, const char* ipText,
                      const char* temperatureText, const char* humidityText,
                      const char* tvocText, const char* eco2Text) {
  const int16_t width = display.width();

  display.fillRect(0, 0, width, 46, GxEPD_BLACK);
  drawText(dateText, 6, 17, FONT_MEDIUM, GxEPD_WHITE);
  drawText(timeText, 6, 40, FONT_LARGE, GxEPD_WHITE);
  drawWifiStatusIcon(width - WIFI_ICON_WIDTH - 3, 18,
                     WiFi.status() == WL_CONNECTED);

  drawCenteredText("Temperatur", 44, 60, FONT_SMALL, GxEPD_BLACK);
  drawCenteredText("Feuchte", 132, 60, FONT_SMALL, GxEPD_BLACK);
  drawCenteredValueWithUnit(temperatureText, "\xC2\xB0" "C", 44, 87);
  drawCenteredValueWithUnit(humidityText, "%", 132, 87);
  display.drawFastVLine(88, 52, 42, GxEPD_BLACK);
  display.drawFastHLine(0, 98, width, GxEPD_BLACK);
  display.drawFastHLine(0, 99, width, GxEPD_BLACK);

  drawCenteredText("Luftqualit\xC3\xA4" "t (AQI)", 88, 112, FONT_SMALL,
                   GxEPD_BLACK);
  char aqiText[4] = "--";
  if (reading.ens160Available) {
    snprintf(aqiText, sizeof(aqiText), "%u", reading.aqi);
  }
  drawCenteredText(aqiText, 34, 142, FONT_LARGE, GxEPD_BLACK);
  const char* rating = ratingText(reading);
  textRenderer.setFont(FONT_RATING);
  const uint8_t* ratingFont =
      textRenderer.getUTF8Width(rating) <= 106 ? FONT_RATING : FONT_MEDIUM;
  drawCenteredText(rating, 116, 140, ratingFont, GxEPD_BLACK);

  for (uint8_t index = 0; index < 5; ++index) {
    const int16_t boxX = P_AQI_BAR_FIRST_BOX_X +
                         index * (P_AQI_BOX_WIDTH + P_AQI_BOX_GAP);
    const bool active = reading.ens160Available && reading.aqi >= index + 1;
    const bool visible = index < 3 || !active || aqiAlertVisible;
    if (active && visible) {
      display.fillRect(boxX, P_AQI_BAR_Y, P_AQI_BOX_WIDTH, P_AQI_BOX_HEIGHT,
                       GxEPD_BLACK);
    } else {
      display.drawRect(boxX, P_AQI_BAR_Y, P_AQI_BOX_WIDTH, P_AQI_BOX_HEIGHT,
                       GxEPD_BLACK);
    }
  }
  display.drawFastHLine(0, 172, width, GxEPD_BLACK);
  display.drawFastHLine(0, 173, width, GxEPD_BLACK);

  drawText("TVOC", 10, 195, FONT_MEDIUM, GxEPD_BLACK);
  drawText("ppb", 10, 207, FONT_SMALL, GxEPD_BLACK);
  drawRightAlignedText(tvocText, width - 10, 205, FONT_LARGE);
  display.drawFastHLine(10, 214, width - 20, GxEPD_BLACK);
  drawText("eCO2", 10, 233, FONT_MEDIUM, GxEPD_BLACK);
  drawText("ppm", 10, 245, FONT_SMALL, GxEPD_BLACK);
  drawRightAlignedText(eco2Text, width - 10, 243, FONT_LARGE);
  drawText(ipText, 2, 261, u8g2_font_4x6_tf, GxEPD_BLACK);
}

void drawDashboard(const SensorReading& reading, const struct tm* currentTime,
                   bool fullRefresh) {
  displayedReading = reading;
  displayedTimeValid = currentTime != nullptr;
  if (displayedTimeValid) displayedTime = *currentTime;
  static const char* const weekdays[] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};
  char dateText[24] = "--.--.----";
  char timeText[9] = "--:--:--";
  char ipText[24] = "IP: --";
  char temperatureText[12] = "--.-";
  char humidityText[8] = "--";
  char tvocText[8] = "----";
  char eco2Text[8] = "----";

  if (currentTime != nullptr) {
    snprintf(dateText, sizeof(dateText), "%s %02d.%02d.%04d",
             weekdays[currentTime->tm_wday], currentTime->tm_mday,
             currentTime->tm_mon + 1, currentTime->tm_year + 1900);
    strftime(timeText, sizeof(timeText), "%H:%M:%S", currentTime);
  }
  if (WiFi.status() == WL_CONNECTED) {
    const String ipAddress = WiFi.localIP().toString();
    snprintf(ipText, sizeof(ipText), "IP:%s", ipAddress.c_str());
  }

  if (reading.climateAvailable) {
    snprintf(temperatureText, sizeof(temperatureText), "%.1f",
             reading.temperature);
    snprintf(humidityText, sizeof(humidityText), "%.0f", reading.humidity);
  }
  if (reading.ens160Available) {
    snprintf(tvocText, sizeof(tvocText), "%u", reading.tvoc);
    snprintf(eco2Text, sizeof(eco2Text), "%u", reading.eco2);
  }

  if (fullRefresh) {
    display.setFullWindow();
  } else {
    display.setPartialWindow(0, 0, display.width(), display.height());
  }

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    if (displayPortrait) {
      drawPortraitPage(reading, dateText, timeText, ipText, temperatureText,
                       humidityText, tvocText, eco2Text);
      continue;   // springt zu display.nextPage()
    }
    display.fillRect(0, 0, display.width(), 29, GxEPD_BLACK);

    textRenderer.setFont(FONT_RATING);
    const int16_t dateWidth = textRenderer.getUTF8Width(dateText);
    const int16_t timeWidth = textRenderer.getUTF8Width(timeText);
    const int16_t timeX = display.width() - 7 - timeWidth;
    const int16_t dateEnd = 6 + dateWidth;
    const int16_t iconWidth = WIFI_ICON_WIDTH;
    const int16_t availableGap = timeX - dateEnd;
    drawText(dateText, 6, 22, FONT_RATING, GxEPD_WHITE);
    if (availableGap >= iconWidth) {
      const int16_t iconX = dateEnd + (availableGap - iconWidth) / 2;
      drawWifiStatusIcon(iconX, 3, WiFi.status() == WL_CONNECTED);
    }
    drawText(timeText, timeX, 22, FONT_RATING, GxEPD_WHITE);

    drawCenteredText("Temperatur", 66, 43, FONT_SMALL, GxEPD_BLACK);
    drawCenteredText("Feuchte", 198, 43, FONT_SMALL, GxEPD_BLACK);
    drawCenteredValueWithUnit(temperatureText, "\xC2\xB0" "C", 66, 70);
    drawCenteredValueWithUnit(humidityText, "%", 198, 70);
    display.drawFastVLine(132, 35, 41, GxEPD_BLACK);
    display.drawFastHLine(0, 80, display.width(), GxEPD_BLACK);
    display.drawFastHLine(0, 81, display.width(), GxEPD_BLACK);

    drawCenteredText("Luftqualit\xC3\xA4" "t (AQI)", 66, 93, FONT_SMALL,
                     GxEPD_BLACK);
    char aqiText[4] = "--";
    if (reading.ens160Available) {
      snprintf(aqiText, sizeof(aqiText), "%u", reading.aqi);
    }

    const int16_t boxWidth = 17;
    const int16_t boxHeight = 11;
    const int16_t boxGap = 4;
    const int16_t boxesWidth = 5 * boxWidth + 4 * boxGap;
    const int16_t firstBoxX = 198 - boxesWidth / 2;
    for (uint8_t index = 0; index < 5; ++index) {
      const int16_t boxX = AQI_BAR_FIRST_BOX_X +
                           index * (AQI_BOX_WIDTH + AQI_BOX_GAP);
      const bool active = reading.ens160Available && reading.aqi >= index + 1;
      const bool visible = index < 3 || !active || aqiAlertVisible;
      if (active && visible) {
        display.fillRect(boxX, AQI_BAR_Y, AQI_BOX_WIDTH, AQI_BOX_HEIGHT,
                         GxEPD_BLACK);
      } else {
        display.drawRect(boxX, AQI_BAR_Y, AQI_BOX_WIDTH, AQI_BOX_HEIGHT,
                         GxEPD_BLACK);
      }
    }
    drawCenteredText(aqiText, 66, 121, FONT_LARGE, GxEPD_BLACK);

    const char* rating = "Kein Sensor";
    if (reading.ens160Available) {
      if (reading.status == 1 ) {
        rating = "Aufw\xC3\xA4" "rmen...";
      } else if (reading.status == 2) {
        rating ="Anlaufphase";
      } else if (reading.status == 0) {
        rating = aqiLabel(reading.aqi);
      } else {
        rating = "Messfehler";
      }
    }
    textRenderer.setFont(FONT_RATING);
    const uint8_t* ratingFont =
        textRenderer.getUTF8Width(rating) <= 124 ? FONT_RATING : FONT_MEDIUM;
    drawCenteredText(rating, 198, 120, ratingFont, GxEPD_BLACK);

    display.drawFastHLine(0, 129, display.width(), GxEPD_BLACK);
    display.drawFastHLine(0, 130, display.width(), GxEPD_BLACK);
    display.drawFastVLine(132, 134, 42, GxEPD_BLACK);
    drawCenteredText("TVOC (ppb)", 66, 142, FONT_SMALL, GxEPD_BLACK);
    drawCenteredText("eCO2 (ppm)", 198, 142, FONT_SMALL, GxEPD_BLACK);
    drawCenteredText(tvocText, 66, 168, FONT_LARGE, GxEPD_BLACK);
    drawCenteredText(eco2Text, 198, 168, FONT_LARGE, GxEPD_BLACK);
    drawText(ipText, 2, 175, u8g2_font_4x6_tf, GxEPD_BLACK);
  } while (display.nextPage());
}

String escapeHtml(const String& value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  for (size_t index = 0; index < value.length(); ++index) {
    switch (value[index]) {
      case '&': escaped += F("&amp;"); break;
      case '<': escaped += F("&lt;"); break;
      case '>': escaped += F("&gt;"); break;
      case '"': escaped += F("&quot;"); break;
      case '\'': escaped += F("&#39;"); break;
      default: escaped += value[index]; break;
    }
  }
  return escaped;
}

void appendNavLink(String& page, const char* href, const char* label,
                   const String& activeTitle) {
  page += F("<a href='");
  page += href;
  page += F("'");
  if (activeTitle == label) page += F(" class='active'");
  page += F(">");
  page += label;
  page += F("</a>");
}

const char PAGE_CSS[] PROGMEM = R"CSS(
:root{--bg:#f1f5f9;--card:#fff;--text:#1e293b;--muted:#64748b;--primary:#2563eb;--primary-dark:#1d4ed8;--border:#e2e8f0;--ok:#16a34a;--warn:#d97706;--err:#dc2626}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font-family:-apple-system,'Segoe UI',Roboto,sans-serif;font-size:15px}
.topbar{background:#111827;color:#fff;padding:12px 16px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.topbar .brand{font-weight:700;font-size:16px;margin-right:auto;display:flex;align-items:center;gap:8px;white-space:nowrap}
.topbar nav{display:flex;gap:2px;flex-wrap:wrap}
.topbar nav a{color:#cbd5e1;text-decoration:none;padding:7px 11px;border-radius:6px;font-size:12.5px;white-space:nowrap}
.topbar nav a:hover{background:#1f2937;color:#fff}
.topbar nav a.active{background:var(--primary);color:#fff}
.wrap{max-width:760px;margin:20px auto;padding:0 16px 40px}
.card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:20px;margin-bottom:16px;box-shadow:0 1px 2px rgba(0,0,0,.04)}
.card h2{margin:0 0 14px;font-size:16px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.subcard{border:1px solid var(--border);border-radius:8px;padding:14px;margin:14px 0;background:#fafbfc}
.subcard h3{margin:0 0 10px;font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.04em}
.row{margin-bottom:16px}
.row .rowhead{display:flex;justify-content:space-between;align-items:baseline}
.row .rowhead .val{font-weight:600;color:var(--primary);font-size:13px}
label{display:block;font-size:12.5px;color:var(--muted);margin-bottom:4px}
input[type=text],input[type=password],input[type=number],select{width:100%;padding:9px 10px;border:1px solid var(--border);border-radius:6px;font-size:14px;background:#fff;color:var(--text);font-family:inherit}
input:focus,select:focus{outline:2px solid var(--primary);outline-offset:-1px}
.hint{font-size:11.5px;color:var(--muted);margin-top:3px;font-weight:400}
.btn{background:var(--primary);color:#fff;border:0;padding:10px 18px;border-radius:6px;font-size:14px;cursor:pointer;font-weight:600}
.btn:hover{background:var(--primary-dark)}
.btn.secondary{background:#fff;color:var(--text);border:1px solid var(--border)}
.btn.small{padding:7px 12px;font-size:12.5px}
.pill{display:inline-block;padding:3px 10px;border-radius:999px;font-size:12px;font-weight:600}
.pill.ok{background:#dcfce7;color:var(--ok)}
.pill.warn{background:#fef3c7;color:var(--warn)}
.pill.err{background:#fee2e2;color:var(--err)}
.statgrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media (max-width:420px){.statgrid{grid-template-columns:1fr}}
.stat{background:#f8fafc;border:1px solid var(--border);border-radius:8px;padding:10px 12px}
.stat .k{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.03em}
.stat .v{font-size:15px;font-weight:600;margin-top:2px}
.switchrow{display:flex;align-items:center;justify-content:space-between;gap:10px}
.switch{position:relative;display:inline-block;width:42px;height:24px;flex:none}
.switch input{opacity:0;width:0;height:0}
.slider2{position:absolute;cursor:pointer;inset:0;background:#cbd5e1;border-radius:24px;transition:.15s}
.slider2:before{content:'';position:absolute;height:18px;width:18px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.15s}
input:checked+.slider2{background:var(--primary)}
input:checked+.slider2:before{transform:translateX(18px)}
.actionbar{display:flex;gap:8px;flex-wrap:wrap;margin-top:4px}
input[type=range]{-webkit-appearance:none;width:100%;height:6px;border-radius:3px;background:var(--border);outline:none;margin:4px 0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;border-radius:50%;background:var(--primary);cursor:pointer;border:2px solid #fff;box-shadow:0 1px 3px rgba(0,0,0,.3)}
input[type=range]::-moz-range-thumb{width:18px;height:18px;border-radius:50%;background:var(--primary);cursor:pointer;border:2px solid #fff;box-shadow:0 1px 3px rgba(0,0,0,.3)}
.wrap:has(.statuslayout){max-width:1000px}
.statuslayout{display:flex;gap:20px;align-items:flex-start;flex-wrap:wrap}
.statuscol{flex:1 1 300px;min-width:0}
.screencol{flex:0 0 auto;max-width:100%}
.screencol img{display:block;max-width:100%;border:8px solid #222;border-radius:8px;background:#fff}
.swatch{width:28px;height:28px;border-radius:50%;border:2px solid var(--border);flex:none}
)CSS";

String renderPage(const String& title, const String& content) {
  String page;
  page.reserve(5000);
  page += F("<!doctype html><html lang='de'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>");
  page += title;
  page += F(" | AIRQ Sensor</title><style>");
  page += FPSTR(PAGE_CSS);
  page += F("</style></head><body><div class='topbar'><div class='brand'>AIRQ Sensor</div><nav>");
  appendNavLink(page, "/", "Status", title);
  appendNavLink(page, "/wifi", "WLAN", title);
  appendNavLink(page, "/mqtt", "MQTT", title);
  appendNavLink(page, "/params", "Parameter", title);
  appendNavLink(page, "/firmware", "Firmware", title);
  page += F("</nav></div><div class='wrap'><div class='card'>");
  page += content;
  page += F("</div></div></body></html>");
  return page;
}

void sendPage(const String& title, const String& content, int statusCode = 200) {
  webServer.send(statusCode, "text/html; charset=utf-8",
                 renderPage(title, content));
}

bool requireWebAdmin() {
  if (webServer.authenticate(WEB_ADMIN_USERNAME, WEB_ADMIN_PASSWORD)) {
    return true;
  }
  webServer.requestAuthentication(BASIC_AUTH, "AIRQ Sensor Settings");
  return false;
}

void loadPersistentSettings() {
  Preferences preferences;
  preferences.begin("airq", true);
  configuredWifiSsid = preferences.getString("ssid", WIFI_SSID);
  configuredWifiPassword = preferences.getString("password", WIFI_PASSWORD);
  dashboardRefreshIntervalMs =
      constrain(preferences.getUInt("refresh", 1), 1UL, 3600UL) * 1000UL;
  aqiLedBrightness = preferences.getUChar("brightness", WS2812_BRIGHTNESS);
  fallbackTemperatureC = preferences.getFloat(
      "fallbackTemp", ENS160_FALLBACK_TEMPERATURE_C);
  fallbackHumidityPercent = preferences.getFloat(
      "fallbackHum", ENS160_FALLBACK_HUMIDITY_PERCENT);
  mqttHost = preferences.getString("mqttHost", MQTT_DEFAULT_HOST);
  mqttPort = preferences.getUShort("mqttPort", MQTT_DEFAULT_PORT);
  mqttUsername = preferences.getString("mqttUser", "");
  mqttPassword = preferences.getString("mqttPass", "");
  mqttBaseTopic = preferences.getString("mqttTopic", MQTT_DEFAULT_BASE_TOPIC);
  mqttEnabled = preferences.getBool("mqttEnabled", false);
  simulateSensorData = preferences.getBool("simSensor", SIMULATE_SENSOR_DATA != 0);
  simulateAqiTest = preferences.getBool("simAqi", SIMULATE_AQI_TEST != 0);
  simulateValuesAutomatically = preferences.getBool("simAuto", true);
  simulatedTemperatureC = preferences.getFloat("simTemp", 22.0f);
  simulatedHumidityPercent = preferences.getFloat("simHum", 45.0f);
  simulatedAqi = preferences.getUChar("simAqiValue", 3);
  simulatedTvoc = preferences.getUShort("simTvoc", 120);
  simulatedEco2 = preferences.getUShort("simEco2", 540);
  tvocMedianWindow = constrain(preferences.getUChar("tvocMedian", 10), 1, MEDIAN_MAX_WINDOW);
  eco2MedianWindow = constrain(preferences.getUChar("eco2Median", 10), 1, MEDIAN_MAX_WINDOW);
  temperatureOffsetC = constrain(preferences.getFloat("tempOffset", 0.0f), -10.0f, 10.0f);
  displayPortrait = preferences.getBool("portrait", false);
  preferences.end();
}

void savePersistentSettings() {
  Preferences preferences;
  preferences.begin("airq", false);
  preferences.putString("ssid", configuredWifiSsid);
  preferences.putString("password", configuredWifiPassword);
  preferences.putUInt("refresh", dashboardRefreshIntervalMs / 1000UL);
  preferences.putUChar("brightness", aqiLedBrightness);
  preferences.putFloat("fallbackTemp", fallbackTemperatureC);
  preferences.putFloat("fallbackHum", fallbackHumidityPercent);
  preferences.putString("mqttHost", mqttHost);
  preferences.putUShort("mqttPort", mqttPort);
  preferences.putString("mqttUser", mqttUsername);
  preferences.putString("mqttPass", mqttPassword);
  preferences.putString("mqttTopic", mqttBaseTopic);
  preferences.putBool("mqttEnabled", mqttEnabled);
  preferences.putBool("simSensor", simulateSensorData);
  preferences.putBool("simAqi", simulateAqiTest);
  preferences.putBool("simAuto", simulateValuesAutomatically);
  preferences.putFloat("simTemp", simulatedTemperatureC);
  preferences.putFloat("simHum", simulatedHumidityPercent);
  preferences.putUChar("simAqiValue", simulatedAqi);
  preferences.putUShort("simTvoc", simulatedTvoc);
  preferences.putUShort("simEco2", simulatedEco2);
  preferences.putUChar("tvocMedian", tvocMedianWindow);
  preferences.putUChar("eco2Median", eco2MedianWindow);
  preferences.putFloat("tempOffset", temperatureOffsetC);
  preferences.putBool("portrait", displayPortrait);
  preferences.end();
}

void appendStatusStat(String& body, const char* label, const String& value,
                      bool error = false) {
  body += F("<div class='stat'><div class='k'>");
  body += label;
  body += F("</div><div class='v");
  if (error) body += F(" err-text");
  body += F("'>");
  body += escapeHtml(value);
  body += F("</div></div>");
}

// ---------- Web-Abbild des E-Paper-Displays (SVG) ----------

void svgText(String& svg, int16_t x, int16_t y, uint8_t size, const String& text,
             const char* anchor = "start", bool white = false,
             bool bold = true) {
  char head[160];
  snprintf(head, sizeof(head),
           "<text x='%d' y='%d' font-size='%u' text-anchor='%s' fill='%s'%s>",
           x, y, size, anchor, white ? "#fff" : "#000",
           bold ? " font-weight='bold'" : "");
  svg += head;
  svg += escapeHtml(text);
  svg += F("</text>");
}

void svgValueWithUnit(String& svg, int16_t centerX, int16_t y,
                      const String& value, const String& unit) {
  char head[120];
  snprintf(head, sizeof(head),
           "<text x='%d' y='%d' font-size='25' font-weight='bold' "
           "text-anchor='middle'>", centerX, y);
  svg += head;
  svg += escapeHtml(value);
  svg += F("<tspan font-size='14' dx='5'>");
  svg += escapeHtml(unit);
  svg += F("</tspan></text>");
}

void svgRect(String& svg, int16_t x, int16_t y, int16_t w, int16_t h,
             bool filled, const char* extraClass = "") {
  char buffer[160];
  if (filled) {
    snprintf(buffer, sizeof(buffer),
             "<rect x='%d' y='%d' width='%d' height='%d' fill='#000' class='%s'/>",
             x, y, w, h, extraClass);
  } else {
    snprintf(buffer, sizeof(buffer),
             "<rect x='%.1f' y='%.1f' width='%d' height='%d' fill='none' "
             "stroke='#000' stroke-width='1'/>",
             x + 0.5f, y + 0.5f, w - 1, h - 1);
  }
  svg += buffer;
}

void svgWifiIcon(String& svg, int16_t x, int16_t y, bool connected) {
  const float cx = x + 12;
  const float cy = y + 20;
  const uint8_t arcs = connected ? wifiStrengthArcs() : 0;
  const float radii[3] = {6.0f, 11.0f, 16.0f};
  char buffer[200];
  for (uint8_t band = 0; band < 3; ++band) {
    const float r = radii[band];
    const float d = r * 0.7071f;
    snprintf(buffer, sizeof(buffer),
             "<path d='M%.1f %.1f A%.1f %.1f 0 0 1 %.1f %.1f' fill='none' "
             "stroke='#fff' stroke-width='3'%s/>",
             cx - d, cy - d, r, r, cx + d, cy - d,
             band < arcs ? "" : " stroke-dasharray='1.5 1.5'");
    svg += buffer;
  }
  snprintf(buffer, sizeof(buffer),
           "<circle cx='%.1f' cy='%.1f' r='2.5' fill='#fff'/>", cx, cy);
  svg += buffer;
  if (!connected) {
    snprintf(buffer, sizeof(buffer),
             "<path d='M%d %d l6 6 M%d %d l-6 6' stroke='#fff' stroke-width='2'/>",
             x + 18, y + 15, x + 24, y + 15);
    svg += buffer;
  }
}

void svgAqiBoxes(String& svg, const SensorReading& reading, int16_t firstX,
                 int16_t y, int16_t w, int16_t h, int16_t gap) {
  for (uint8_t index = 0; index < 5; ++index) {
    const int16_t boxX = firstX + index * (w + gap);
    const bool active = reading.ens160Available && reading.aqi >= index + 1;
    svgRect(svg, boxX, y, w, h, false);
    if (active) {
      svgRect(svg, boxX, y, w, h, true, index >= 3 ? "blink" : "");
    }
  }
}

String renderDisplaySvg() {
  const SensorReading& reading = displayedReading;
  static const char* const weekdays[] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};
  char dateText[24] = "--.--.----";
  char timeText[9] = "--:--:--";
  String ipText = "IP: --";
  String temperatureText = "--.-";
  String humidityText = "--";
  String tvocText = "----";
  String eco2Text = "----";
  String aqiText = "--";

  if (displayedTimeValid) {
    snprintf(dateText, sizeof(dateText), "%s %02d.%02d.%04d",
             weekdays[displayedTime.tm_wday], displayedTime.tm_mday,
             displayedTime.tm_mon + 1, displayedTime.tm_year + 1900);
    strftime(timeText, sizeof(timeText), "%H:%M:%S", &displayedTime);
  }
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected) ipText = "IP:" + WiFi.localIP().toString();
  if (reading.climateAvailable) {
    temperatureText = String(reading.temperature, 1);
    humidityText = String(reading.humidity, 0);
  }
  if (reading.ens160Available) {
    tvocText = String(reading.tvoc);
    eco2Text = String(reading.eco2);
    aqiText = String(reading.aqi);
  }
  const String rating = ratingText(reading);
  const uint8_t ratingSize = rating.length() <= 10 ? 19 : 14;

  const int16_t width = displayPortrait ? 176 : 264;
  const int16_t height = displayPortrait ? 264 : 176;
  String svg;
  svg.reserve(4500);
  char head[300];
  snprintf(head, sizeof(head),
           "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 %d %d' "
           "width='%d' height='%d' font-family='Helvetica,Arial,sans-serif'>"
           "<style>.blink{animation:b 1s steps(1) infinite}"
           "@keyframes b{50%%{opacity:0}}</style>"
           "<rect width='%d' height='%d' fill='#fff'/>",
           width, height, width, height, width, height);
  svg += head;

  if (displayPortrait) {
    svgRect(svg, 0, 0, width, 46, true);
    svgText(svg, 6, 17, 14, dateText, "start", true);
    svgText(svg, 6, 40, 25, timeText, "start", true);
    svgWifiIcon(svg, width - WIFI_ICON_WIDTH - 3, 18, wifiConnected);

    svgText(svg, 44, 60, 11, "Temperatur", "middle");
    svgText(svg, 132, 60, 11, "Feuchte", "middle");
    svgValueWithUnit(svg, 44, 87, temperatureText, "\xC2\xB0" "C");
    svgValueWithUnit(svg, 132, 87, humidityText, "%");
    svgRect(svg, 88, 52, 1, 42, true);
    svgRect(svg, 0, 98, width, 2, true);

    svgText(svg, 88, 112, 11, "Luftqualit\xC3\xA4" "t (AQI)", "middle");
    svgText(svg, 34, 142, 25, aqiText, "middle");
    svgText(svg, 116, 140, ratingSize, rating, "middle");
    svgAqiBoxes(svg, reading, P_AQI_BAR_FIRST_BOX_X, P_AQI_BAR_Y,
                P_AQI_BOX_WIDTH, P_AQI_BOX_HEIGHT, P_AQI_BOX_GAP);
    svgRect(svg, 0, 172, width, 2, true);

    svgText(svg, 10, 195, 14, "TVOC");
    svgText(svg, 10, 207, 11, "ppb");
    svgText(svg, width - 10, 205, 25, tvocText, "end");
    svgRect(svg, 10, 214, width - 20, 1, true);
    svgText(svg, 10, 233, 14, "eCO2");
    svgText(svg, 10, 245, 11, "ppm");
    svgText(svg, width - 10, 243, 25, eco2Text, "end");
    svgText(svg, 2, 261, 7, ipText, "start", false, false);
  } else {
    svgRect(svg, 0, 0, width, 29, true);
    svgText(svg, 6, 22, 19, dateText, "start", true);
    svgWifiIcon(svg, 140, 3, wifiConnected);
    svgText(svg, width - 7, 22, 19, timeText, "end", true);

    svgText(svg, 66, 43, 11, "Temperatur", "middle");
    svgText(svg, 198, 43, 11, "Feuchte", "middle");
    svgValueWithUnit(svg, 66, 70, temperatureText, "\xC2\xB0" "C");
    svgValueWithUnit(svg, 198, 70, humidityText, "%");
    svgRect(svg, 132, 35, 1, 41, true);
    svgRect(svg, 0, 80, width, 2, true);

    svgText(svg, 66, 93, 11, "Luftqualit\xC3\xA4" "t (AQI)", "middle");
    svgAqiBoxes(svg, reading, AQI_BAR_FIRST_BOX_X, AQI_BAR_Y,
                AQI_BOX_WIDTH, AQI_BOX_HEIGHT, AQI_BOX_GAP);
    svgText(svg, 66, 121, 25, aqiText, "middle");
    svgText(svg, 198, 120, ratingSize, rating, "middle");

    svgRect(svg, 0, 129, width, 2, true);
    svgRect(svg, 132, 134, 1, 42, true);
    svgText(svg, 66, 142, 11, "TVOC (ppb)", "middle");
    svgText(svg, 198, 142, 11, "eCO2 (ppm)", "middle");
    svgText(svg, 66, 168, 25, tvocText, "middle");
    svgText(svg, 198, 168, 25, eco2Text, "middle");
    svgText(svg, 2, 175, 7, ipText, "start", false, false);
  }
  svg += F("</svg>");
  return svg;
}

void handleDisplaySvg() {
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "image/svg+xml; charset=utf-8", renderDisplaySvg());
}

void handleStatusPage() {
  String body = F("<h2>Status <span class='pill ok'>OK</span></h2>"
                  "<div class='statuslayout'><div class='statuscol'><div class='statgrid'>");
  appendStatusStat(body, "Zustand", "Sensoranzeige aktiv");
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  appendStatusStat(body, "WLAN", wifiConnected ? "Verbunden" : "Getrennt",
                   !wifiConnected);
  appendStatusStat(body, "IP-Adresse",
                   wifiConnected ? WiFi.localIP().toString() : String("-"));
  struct tm currentTime;
  char timeText[24] = "nicht synchronisiert";
  if (readClock(currentTime)) {
    strftime(timeText, sizeof(timeText), "%d.%m.%Y %H:%M:%S", &currentTime);
  }
  appendStatusStat(body, "Uhrzeit", timeText);
  appendStatusStat(body, "Temperatur", latestReading.climateAvailable
      ? String(latestReading.temperature, 1) + " C" : String("-"));
  appendStatusStat(body, "Feuchte", latestReading.climateAvailable
      ? String(latestReading.humidity, 0) + " %" : String("-"));
  String aqiText = latestReading.ens160Available
      ? String(latestReading.aqi) + " - " + aqiLabel(latestReading.aqi)
      : String("Keine Daten");
  appendStatusStat(body, "AQI", aqiText, !latestReading.ens160Available);
  appendStatusStat(body, "TVOC", latestReading.ens160Available
      ? String(latestReading.tvoc) + " ppb" : String("-"));
  appendStatusStat(body, "eCO2", latestReading.ens160Available
      ? String(latestReading.eco2) + " ppm" : String("-"));
  appendStatusStat(body, "MQTT", mqttEnabled
      ? (mqttClient.connected() ? "Verbunden" : "Getrennt") : "Deaktiviert",
      mqttEnabled && !mqttClient.connected());
  body += F("</div></div><div class='screencol'>"
            "<img id='epd' src='/display.svg' alt='Display' style='width:");
  body += displayPortrait ? F("264px") : F("396px");
  body += F("'></div></div>"
            "<script>setInterval(function(){"
            "document.getElementById('epd').src='/display.svg?t='+Date.now();"
            "fetch('/?t='+Date.now(),{cache:'no-store'}).then(function(r){return r.text()})"
            ".then(function(h){var d=new DOMParser().parseFromString(h,'text/html');"
            "var n=d.querySelector('.statgrid'),o=document.querySelector('.statgrid');"
            "if(n&&o)o.innerHTML=n.innerHTML}).catch(function(){})},2000)</script>");
  sendPage("Status", body);
}

void handleParametersPage() {
  if (!requireWebAdmin()) return;
  String body = F("<h2>Parameter</h2><form method='post' action='/params'>"
                  "<div class='row'><label for='refresh'>Display-Refresh (Sekunden)</label>"
                  "<input id='refresh' name='refresh' type='number' min='1' max='3600' value='");
  body += String(dashboardRefreshIntervalMs / 1000UL);
  body += F("' required></div><label class='chk'><input name='portrait' type='checkbox' value='1'");
  if (displayPortrait) body += F(" checked");
  body += F("> Display hochkant (90&deg; gedreht)</label>");
  body += F("<div class='row'><div class='rowhead'><label for='brightness'>LED-Helligkeit</label><span class='val' id='brightnessValue'>");
  body += String(aqiLedBrightness);
  body += F("</span></div><input id='brightness' name='brightness' type='range' min='1' max='255' value='");
  body += String(aqiLedBrightness);
  body += F("' oninput=\"document.getElementById('brightnessValue').textContent=this.value\"></div>");
  body += F("<div class='row'><label for='tempOffset'>Temperatur-Kompensation (C)</label>"
            "<input id='tempOffset' name='tempOffset' type='number' min='-10' max='10' step='0.1' value='");
  body += String(temperatureOffsetC, 1);
  body += F("' required><div class='hint'>Wird zur gemessenen Temperatur addiert, z. B. -1.5 bei Eigenerwärmung.</div></div>");
  body += F("<div class='row'><label for='fallbackTemp'>HDC1008 Fallback Temperatur (C)</label>"
            "<input id='fallbackTemp' name='fallbackTemp' type='number' min='-40' max='125' step='0.1' value='");
  body += String(fallbackTemperatureC, 1);
  body += F("' required></div><div class='row'><label for='fallbackHum'>HDC1008 Fallback Feuchte (%)</label>"
            "<input id='fallbackHum' name='fallbackHum' type='number' min='0' max='100' step='0.1' value='");
  body += String(fallbackHumidityPercent, 1);
  body += F("' required></div><div class='subcard'><h3>Median-Filter</h3>"
            "<div class='row'><label for='tvocMedian'>TVOC Fenster (Samples, 1 = aus)</label>"
            "<input id='tvocMedian' name='tvocMedian' type='number' min='1' max='15' step='1' value='");
  body += String(tvocMedianWindow);
  body += F("' required></div><div class='row'><label for='eco2Median'>eCO2 Fenster (Samples, 1 = aus)</label>"
            "<input id='eco2Median' name='eco2Median' type='number' min='1' max='15' step='1' value='");
  body += String(eco2MedianWindow);
  body += F("' required></div><div class='hint'>1 Sample pro Sekunde. Ungerade Werte (3, 5, 7 ...) empfohlen.</div></div>"
            "<div class='subcard'><h3>Simulation</h3>"
            "<label class='chk'><input name='simulateSensorData' type='checkbox' value='1'");
  if (simulateSensorData) body += F(" checked");
  body += F("> Sensorwerte simulieren</label>"
            "<label class='chk'><input name='simulateAqiTest' type='checkbox' value='1'");
  if (simulateAqiTest) body += F(" checked");
  body += F("> AQI-Test: Stufen 1-5 zyklisch testen</label>"
            "<label class='chk'><input name='simulateAuto' type='checkbox' value='1'");
  if (simulateValuesAutomatically) body += F(" checked");
  body += F("> Werte automatisch ändern</label>"
            "<div class='row'><label for='simTemp'>Temperatur (C)</label>"
            "<input id='simTemp' name='simTemp' type='number' min='-40' max='125' step='0.1' value='");
  body += String(simulatedTemperatureC, 1);
  body += F("'></div><div class='row'><label for='simHum'>Feuchte (%)</label>"
            "<input id='simHum' name='simHum' type='number' min='0' max='100' step='0.1' value='");
  body += String(simulatedHumidityPercent, 1);
  body += F("'></div><div class='row'><label for='simAqi'>AQI (1-5)</label>"
            "<input id='simAqi' name='simAqi' type='number' min='1' max='5' step='1' value='");
  body += String(simulatedAqi);
  body += F("'></div><div class='row'><label for='simTvoc'>TVOC (ppb)</label>"
            "<input id='simTvoc' name='simTvoc' type='number' min='0' max='65535' step='1' value='");
  body += String(simulatedTvoc);
  body += F("'></div><div class='row'><label for='simEco2'>eCO2 (ppm)</label>"
            "<input id='simEco2' name='simEco2' type='number' min='0' max='65535' step='1' value='");
  body += String(simulatedEco2);
  body += F("'></div><div class='hint'>Automatische Werte: AQI-Test alle 10 s, Klima alle 10 s, TVOC/eCO2 alle 5 s. Bei deaktivierter Automatik bleiben die eingegebenen Werte konstant.</div></div>"
            "<button class='btn' type='submit'>Speichern</button></form>"
            "<div class='subcard'><h3>Neustart</h3><div class='actionbar'>"
            "<form method='post' action='/restart-sensors'>"
            "<button class='btn secondary' type='submit'>Sensoren neu starten</button></form>"
            "<form method='post' action='/restart' onsubmit=\"return confirm('Gerät neu starten?')\">"
            "<button class='btn secondary' type='submit'>Gerät neu starten</button></form>"
            "</div><div class='hint'>Nach dem Sensor-Neustart braucht der ENS160 einige Minuten Aufwärmzeit.</div></div>");
  sendPage("Parameter", body);
}

void handleParametersSave() {
  if (!requireWebAdmin()) return;
  const long refreshSeconds = webServer.arg("refresh").toInt();
  const long brightness = webServer.arg("brightness").toInt();
  const float fallbackTemp = webServer.arg("fallbackTemp").toFloat();
  const float fallbackHum = webServer.arg("fallbackHum").toFloat();
  const float simTemp = webServer.arg("simTemp").toFloat();
  const float simHum = webServer.arg("simHum").toFloat();
  const long simAqi = webServer.arg("simAqi").toInt();
  const long simTvoc = webServer.arg("simTvoc").toInt();
  const long simEco2 = webServer.arg("simEco2").toInt();
  const long tvocWindow = webServer.arg("tvocMedian").toInt();
  const long eco2Window = webServer.arg("eco2Median").toInt();
  const float tempOffset = webServer.arg("tempOffset").toFloat();
  if (refreshSeconds < 1 || refreshSeconds > 3600 || brightness < 1 ||
      brightness > 255 || !isfinite(fallbackTemp) || fallbackTemp < -40 ||
      fallbackTemp > 125 || !isfinite(fallbackHum) || fallbackHum < 0 ||
      fallbackHum > 100 || !isfinite(simTemp) || simTemp < -40 ||
      simTemp > 125 || !isfinite(simHum) || simHum < 0 || simHum > 100 ||
      simAqi < 1 || simAqi > 5 || simTvoc < 0 || simTvoc > 65535 ||
      simEco2 < 0 || simEco2 > 65535 ||
      tvocWindow < 1 || tvocWindow > MEDIAN_MAX_WINDOW ||
      eco2Window < 1 || eco2Window > MEDIAN_MAX_WINDOW ||
      !isfinite(tempOffset) || tempOffset < -10 || tempOffset > 10) {
    sendPage("Parameter", F("<h2>Parameter</h2><p class='msg info'>Ungültige Werte. "
                            "Refresh 1-3600 s, LED 1-255, Temperatur -40 bis 125 C, "
                            "Feuchte 0 bis 100 %, Median-Fenster 1-15, Kompensation -10 bis 10 C.</p>"), 400);
    return;
  }

  dashboardRefreshIntervalMs = static_cast<uint32_t>(refreshSeconds) * 1000UL;
  aqiLedBrightness = static_cast<uint8_t>(brightness);
  temperatureOffsetC = tempOffset;
  const bool portrait = webServer.hasArg("portrait");
  if (portrait != displayPortrait) {
    displayPortrait = portrait;
    displayLayoutChanged = true;
  }
  fallbackTemperatureC = fallbackTemp;
  fallbackHumidityPercent = fallbackHum;
  simulateSensorData = webServer.hasArg("simulateSensorData");
  simulateAqiTest = webServer.hasArg("simulateAqiTest");
  simulateValuesAutomatically = webServer.hasArg("simulateAuto");
  simulatedTemperatureC = simTemp;
  simulatedHumidityPercent = simHum;
  simulatedAqi = static_cast<uint8_t>(simAqi);
  simulatedTvoc = static_cast<uint16_t>(simTvoc);
  simulatedEco2 = static_cast<uint16_t>(simEco2);
  if (tvocWindow != tvocMedianWindow) resetMedian(tvocMedian);
  if (eco2Window != eco2MedianWindow) resetMedian(eco2Median);
  tvocMedianWindow = static_cast<uint8_t>(tvocWindow);
  eco2MedianWindow = static_cast<uint8_t>(eco2Window);
  aqiPixels.setBrightness(aqiLedBrightness);
  updateAqiPixels(latestReading);
  if (ens160Ready) {
    ens160.setTempAndHum(fallbackTemperatureC, fallbackHumidityPercent);
  }
  savePersistentSettings();
  sendPage("Parameter", F("<h2>Parameter</h2><p class='msg ok'>Einstellungen gespeichert.</p>"));
}

void handleSensorRestart() {
  if (!requireWebAdmin()) return;
  sensorRestartPending = true;
  sendPage("Parameter", F("<h2>Parameter</h2><p class='msg ok'>Sensoren werden neu gestartet.</p>"
                          "<p><a href='/params'>Zurück</a></p>"));
}

void handleDeviceRestart() {
  if (!requireWebAdmin()) return;
  sendPage("Parameter", F("<h2>Parameter</h2><p class='msg ok'>Gerät startet neu...</p>"));
  delay(700);
  ESP.restart();
}

void handleWifiPage() {
  if (!requireWebAdmin()) return;
  String body = F("<h2>WLAN</h2><form method='post' action='/wifi'>"
                  "<div class='row'><label for='ssid'>SSID</label><input id='ssid' name='ssid' type='text' maxlength='32' value='");
  body += escapeHtml(configuredWifiSsid);
  body += F("' required></div><div class='row'><label for='password'>Passwort</label>"
            "<input id='password' name='password' type='password' maxlength='64' "
            "autocomplete='new-password' placeholder='Leer lassen, um es beizubehalten'>"
            "<div class='hint'>Passwort bleibt unverändert, wenn das Feld leer bleibt.</div></div>"
            "<label class='chk'><input name='clearPassword' type='checkbox' value='1'> Passwort leeren</label>"
            "<button class='btn' type='submit'>Speichern und verbinden</button></form>");
  sendPage("WLAN", body);
}

void handleWifiSave() {
  if (!requireWebAdmin()) return;
  String ssid = webServer.arg("ssid");
  ssid.trim();
  if (ssid.isEmpty()) {
    sendPage("WLAN", F("<h2>WLAN</h2><p class='msg info'>SSID darf nicht leer sein.</p>"), 400);
    return;
  }
  configuredWifiSsid = ssid;
  const String password = webServer.arg("password");
  if (webServer.hasArg("clearPassword")) {
    configuredWifiPassword = "";
  } else if (!password.isEmpty()) {
    configuredWifiPassword = password;
  }
  savePersistentSettings();
  wifiReconnectPending = true;
  sendPage("WLAN", F("<h2>WLAN</h2><p class='msg ok'>Zugangsdaten gespeichert. "
                      "Die Verbindung wird neu aufgebaut; die neue IP-Adresse "
                      "erscheint im seriellen Monitor.</p>"));
}

bool validMqttTopic(const String& topic) {
  if (topic.isEmpty()) return false;
  for (size_t index = 0; index < topic.length(); ++index) {
    const char value = topic[index];
    if (!((value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
          (value >= '0' && value <= '9') || value == '_' || value == '-' ||
          value == '/')) {
      return false;
    }
  }
  return true;
}

void handleMqttPage() {
  if (!requireWebAdmin()) return;
  String body = F("<h2>MQTT / Home Assistant</h2><form method='post' action='/mqtt'>"
                  "<label class='chk'><input name='enabled' type='checkbox' value='1'");
  if (mqttEnabled) body += F(" checked");
  body += F("> MQTT aktiv</label><div class='row'><label for='host'>Broker-Adresse</label>"
            "<input id='host' name='host' type='text' maxlength='128' value='");
  body += escapeHtml(mqttHost);
  body += F("'></div><div class='row'><label for='port'>Port</label><input id='port' name='port' type='number' min='1' max='65535' value='");
  body += String(mqttPort);
  body += F("'></div><div class='row'><label for='user'>Benutzername (optional)</label>"
            "<input id='user' name='user' type='text' maxlength='64' value='");
  body += escapeHtml(mqttUsername);
  body += F("'></div><div class='row'><label for='password'>Passwort (leer lassen, um es beizubehalten)</label>"
            "<input id='password' name='password' type='password' maxlength='64' autocomplete='new-password'>"
            "</div><div class='row'><label for='topic'>Basis-Topic</label><input id='topic' name='topic' type='text' maxlength='64' value='");
  body += escapeHtml(mqttBaseTopic);
  body += F("' required><div class='hint'>Home Assistant MQTT Discovery wird automatisch veröffentlicht.</div></div>"
            "<button class='btn' type='submit'>Speichern</button></form>");
  sendPage("MQTT", body);
}

void handleMqttSave() {
  if (!requireWebAdmin()) return;
  String host = webServer.arg("host");
  host.trim();
  String topic = webServer.arg("topic");
  topic.trim();
  while (topic.endsWith("/")) topic.remove(topic.length() - 1);
  const long port = webServer.arg("port").toInt();
  const bool enabled = webServer.hasArg("enabled");
  if ((enabled && host.isEmpty()) || port < 1 || port > 65535 ||
      !validMqttTopic(topic)) {
    sendPage("MQTT", F("<h2>MQTT</h2><p class='msg info'>Broker und Topic prüfen; "
                       "Port muss 1-65535 sein.</p>"), 400);
    return;
  }
  mqttHost = host;
  mqttPort = static_cast<uint16_t>(port);
  mqttUsername = webServer.arg("user");
  const String password = webServer.arg("password");
  if (!password.isEmpty()) mqttPassword = password;
  mqttBaseTopic = topic;
  mqttEnabled = enabled;
  savePersistentSettings();
  mqttReconnectPending = true;
  sendPage("MQTT", F("<h2>MQTT</h2><p class='msg ok'>Einstellungen gespeichert. "
                      "Verbindungsstatus ist auf der Statusseite sichtbar.</p>"));
}

String mqttDeviceId() {
  String id = WiFi.macAddress();
  id.replace(":", "");
  id.toLowerCase();
  return id;
}

void publishMqttDiscovery() {
  const String id = mqttDeviceId();
  const String stateTopic = mqttBaseTopic + "/" + id;
  const String availabilityTopic = stateTopic + "/availability";
  const char* suffixes[] = {"aqi", "temperature", "humidity", "tvoc", "eco2"};
  const char* names[] = {"AQI", "Temperature", "Humidity", "TVOC", "eCO2"};
  const char* classes[] = {"aqi", "temperature", "humidity",
                           "volatile_organic_compounds_parts", "carbon_dioxide"};
  const char* units[] = {"", "\\u00B0C", "%", "ppb", "ppm"};

  for (uint8_t index = 0; index < 5; ++index) {
    const bool hasUnit = units[index][0] != '\0';
    char discoveryTopic[160];
    char payload[700];
    snprintf(discoveryTopic, sizeof(discoveryTopic),
             "homeassistant/sensor/airq_%s_%s/config", id.c_str(), suffixes[index]);
    snprintf(payload, sizeof(payload),
             "{\"name\":\"AIRQ %s\",\"uniq_id\":\"airq_%s_%s\","
             "\"stat_t\":\"%s/%s\",\"avty_t\":\"%s\","
             "\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
             "\"dev_cla\":\"%s\",\"dev\":{\"ids\":[\"airq_%s\"],"
             "\"name\":\"AIRQ Sensor\",\"mf\":\"ESP32\","
             "\"mdl\":\"Waveshare e-Paper Monitor\"},"
             "\"stat_cla\":\"measurement\"%s%s%s}",
             names[index], id.c_str(), suffixes[index], stateTopic.c_str(),
             suffixes[index], availabilityTopic.c_str(), classes[index], id.c_str(),
             hasUnit ? ",\"unit_of_meas\":\"" : "",
             units[index], hasUnit ? "\"" : "");
    mqttClient.publish(discoveryTopic, payload, true);
  }
}

void maintainMqttConnection() {
  if (mqttReconnectPending) {
    mqttReconnectPending = false;
    mqttClient.disconnect();
    lastMqttConnectAttemptMillis = 0;
  }
  if (!mqttEnabled || mqttHost.isEmpty() || WiFi.status() != WL_CONNECTED) {
    mqttClient.loop();
    return;
  }
  if (!mqttClient.connected() &&
      millis() - lastMqttConnectAttemptMillis >= 5000) {
    lastMqttConnectAttemptMillis = millis();
    const String id = mqttDeviceId();
    const String availabilityTopic = mqttBaseTopic + "/" + id + "/availability";
    char clientId[40];
    snprintf(clientId, sizeof(clientId), "airq-%s", id.c_str());
    bool connected;
    if (mqttUsername.isEmpty()) {
      connected = mqttClient.connect(clientId, availabilityTopic.c_str(), 0,
                                     true, "offline");
    } else {
      connected = mqttClient.connect(clientId, mqttUsername.c_str(),
                                     mqttPassword.c_str(),
                                     availabilityTopic.c_str(), 0, true,
                                     "offline");
    }
    if (connected) {
      mqttClient.publish(availabilityTopic.c_str(), "online", true);
      publishMqttDiscovery();
      Serial.println("MQTT connected; Home Assistant discovery published.");
    } else {
      Serial.printf("MQTT connection failed, state=%d\n", mqttClient.state());
    }
  }
  mqttClient.loop();
}

void publishMqttReadings(const SensorReading& reading) {
  if (!mqttEnabled || !mqttClient.connected()) return;
  const String prefix = mqttBaseTopic + "/" + mqttDeviceId() + "/";
  char value[24];
  if (reading.ens160Available) {
    snprintf(value, sizeof(value), "%u", reading.aqi);
    mqttClient.publish((prefix + "aqi").c_str(), value, true);
    snprintf(value, sizeof(value), "%u", reading.tvoc);
    mqttClient.publish((prefix + "tvoc").c_str(), value, true);
    snprintf(value, sizeof(value), "%u", reading.eco2);
    mqttClient.publish((prefix + "eco2").c_str(), value, true);
  }
  if (reading.climateAvailable) {
    snprintf(value, sizeof(value), "%.1f", reading.temperature);
    mqttClient.publish((prefix + "temperature").c_str(), value, true);
    snprintf(value, sizeof(value), "%.1f", reading.humidity);
    mqttClient.publish((prefix + "humidity").c_str(), value, true);
  }
  mqttClient.publish((prefix + "availability").c_str(), "online", true);
}

void handleFirmwarePage() {
  if (!requireWebAdmin()) return;
  sendPage("Firmware", F(
      "<h2>Firmware</h2>"
      "<div id='fwForm'><p class='hint'>PlatformIO-Datei firmware.bin auswählen.</p>"
      "<form id='fwUpload' method='post' action='/firmware' enctype='multipart/form-data'>"
      "<div class='row'><label for='firmware'>Firmware-Datei</label>"
      "<input id='firmware' name='firmware' type='file' accept='.bin' required></div>"
      "<button class='btn' type='submit'>Hochladen und installieren</button></form></div>"
      "<div id='fwStatus' style='display:none'>"
      "<p id='fwFile' class='hint'></p>"
      "<div class='subcard'><h3>1. Datei hochladen</h3>"
      "<div style='height:10px;background:var(--border);border-radius:5px;overflow:hidden'>"
      "<div id='fwBar' style='height:100%;width:0;background:var(--primary);transition:width .2s'></div></div>"
      "<p id='fwUp' style='margin:8px 0 0'>0 %</p></div>"
      "<div class='subcard'><h3>2. Firmware einspielen</h3><p id='fwInst' style='margin:0'>Wartet auf Upload...</p></div>"
      "<div class='subcard'><h3>3. Neustart</h3><p id='fwBoot' style='margin:0'>Wartet...</p></div>"
      "<div class='actionbar' id='fwRetry' style='display:none'>"
      "<a class='btn secondary' href='/firmware'>Erneut versuchen</a></div></div>"
      "<script>"
      "var f=document.getElementById('fwUpload');"
      "function $(i){return document.getElementById(i)}"
      "function fail(t){$('fwInst').innerHTML=\"<span class='pill err'>Fehler</span> \"+t;$('fwRetry').style.display='flex';}"
      "function waitBoot(n){fetch('/',{cache:'no-store'}).then(function(r){"
      "$('fwBoot').innerHTML=\"<span class='pill ok'>Fertig</span> Ger\\u00e4t l\\u00e4uft wieder. Weiter zur Statusseite...\";"
      "setTimeout(function(){location.href='/'},1500)}).catch(function(){"
      "$('fwBoot').textContent='Ger\\u00e4t startet neu... ('+n+' s)';setTimeout(function(){waitBoot(n+2)},2000)})}"
      "f.onsubmit=function(e){e.preventDefault();var file=$('firmware').files[0];if(!file)return;"
      "$('fwForm').style.display='none';$('fwStatus').style.display='block';"
      "$('fwFile').textContent='Datei: '+file.name+' ('+Math.round(file.size/1024)+' KB)';"
      "var x=new XMLHttpRequest();x.open('POST','/firmware');"
      "x.upload.onprogress=function(ev){if(!ev.lengthComputable)return;var p=Math.round(ev.loaded*100/ev.total);"
      "$('fwBar').style.width=p+'%';$('fwUp').textContent=p+' %';"
      "$('fwInst').textContent='Wird w\\u00e4hrend des Uploads in den Flash geschrieben...'};"
      "x.upload.onload=function(){$('fwUp').innerHTML=\"<span class='pill ok'>Hochgeladen</span> Datei vollst\\u00e4ndig \\u00fcbertragen.\";"
      "$('fwInst').textContent='Firmware wird gepr\\u00fcft und aktiviert...'};"
      "x.onload=function(){if(x.status==200){"
      "$('fwInst').innerHTML=\"<span class='pill ok'>Installiert</span> Firmware erfolgreich eingespielt.\";"
      "$('fwBoot').textContent='Ger\\u00e4t startet neu...';setTimeout(function(){waitBoot(4)},4000)}"
      "else{fail('Update fehlgeschlagen (HTTP '+x.status+'). Seriellen Monitor pr\\u00fcfen.')}};"
      "x.onerror=function(){fail('Verbindung abgebrochen.')};"
      "var d=new FormData();d.append('firmware',file);x.send(d)};"
      "</script>"));
}

void handleFirmwareUpload() {
  if (!webServer.authenticate(WEB_ADMIN_USERNAME, WEB_ADMIN_PASSWORD)) return;
  HTTPUpload& upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    otaUploadSucceeded = false;
    Serial.printf("OTA upload started: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    otaUploadSucceeded = Update.end(true);
    if (otaUploadSucceeded) {
      Serial.printf("OTA upload finished: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaUploadSucceeded = false;
    Serial.println("OTA upload aborted.");
  }
}

void handleFirmwareResult() {
  if (!requireWebAdmin()) return;
  if (!otaUploadSucceeded) {
    sendPage("Firmware", F("<h2>Firmware</h2><p class='pill err'>Update fehlgeschlagen. "
                            "Seriellen Monitor prüfen.</p>"), 500);
    return;
  }
  webServer.send(200, "text/html; charset=utf-8",
                 renderPage("Firmware", F("<h2>Firmware</h2><p class='msg ok'>Update erfolgreich. "
                                           "Das Gerät startet neu.</p>")));
  delay(700);
  ESP.restart();
}

void setupWebServer() {
  webServer.on("/", HTTP_GET, handleStatusPage);
  webServer.on("/display.svg", HTTP_GET, handleDisplaySvg);
  webServer.on("/params", HTTP_GET, handleParametersPage);
  webServer.on("/params", HTTP_POST, handleParametersSave);
  webServer.on("/restart-sensors", HTTP_POST, handleSensorRestart);
  webServer.on("/restart", HTTP_POST, handleDeviceRestart);
  webServer.on("/wifi", HTTP_GET, handleWifiPage);
  webServer.on("/wifi", HTTP_POST, handleWifiSave);
  webServer.on("/mqtt", HTTP_GET, handleMqttPage);
  webServer.on("/mqtt", HTTP_POST, handleMqttSave);
  webServer.on("/firmware", HTTP_GET, handleFirmwarePage);
  webServer.on("/firmware", HTTP_POST, handleFirmwareResult,
                handleFirmwareUpload);
  webServer.onNotFound([]() {
    sendPage("Nicht gefunden", F("<h2>404</h2><p>Diese Seite gibt es nicht.</p>"), 404);
  });
  webServer.begin();
  Serial.println("Web interface started on port 80.");
}

void applyPendingNetworkSettings() {
  if (wifiReconnectPending) {
    wifiReconnectPending = false;
    Serial.printf("Reconnecting to WLAN '%s'.\n", configuredWifiSsid.c_str());
    WiFi.disconnect(false, false);
    WiFi.begin(configuredWifiSsid.c_str(), configuredWifiPassword.c_str());
  }
  if (mqttReconnectPending) {
    mqttReconnectPending = false;
    mqttClient.disconnect();
    mqttClient.setServer(mqttHost.c_str(), mqttPort);
    lastMqttConnectAttemptMillis = 0;
  }
}

void connectToWiFi() {
  if (configuredWifiSsid.isEmpty()) {
    Serial.println("Wi-Fi not configured; clock will show --:--.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(configuredWifiSsid.c_str(), configuredWifiPassword.c_str());
  Serial.print("Connecting to Wi-Fi");

  const unsigned long startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < 15000) {
    Serial.print('.');
    delay(250);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi unavailable; clock will retry NTP in the background.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  loadPersistentSettings();
  mqttClient.setBufferSize(768);
  mqttClient.setServer(mqttHost.c_str(), mqttPort);

  aqiPixels.begin();
  aqiPixels.setBrightness(aqiLedBrightness);
  aqiPixels.clear();
  aqiPixels.show();

  SPI.begin(EPD_SCK_PIN, -1, EPD_MOSI_PIN, EPD_CS_PIN);
  display.init(115200, true, 2, false);
  display.setRotation(displayRotation());
  textRenderer.begin(display);
  textRenderer.setFontMode(1);

  Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
  if (simulateSensorData) {
    Serial.println("Sensor simulation enabled.");
  } else {
    hdc1008Ready = initializeHDC1008();
    ens160Ready = initializeENS160();
  }

  connectToWiFi();
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");
  setupWebServer();

  struct tm currentTime;
  const bool clockAvailable = readClock(currentTime);
  const SensorReading reading = readSensors();
  latestReading = reading;
  displayedAqi = reading.aqi;
  displayedAqiAvailable = reading.ens160Available;
  updateAqiPixels(reading);
  aqiAlertVisible = true;
  logStatus(reading, clockAvailable ? &currentTime : nullptr);
  drawDashboard(reading, clockAvailable ? &currentTime : nullptr, true);
  lastAqiBlinkMillis = millis();
  lastRefreshMillis = millis();
  lastFullRefreshMillis = lastRefreshMillis;
}

void loop() {
  webServer.handleClient();
  applyPendingNetworkSettings();
  maintainMqttConnection();
  if (sensorRestartPending) {
    sensorRestartPending = false;
    restartSensors();
  }

  struct tm currentTime;
  const bool clockAvailable = readClock(currentTime);
  const uint32_t refreshStartedAt = millis();
  const bool dashboardRefreshDue =
      refreshStartedAt - lastRefreshMillis >= dashboardRefreshIntervalMs;
  if (displayLayoutChanged) {
    display.setRotation(displayRotation());
  }
  const bool fullRefreshDue =
      refreshStartedAt - lastFullRefreshMillis >= 1800000UL ||
      displayLayoutChanged;
  displayLayoutChanged = false;

  if (dashboardRefreshDue || fullRefreshDue) {
    const SensorReading reading = readSensors();
    latestReading = reading;
    const bool aqiChanged = reading.aqi != displayedAqi ||
                            reading.ens160Available != displayedAqiAvailable;
    if (aqiChanged) {
      aqiAlertVisible = true;
      lastAqiBlinkMillis = millis();
    }
    updateAqiPixels(reading);
    displayedAqi = reading.aqi;

    displayedAqiAvailable = reading.ens160Available;
    logStatus(reading, clockAvailable ? &currentTime : nullptr);
    drawDashboard(reading, clockAvailable ? &currentTime : nullptr,
                  fullRefreshDue);
    if (fullRefreshDue) lastFullRefreshMillis = refreshStartedAt;
    publishMqttReadings(reading);
    lastRefreshMillis = refreshStartedAt;
  }
  if (millis() - lastLedUpdateMillis >= LED_UPDATE_INTERVAL_MS) {
    const SensorReading ledReading = readSensors();
    latestReading = ledReading;
    updateAqiPixels(ledReading);
    lastLedUpdateMillis = millis();
  }
  const uint32_t currentMillis = millis();
  if (displayedAqiAvailable && displayedAqi >= 4 &&
      currentMillis - lastAqiBlinkMillis >= AQI_BLINK_INTERVAL_MS) {
    aqiAlertVisible = !aqiAlertVisible;
    drawAqiAlertBoxes(displayedAqi, displayedAqiAvailable, aqiAlertVisible);
    lastAqiBlinkMillis = millis();
  }

  delay(50);
}
