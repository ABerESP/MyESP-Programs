#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>

#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <GxEPD2_BW.h>
#include <U8g2_for_Adafruit_GFX.h>

#include "config.h"

DFRobot_ENS160_I2C ens160(&Wire, SENSOR_I2C_ADDRESS);
Adafruit_AHTX0 aht20;
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(
  GxEPD2_270(EPD_CS_PIN, EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN));
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
bool aht20Ready = false;
int lastRenderedMinute = -1;
uint32_t lastRefreshMillis = 0;
uint8_t minutesSinceFullRefresh = 0;
uint8_t displayedAqi = 0;
bool displayedAqiAvailable = false;
bool aqiAlertVisible = true;
uint32_t lastAqiBlinkMillis = 0;

const uint32_t AQI_BLINK_INTERVAL_MS = 500;
const int16_t AQI_BAR_FIRST_BOX_X = 148;
const int16_t AQI_BAR_Y = 89;
const int16_t AQI_BOX_WIDTH = 17;
const int16_t AQI_BOX_HEIGHT = 11;
const int16_t AQI_BOX_GAP = 4;

uint8_t simulatedAqiTestValue() {
  return static_cast<uint8_t>((millis() / 1000UL) % 5 + 1);
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

bool initializeENS160() {
  if (ens160.begin() != NO_ERR) {
    Serial.println("SEN0515 not found; will retry on the next refresh.");
    return false;
  }

  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(ENS160_FALLBACK_TEMPERATURE_C,
                       ENS160_FALLBACK_HUMIDITY_PERCENT);
  Serial.println("SEN0515 ready.");
  return true;
}

bool initializeAHT20() {
  if (!aht20.begin(&Wire)) {
    Serial.println("AHT20 not found; will retry on the next refresh.");
    return false;
  }

  Serial.println("AHT20 ready.");
  return true;
}

SensorReading readSensors() {
#if SIMULATE_SENSOR_DATA
  const float phase = millis() / 1000.0f;
#if SIMULATE_AQI_TEST
  const uint8_t aqi = simulatedAqiTestValue();
#else
  const uint8_t aqi = 1 + (static_cast<uint32_t>(phase / 5.0f) % 5);
#endif
  SensorReading reading = {
      true,
      true,
      0,
      aqi,
      static_cast<uint16_t>(180.0f + 90.0f * (1.0f + sinf(phase / 4.0f))),
      static_cast<uint16_t>(430.0f + 140.0f * (1.0f + sinf(phase / 6.0f))),
      22.0f + 1.5f * sinf(phase / 8.0f),
      45.0f + 8.0f * sinf(phase / 10.0f)};
  return reading;
#else
  SensorReading reading = {false, false, 3, 0, 0, 0, 0.0f, 0.0f};

  if (!aht20Ready) {
    aht20Ready = initializeAHT20();
  }

  if (aht20Ready) {
    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    aht20.getEvent(&humidityEvent, &temperatureEvent);
    if (isfinite(temperatureEvent.temperature) &&
        isfinite(humidityEvent.relative_humidity)) {
      reading.climateAvailable = true;
      reading.temperature = temperatureEvent.temperature;
      reading.humidity = humidityEvent.relative_humidity;
    }
  }

  if (!ens160Ready) {
    ens160Ready = initializeENS160();
  }

  if (!ens160Ready) {
#if SIMULATE_AQI_TEST
    reading.ens160Available = true;
    reading.status = 0;
  reading.aqi = simulatedAqiTestValue();
#endif
    return reading;
  }

  if (reading.climateAvailable) {
    ens160.setTempAndHum(reading.temperature, reading.humidity);
  }

  reading.ens160Available = true;
  reading.status = ens160.getENS160Status();
  reading.aqi = ens160.getAQI();
  reading.tvoc = ens160.getTVOC();
  reading.eco2 = ens160.getECO2();
#if SIMULATE_AQI_TEST
  reading.status = 0;
  reading.aqi = simulatedAqiTestValue();
#endif
  return reading;
#endif
}

bool readClock(struct tm& currentTime) {
  return getLocalTime(&currentTime, 20);
}

void logStatus(const SensorReading& reading, const struct tm* currentTime) {
  char timeText[24] = "not synchronized";
  if (currentTime != nullptr) {
    strftime(timeText, sizeof(timeText), "%Y-%m-%d %H:%M:%S", currentTime);
  }

#if SIMULATE_SENSOR_DATA
  const char* const sensorSource = "simulated";
#else
  const char* const sensorSource = "hardware";
#endif

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

void drawDashboard(const SensorReading& reading, const struct tm* currentTime,
                   bool fullRefresh) {
  static const char* const weekdays[] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};
  char dateText[24] = "--.--.----";
  char timeText[9] = "--:--:--";
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
    display.fillRect(0, 0, display.width(), 29, GxEPD_BLACK);

    drawText(dateText, 6, 24, FONT_LARGE, GxEPD_WHITE);
    textRenderer.setFont(FONT_LARGE);
    const int16_t timeWidth = textRenderer.getUTF8Width(timeText);
    drawText(timeText, display.width() - 7 - timeWidth, 24, FONT_LARGE,
             GxEPD_WHITE);

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
      if (reading.status == 1 || reading.status == 2) {
        rating = "Aufw\xC3\xA4" "rmen...";
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
  } while (display.nextPage());
}

void connectToWiFi() {
  if (strlen(WIFI_SSID) == 0) {
    Serial.println("Wi-Fi not configured; clock will show --:--.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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

  SPI.begin(EPD_SCK_PIN, -1, EPD_MOSI_PIN, EPD_CS_PIN);
  display.init(115200, true, 2, false);
  display.setRotation(1);
  textRenderer.begin(display);
  textRenderer.setFontMode(1);

  Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
#if SIMULATE_SENSOR_DATA
  Serial.println("Sensor simulation enabled.");
#else
  aht20Ready = initializeAHT20();
  ens160Ready = initializeENS160();
#endif

  connectToWiFi();
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");

  struct tm currentTime;
  const bool clockAvailable = readClock(currentTime);
  const SensorReading reading = readSensors();
  displayedAqi = reading.aqi;
  displayedAqiAvailable = reading.ens160Available;
  aqiAlertVisible = true;
  logStatus(reading, clockAvailable ? &currentTime : nullptr);
  drawDashboard(reading, clockAvailable ? &currentTime : nullptr, true);
  lastAqiBlinkMillis = millis();
  if (clockAvailable) {
    lastRenderedMinute = currentTime.tm_min;
  }
  lastRefreshMillis = millis();
}

void loop() {
  struct tm currentTime;
  const bool clockAvailable = readClock(currentTime);
  const bool minuteChanged =
      clockAvailable && currentTime.tm_min != lastRenderedMinute;
  const uint32_t refreshStartedAt = millis();
  const bool dashboardRefreshDue =
      refreshStartedAt - lastRefreshMillis >= 1000;

  if (minuteChanged || dashboardRefreshDue) {
    const SensorReading reading = readSensors();
    const bool aqiChanged = reading.aqi != displayedAqi ||
                            reading.ens160Available != displayedAqiAvailable;
    if (aqiChanged) {
      aqiAlertVisible = true;
      lastAqiBlinkMillis = millis();
    }
    displayedAqi = reading.aqi;
    displayedAqiAvailable = reading.ens160Available;
    logStatus(reading, clockAvailable ? &currentTime : nullptr);
    bool fullRefresh = false;
    if (minuteChanged && ++minutesSinceFullRefresh >= 30) {
      fullRefresh = true;
      minutesSinceFullRefresh = 0;
    }
    drawDashboard(reading, clockAvailable ? &currentTime : nullptr,
                  fullRefresh);
    if (clockAvailable) {
      lastRenderedMinute = currentTime.tm_min;
    }
    lastRefreshMillis = refreshStartedAt;
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