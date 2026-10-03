#include <Arduino.h>
#include <SPI.h>

#include <GxEPD2_3C.h>

#include "config.h"

namespace {
constexpr uint32_t TIMER_DURATION_SECONDS = 8UL * 60UL * 60UL + 30UL * 60UL;
constexpr uint32_t SQUARE_DURATION_SECONDS = 30UL * 60UL;
constexpr uint32_t DISPLAY_UPDATE_INTERVAL_SECONDS = 5UL * 60UL;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint8_t TIMER_SQUARE_COUNT = 17;

enum class TimerState : uint8_t { Ready, Active, Finished };

GxEPD2_3C<GxEPD2_213c, 16> display(
    GxEPD2_213c(EPD_CS_PIN, EPD_DC_PIN, EPD_RST_PIN, EPD_BUSY_PIN));

TimerState timerState = TimerState::Ready;
uint32_t timerStartedAtMs = 0;
uint32_t lastDisplayedInterval = UINT32_MAX;
uint32_t buttonChangedAtMs = 0;
uint8_t lastButtonReading = HIGH;
uint8_t stableButtonState = HIGH;
bool displaySleeping = false;

void printTwoDigits(uint32_t value) {
  if (value < 10) {
    display.print('0');
  }
  display.print(value);
}

void drawTimerScreen(uint32_t remainingSeconds, TimerState state) {
  const bool active = state == TimerState::Active;
  const uint32_t remainingMinutes = (remainingSeconds + 59UL) / 60UL;
  const uint32_t hours = remainingMinutes / 60UL;
  const uint32_t minutes = remainingMinutes % 60UL;
  const uint32_t elapsedSeconds = TIMER_DURATION_SECONDS - remainingSeconds;
  const uint32_t progressPercent =
      (elapsedSeconds * 100UL + TIMER_DURATION_SECONDS / 2UL) /
      TIMER_DURATION_SECONDS;
  uint32_t expiredSquares = elapsedSeconds / SQUARE_DURATION_SECONDS;
  if (expiredSquares > TIMER_SQUARE_COUNT) {
    expiredSquares = TIMER_SQUARE_COUNT;
  }

  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(1);
    display.setCursor(8, 5);
    display.print(F("WORKING TIMER"));

    const int16_t statusTextX = active ? 174 : 168;
    const int16_t statusCircleX = active ? 167 : 161;
    display.fillCircle(statusCircleX, 9, 3, active ? GxEPD_RED : GxEPD_WHITE);
    display.drawCircle(statusCircleX, 9, 3, GxEPD_BLACK);
    display.setCursor(statusTextX, 5);
    if (state == TimerState::Active) {
      display.print(F("AKTIV"));
    } else if (state == TimerState::Ready) {
      display.print(F("BEREIT"));
    } else {
      display.print(F("FERTIG"));
    }
    display.drawFastHLine(8, 20, 196, GxEPD_BLACK);

    display.setTextSize(3);
    display.setCursor(8, 27);
    printTwoDigits(hours);
    display.print(':');
    printTwoDigits(minutes);

    display.setTextSize(1);
    for (uint8_t square = 0; square < TIMER_SQUARE_COUNT; ++square) {
      const int16_t x = 6 + square * 12;
      const uint16_t fillColor = square < expiredSquares ? GxEPD_WHITE : GxEPD_RED;
      display.fillRect(x + 1, 60, 7, 7, fillColor);
      display.drawRect(x, 59, 9, 9, GxEPD_BLACK);
    }

    display.setCursor(7, 70);
  display.print(F("Schon "));
  display.print(progressPercent);
  display.print(F("% geschafft"));
    display.drawFastHLine(7, 83, 198, GxEPD_BLACK);
    display.setCursor(7, 87);
    display.print(F("GESAMT 8,5 H"));
    display.setCursor(120, 87);
    display.print(F("JE FELD 30 MIN"));
  } while (display.nextPage());
}

void startTimer() {
  if (displaySleeping) {
    display.init(115200, true, 2, false);
    displaySleeping = false;
  }
  timerStartedAtMs = millis();
  timerState = TimerState::Active;
  lastDisplayedInterval = UINT32_MAX;
  Serial.println(F("Timer gestartet."));
}

void handleStartButton() {
  const uint8_t reading = digitalRead(TIMER_BUTTON_PIN);
  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonChangedAtMs = millis();
  }

  if ((millis() - buttonChangedAtMs) >= BUTTON_DEBOUNCE_MS &&
      reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW && timerState != TimerState::Active) {
      startTimer();
    }
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Working timer - Taster startet 8h30 countdown"));

  pinMode(TIMER_BUTTON_PIN, INPUT_PULLUP);
  lastButtonReading = digitalRead(TIMER_BUTTON_PIN);
  stableButtonState = lastButtonReading;
  buttonChangedAtMs = millis();

  display.init(115200, true, 2, false);
  drawTimerScreen(TIMER_DURATION_SECONDS, TimerState::Ready);
  lastDisplayedInterval = 0;
  Serial.println(F("Bereit. Taster an GPIO12 startet den Timer."));
}

void loop() {
  handleStartButton();

  uint32_t elapsedSeconds = 0;
  uint32_t remainingSeconds = TIMER_DURATION_SECONDS;
  if (timerState == TimerState::Active) {
    elapsedSeconds = (millis() - timerStartedAtMs) / 1000UL;
    if (elapsedSeconds >= TIMER_DURATION_SECONDS) {
      elapsedSeconds = TIMER_DURATION_SECONDS;
      remainingSeconds = 0;
      timerState = TimerState::Finished;
    } else {
      remainingSeconds = TIMER_DURATION_SECONDS - elapsedSeconds;
    }
  } else if (timerState == TimerState::Finished) {
    elapsedSeconds = TIMER_DURATION_SECONDS;
    remainingSeconds = 0;
  }

  const uint32_t updateInterval =
      elapsedSeconds / DISPLAY_UPDATE_INTERVAL_SECONDS;

  if (updateInterval != lastDisplayedInterval) {
    drawTimerScreen(remainingSeconds, timerState);
    lastDisplayedInterval = updateInterval;
    if (timerState == TimerState::Finished) {
      display.hibernate();
      displaySleeping = true;
      Serial.println(F("Timer beendet."));
    }
  }

  delay(20);
}