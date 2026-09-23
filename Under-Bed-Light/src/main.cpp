/*
 * Under Bed Light — ESP32-C3 mini
 * ============================================
 * 2x PIR (SR602)      -> motion left / right
 * 2x WS2812 5mm LED   -> light left / right (1 pixel each by default, the
 *                         count per side is adjustable in the web UI)
 * 1x Phototransistor  -> ambient brightness (analog)
 *
 * Behaviour
 * ---------
 * - Motion on a side, AND it is dark enough  -> that side's LED fades IN
 * - While motion continues                   -> LED stays at full brightness
 * - No motion for <timeout>                  -> LED fades OUT
 * - Motion returns while fading out          -> smoothly fades back IN from
 *                                                the current brightness
 * - Brightness, color, fade times, timeout, the ambient-light threshold and
 *   its confirmation time are all configurable from a small web UI (sliders)
 *   and persist in flash (NVS). Color is either a color-temperature slider
 *   (warm-to-neutral, extendable down into pure red) or a free-choice color
 *   picker for any hue - a "Farbmodus" switch on the Parameter page picks
 *   which one is active (see recomputeColor() below).
 * - Firmware can be updated from the same web UI (no USB cable needed
 *   after the first flash).
 *
 * Ambient-light reading is deliberately paranoid about being fooled by the
 * unit's own LEDs (or a second unit nearby): the raw sensor is only sampled
 * while both sides are fully off, and a dark/bright verdict only takes
 * effect once it has held steady for a configurable confirmation time (see
 * ambientIsDark() below for the full reasoning).
 *
 * Web UI is a small multi-page app styled to match an existing reference
 * dashboard (dark top bar, light cards, blue accent, stat grid, toggle
 * switches) - see buildPageHead()/PAGE_CSS below.
 *
 * Wiring (defaults below, change PIN_* if you wire it differently):
 *   PIR left     -> GPIO4   (3.3V logic out, SR602 works directly)
 *   PIR right    -> GPIO5
 *   LED left DIN -> GPIO6   (add ~330R series resistor)
 *   LED right DIN-> GPIO3   (add ~330R series resistor)
 *   Phototransistor: confirmed on this build to be wired with the divider
 *     "inverted" (brighter ambient light => LOWER ADC reading, not higher -
 *     see ambientIsDark() and the luxThreshold default, both account for
 *     this already). If you wire a new unit the "normal" way round instead
 *     (collector -> 3V3, emitter -> GPIO1 + 10k resistor to GND, brighter
 *     => higher ADC), flip the comparison in ambientIsDark() back to "<"
 *     and reset luxThreshold to a low raw value again.
 *
 * NOTE ON GPIO7: on some ESP32-C3 "mini" boards (confirmed on the Carenuity
 * C3-Mini) GPIO7 is hard-wired to the board's own onboard WS2812 RGB LED.
 * Using it for an external LED just drives that onboard LED instead of your
 * wiring - that's why PIN_LED_RIGHT is GPIO3 here, not GPIO7. If you're on a
 * different board and unsure which pin (if any) your onboard LED uses, the
 * gpio_ws2812_scan/ sketch in this project can help you find it.
 *
 * All GPIOs above are otherwise non-strapping pins on the ESP32-C3 and free
 * on most "mini"/"super mini" board layouts - but as this example shows,
 * always double check against your specific board before final wiring.
 *
 * ---------------------------------------------------------------------
 * Arduino IDE setup (one-time)
 * ---------------------------------------------------------------------
 * 1. Boards Manager: File > Preferences > "Additional boards manager URLs"
 *    add:  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *    then Tools > Board > Boards Manager, install "esp32" (Espressif Systems).
 * 2. Library Manager: Sketch > Include Library > Manage Libraries,
 *    install "Adafruit NeoPixel". (WiFi/WebServer/Update/Preferences/
 *    DNSServer/ESPmDNS ship with the esp32 board package, no separate
 *    install needed.)
 * 3. Tools menu board settings:
 *      Board:          "ESP32C3 Dev Module"
 *      USB CDC On Boot: "Enabled"   <- needed on boards with only a native
 *                                      USB port (no separate USB-UART chip,
 *                                      e.g. most "Super Mini" boards).
 *                                      If your board has a CP2102/CH340
 *                                      chip, this can be "Disabled" instead.
 *      Flash Size, Partition Scheme, Upload Speed: defaults are fine.
 * 4. Select the correct Port, then Sketch > Upload.
 * 5. This whole folder must be named "UnderBedLight" (matching this .ino
 *    file) for the Arduino IDE to open it as a sketch - keep it that way
 *    if you copy/rename it.
 */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Adafruit_NeoPixel.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Pin configuration
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_PIR_FRONT = 4;
constexpr uint8_t PIN_PIR_SIDE  = 5;
constexpr uint8_t PIN_LED_FRONT = 6;
constexpr uint8_t PIN_LED_SIDE  = 3;   // NOT GPIO7 - that's the onboard WS2812
                                        // RGB LED on the Carenuity C3-Mini!
constexpr uint8_t PIN_PHOTO     = 1;   // ADC1_CH1

// Upper bound for how many WS2812 pixels can be chained per side via the
// web UI. Keep this sane - each pixel needs ~60mA at full white, so 60 LEDs
// on one side is already up to 3.6A and needs a properly sized 5V supply
// (not the ESP32's own 3.3V regulator either way).
constexpr uint8_t MAX_LEDS_PER_SIDE = 60;

constexpr const char* DEVICE_NAME = "Under Bed Light";

// Fallback AP used when no working WiFi is configured yet.
constexpr const char* AP_SSID = "BedLight-Setup";
constexpr const char* AP_PASS = "bedlight123";   // >= 8 chars required by WiFi
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

// NTP time (for real date/time stamps in the debug log, see logMotion()
// below). Requested via configTzTime() once WiFi is up (see connectWiFi());
// sync happens in the background, usually within a few seconds if the
// network has internet access. TZ_INFO is a POSIX TZ string for Germany
// (Central European Time incl. automatic CET/CEST daylight-saving switch) -
// change it if the device runs in a different timezone.
constexpr const char* NTP_SERVER = "pool.ntp.org";
constexpr const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// ---------------------------------------------------------------------------
// Persistent settings (all in "device" units - conversion to/from the UI's
// percent/second sliders happens in the web handlers only)
// ---------------------------------------------------------------------------
struct Settings {
  uint8_t  brightness   = 200;   // 0-255, target max brightness
  uint16_t colorKelvin   = 2300;  // 300-6500K; below ~500K the formula saturates
                                    // to pure red, up to neutral white at 6500K
  uint8_t  colorMode     = 0;     // 0 = Farbtemperatur (colorKelvin), 1 = eigene Volltonfarbe
                                   // (customR/G/B) - see recomputeColor()
  uint8_t  customR       = 0;     // eigene Farbe, nur genutzt wenn colorMode == 1
  uint8_t  customG       = 120;
  uint8_t  customB       = 255;
  uint16_t fadeInMs      = 800;   // ms for 0 -> brightness
  uint16_t fadeOutMs     = 3000;  // ms for brightness -> 0
  uint32_t timeoutMs     = 15000; // ms after last motion before fade-out starts (up to 3600s)
  uint16_t luxThreshold  = 3495;  // raw ADC 0-4095; the divider on this build is wired
                                   // "inverted" (more light -> lower ADC), so higher-than-this
                                   // => dark enough to allow turning on (see ambientIsDark())
  uint32_t ambientConfirmMs = 15000; // how long a dark<->bright change must hold before
                                      // it counts (debounce against self/cross-device
                                      // LED interference, see ambientIsDark())
  uint8_t  numLedsFront  = 1;     // pixels chained on the front data line (1-MAX_LEDS_PER_SIDE)
  uint8_t  numLedsSide   = 1;     // pixels chained on the side data line
  bool     testMode = false;     // forces both LEDs on for wiring tests
  char     deviceLabel[24] = ""; // optional suffix shown after DEVICE_NAME, e.g. "vorne";
                                   // also sanitized into the mDNS hostname, see mdnsHostname()
  bool     debugLogEnabled = false; // motion+brightness log (serial + web), see logMotion()
  uint8_t  logMaxEntries   = 20;    // ring-buffer cap, 1..MAX_LOG_ENTRIES
  char     wifiSsid[33] = "MyBox";
  char     wifiPass[65] = "Ak29#00157";
};

Settings settings;
Preferences prefs;

WebServer server(80);
DNSServer dnsServer;
bool apMode = false;

Adafruit_NeoPixel ledFront(1, PIN_LED_FRONT, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel ledSide(1, PIN_LED_SIDE, NEO_RGB + NEO_KHZ800);

// Cached RGB for the current colorKelvin, recomputed only when it changes.
uint8_t cachedColorR = 255, cachedColorG = 180, cachedColorB = 80;

// ---------------------------------------------------------------------------
// Small integer helpers (avoid pulling in <math.h> round() ambiguity)
// ---------------------------------------------------------------------------
static inline long roundDiv(long num, long den) {
  return (num + den / 2) / den;
}
uint8_t pctToByte(int pct)  { return (uint8_t)constrain(roundDiv((long)pct * 255, 100), 0, 255); }
int     byteToPct(uint8_t b){ return (int)constrain(roundDiv((long)b * 100, 255), 0, 100); }
uint16_t pctToRaw(int pct)  { return (uint16_t)constrain(roundDiv((long)pct * 4095, 100), 0, 4095); }
int      rawToPct(uint16_t r){ return (int)constrain(roundDiv((long)r * 100, 4095), 0, 100); }

// ---------------------------------------------------------------------------
// Kelvin -> RGB (Tanner Helland's approximation). Valid well beyond our
// 2000-6500K slider range, so no extra branching needed here.
// ---------------------------------------------------------------------------
void kelvinToRGB(uint16_t kelvin, uint8_t &r, uint8_t &g, uint8_t &b) {
  float temp = kelvin / 100.0f;
  float red, green, blue;

  if (temp <= 66) {
    red = 255;
  } else {
    red = temp - 60;
    red = 329.698727446f * powf(red, -0.1332047592f);
  }

  if (temp <= 66) {
    green = 99.4708025861f * logf(temp) - 161.1195681661f;
  } else {
    green = temp - 60;
    green = 288.1221695283f * powf(green, -0.0755148492f);
  }

  if (temp >= 66) {
    blue = 255;
  } else if (temp <= 19) {
    blue = 0;
  } else {
    blue = temp - 10;
    blue = 138.5177312231f * logf(blue) - 305.0447927307f;
  }

  r = (uint8_t)constrain(red, 0, 255);
  g = (uint8_t)constrain(green, 0, 255);
  b = (uint8_t)constrain(blue, 0, 255);
}

void recomputeColor() {
  if (settings.colorMode == 1) {
    cachedColorR = settings.customR;
    cachedColorG = settings.customG;
    cachedColorB = settings.customB;
  } else {
    kelvinToRGB(settings.colorKelvin, cachedColorR, cachedColorG, cachedColorB);
  }
}

// ---------------------------------------------------------------------------
// Device name: DEVICE_NAME plus an optional user-editable label appended,
// e.g. "Under Bed Light vorne" - lets two units on the same network be told
// apart on the status page/browser tab title. mdnsHostname() derives a
// matching, sanitized *.local hostname from the same label so two devices
// don't collide on http://bedlight.local/ (ASCII letters/digits only, spaces
// collapse to a single "-", everything else - umlauts, punctuation - is
// simply dropped).
// ---------------------------------------------------------------------------
String displayName() {
  String n = DEVICE_NAME;
  if (strlen(settings.deviceLabel) > 0) {
    n += " ";
    n += settings.deviceLabel;
  }
  return n;
}

String mdnsHostname() {
  String label = settings.deviceLabel;
  label.toLowerCase();
  String suffix;
  for (size_t i = 0; i < label.length(); i++) {
    char c = label.charAt(i);
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      suffix += c;
    } else if ((c == ' ' || c == '-' || c == '_') &&
               suffix.length() > 0 && suffix.charAt(suffix.length() - 1) != '-') {
      suffix += '-';
    }
  }
  while (suffix.length() > 0 && suffix.charAt(suffix.length() - 1) == '-') {
    suffix.remove(suffix.length() - 1);
  }
  String h = "bedlight";
  if (suffix.length() > 0) {
    h += "-";
    h += suffix;
  }
  return h;
}

// ---------------------------------------------------------------------------
// Debug-Log: ring buffer of motion+brightness events (which PIR fired, and
// the brightness at that moment), optionally mirrored to Serial. Off by
// default (settings.debugLogEnabled) so normal operation stays quiet.
// Capped at settings.logMaxEntries (1..MAX_LOG_ENTRIES, configurable from the
// web UI); changing the cap simply clears the log rather than reshuffling a
// half-resized ring buffer.
//
// Defined here, before LightState/SideController, because
// SideController::update() calls logMotion() directly and a free function
// used inside an inline member function must already be declared above it.
// ---------------------------------------------------------------------------
constexpr uint8_t MAX_LOG_ENTRIES = 100;

// Formats a Unix timestamp as "YYYY-MM-DD HH:MM:SS" in local time (see
// TZ_INFO/NTP_SERVER above). Returns a placeholder instead of a bogus date
// if NTP hasn't synced yet (e.g. right after boot, or while running in
// AP/setup mode without internet access) - recognized by the timestamp
// still being implausibly small (long before this project existed).
String formatTimestamp(time_t ts) {
  if (ts < 1600000000) return "n/a (Zeit noch nicht synchronisiert)";
  struct tm tmInfo;
  localtime_r(&ts, &tmInfo);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmInfo);
  return String(buf);
}

struct LogEntry {
  time_t ts;          // Unix-Zeitstempel (Sekunden) des Ereignisses; klein/0, falls beim
                       // Loggen noch keine NTP-Zeit synchronisiert war (siehe formatTimestamp())
  char side[8];       // "vorne" / "seite"
  int brightnessPct;  // 0-100, brightness at the moment motion was detected
};

LogEntry logBuffer[MAX_LOG_ENTRIES];
uint8_t  logHead  = 0; // index the next entry will be written to
uint8_t  logCount = 0; // number of valid entries currently stored

void logClear() {
  logHead = 0;
  logCount = 0;
}

// NOTE: deliberately no standalone "LogEntry& logGet(i)" helper here - the
// Arduino IDE (unlike PlatformIO) auto-generates a forward prototype for
// every function in the sketch and inserts it near the very top of the
// file, before struct LogEntry is defined below. A function whose
// signature mentions LogEntry as a parameter/return type then fails to
// compile there ("'LogEntry' does not name a type") even though the
// function's own definition is perfectly valid at its actual position.
// Local variables of type LogEntry inside a function body are unaffected
// (see handleLogApi() below), so the index math is simply inlined there
// instead of living in its own function.

void logMotion(const char* side, int brightnessPct) {
  if (!settings.debugLogEnabled) return;
  uint8_t cap = constrain(settings.logMaxEntries, (uint8_t)1, MAX_LOG_ENTRIES);
  LogEntry &e = logBuffer[logHead];
  e.ts = time(nullptr);
  strncpy(e.side, side, sizeof(e.side) - 1);
  e.side[sizeof(e.side) - 1] = '\0';
  e.brightnessPct = brightnessPct;
  logHead = (uint8_t)((logHead + 1) % cap);
  if (logCount < cap) logCount++;
  Serial.printf("[LOG] %s Bewegung %s, Helligkeit %d%%\n", formatTimestamp(e.ts).c_str(), side, brightnessPct);
}

// ---------------------------------------------------------------------------
// Ambient light - raw smoothed value only here. ambientBegin()/ambientIsDark()
// are defined further down, after the SideController/frontCtrl/sideCtrl
// declarations, because they need to know each side's LightState (see the
// comment there for why).
// ---------------------------------------------------------------------------
float ambientFiltered = 0;

// ---------------------------------------------------------------------------
// Per-side fade state machine
// ---------------------------------------------------------------------------
enum class LightState : uint8_t { IDLE, FADE_IN, ON, FADE_OUT };

struct SideController {
  const char* name;
  uint8_t pirPin;
  Adafruit_NeoPixel* led;
  const char* logLabel; // "vorne" / "seite", used only for the debug log

  LightState state = LightState::IDLE;
  uint8_t currentBrightness = 0;
  uint8_t fadeStartBrightness = 0;
  unsigned long fadeStartMs = 0;
  unsigned long lastMotionMs = 0;
  bool motionActive = false; // exposed on the status page
  uint8_t numLeds = 1;       // how many chained pixels this side currently drives

  void begin(uint8_t initialNumLeds) {
    pinMode(pirPin, INPUT);
    led->begin();
    led->setBrightness(255);
    setNumLeds(initialNumLeds);
    led->show();
  }

  // Resizes the pixel buffer (Adafruit_NeoPixel::updateLength mallocs a new
  // buffer under the hood) - safe to call any time, including live from the
  // web UI, not just at boot.
  void setNumLeds(uint8_t n) {
    n = constrain(n, 1, MAX_LEDS_PER_SIDE);
    led->updateLength(n);
    numLeds = n;
  }

  void startFadeIn(unsigned long now) {
    fadeStartMs = now;
    fadeStartBrightness = currentBrightness;
    state = LightState::FADE_IN;
  }

  void update(bool ambientDark, unsigned long now) {
    bool nowActive = digitalRead(pirPin) == HIGH;
    if (nowActive && !motionActive) {
      // Rising edge - log the brightness this side was at *when* motion was
      // (re)detected, before this call's fade logic changes it further.
      logMotion(logLabel, byteToPct(currentBrightness));
    }
    motionActive = nowActive;

    if (motionActive) {
      lastMotionMs = now;
      if (state == LightState::IDLE && ambientDark) {
        startFadeIn(now);
      } else if (state == LightState::FADE_OUT) {
        startFadeIn(now); // resume smoothly from current brightness
      }
    }

    switch (state) {
      case LightState::IDLE:
        currentBrightness = 0;
        break;

      case LightState::FADE_IN: {
        unsigned long elapsed = now - fadeStartMs;
        if (settings.fadeInMs == 0 || elapsed >= settings.fadeInMs) {
          currentBrightness = settings.brightness;
          state = LightState::ON;
        } else {
          float t = (float)elapsed / (float)settings.fadeInMs;
          int delta = (int)settings.brightness - (int)fadeStartBrightness;
          currentBrightness = fadeStartBrightness + (int)(delta * t);
        }
        break;
      }

      case LightState::ON:
        currentBrightness = settings.brightness;
        if (now - lastMotionMs >= settings.timeoutMs) {
          fadeStartMs = now;
          fadeStartBrightness = currentBrightness;
          state = LightState::FADE_OUT;
        }
        break;

      case LightState::FADE_OUT: {
        unsigned long elapsed = now - fadeStartMs;
        if (settings.fadeOutMs == 0 || elapsed >= settings.fadeOutMs) {
          currentBrightness = 0;
          state = LightState::IDLE;
        } else {
          float t = (float)elapsed / (float)settings.fadeOutMs;
          currentBrightness = fadeStartBrightness - (int)(fadeStartBrightness * t);
        }
        break;
      }
    }

    render();
  }

  void render() {
    // Test mode shows exactly what the light will look like when it's on
    // (current brightness + color setting), not a hardcoded full blast -
    // that way it also doubles as a way to check the brightness slider.
    uint8_t b = settings.testMode ? settings.brightness : currentBrightness;
    uint8_t r  = (uint16_t)cachedColorR * b / 255;
    uint8_t g  = (uint16_t)cachedColorG * b / 255;
    uint8_t bl = (uint16_t)cachedColorB * b / 255;
    uint32_t c = led->Color(r, g, bl);
    for (uint8_t i = 0; i < numLeds; i++) {
      led->setPixelColor(i, c);
    }
    led->show();
  }

  const char* stateName() const {
    switch (state) {
      case LightState::IDLE: return "aus";
      case LightState::FADE_IN: return "faellt_ein";
      case LightState::ON: return "an";
      case LightState::FADE_OUT: return "faellt_aus";
    }
    return "?";
  }
};

SideController frontCtrl{"front", PIN_PIR_FRONT, &ledFront, "vorne"};
SideController sideCtrl{"side", PIN_PIR_SIDE, &ledSide, "seite"};

// ---------------------------------------------------------------------------
// Ambient light: smoothed + debounced against our own LEDs and short-lived
// interference (a second unit's LED cycle, someone briefly flicking a
// switch).
//
//   1. The raw ADC value only feeds the smoothing filter while BOTH sides
//      are fully IDLE. Whenever either LED is lit (fading in, on, or fading
//      out) the reading is left untouched - it simply cannot be polluted by
//      light our own LEDs never let it see in the first place.
//   2. The filter itself is time-based (not tied to how fast loop() spins)
//      with a short ~2s time constant, just to smooth ADC noise.
//   3. The smoothed value's dark/bright verdict only becomes the value
//      actually used to trigger a fade-in after it has stayed on one side
//      of settings.luxThreshold continuously for
//      settings.ambientConfirmMs. Short bursts (shorter than that) are
//      ignored completely, however strong they are; a real, sustained
//      change (room light switched off for the night) still gets through
//      within a bounded, known delay.
// ---------------------------------------------------------------------------
bool  ambientCandidateDark = false;  // instantaneous verdict on the smoothed value
bool  ambientConfirmedDark = false;  // debounced verdict actually used to trigger
unsigned long ambientCandidateSinceMs = 0;
unsigned long ambientLastSampleMs = 0;

bool anyLedOn() {
  return frontCtrl.state != LightState::IDLE || sideCtrl.state != LightState::IDLE;
}

void ambientBegin() {
  ambientFiltered = analogRead(PIN_PHOTO);
  ambientCandidateDark = ambientFiltered > settings.luxThreshold;
  ambientConfirmedDark = ambientCandidateDark; // trust the very first reading right away
  ambientCandidateSinceMs = millis();
  ambientLastSampleMs = millis();
}

// Call every loop().
bool ambientIsDark() {
  unsigned long now = millis();

  if (!anyLedOn()) {
    // Time-based exponential smoothing (tau ~2s) so the behaviour doesn't
    // depend on how many times per second loop() happens to run.
    unsigned long dt = now - ambientLastSampleMs;
    int raw = analogRead(PIN_PHOTO);
    // Wiring (confirmed on this build): the phototransistor's voltage
    // divider is wired the "inverted" way round - more ambient light means
    // a LOWER ADC reading, not higher. The comparison below (and the
    // luxThreshold default) already account for that. If you rewire it the
    // other way round (collector -> 3V3, emitter -> ADC pin + pulldown to
    // GND, so brighter -> higher ADC), flip both back: candidate below to
    // "<" and the luxThreshold default to a LOW raw value again.
    float alpha = 1.0f - expf(-(float)dt / 2000.0f);
    ambientFiltered += (raw - ambientFiltered) * alpha;
  }
  // else: one of our own LEDs is lit right now - deliberately skip the
  // sample instead of letting its light corrupt the reading. We just keep
  // evaluating against the last trusted value from before it turned on.
  ambientLastSampleMs = now;

  bool candidate = ambientFiltered > settings.luxThreshold;
  if (candidate != ambientCandidateDark) {
    ambientCandidateDark = candidate;
    ambientCandidateSinceMs = now;
  }
  if (now - ambientCandidateSinceMs >= settings.ambientConfirmMs) {
    ambientConfirmedDark = ambientCandidateDark;
  }
  return ambientConfirmedDark;
}

// ---------------------------------------------------------------------------
// Settings persistence (NVS via Preferences)
// ---------------------------------------------------------------------------
void loadSettings() {
  prefs.begin("bedlight", true);
  settings.brightness   = prefs.getUChar("bright", settings.brightness);
  settings.colorKelvin  = prefs.getUShort("kelvin", settings.colorKelvin);
  settings.colorMode    = prefs.getUChar("colorMode", settings.colorMode);
  settings.customR      = prefs.getUChar("custR", settings.customR);
  settings.customG      = prefs.getUChar("custG", settings.customG);
  settings.customB      = prefs.getUChar("custB", settings.customB);
  settings.fadeInMs     = prefs.getUShort("fadeIn", settings.fadeInMs);
  settings.fadeOutMs    = prefs.getUShort("fadeOut", settings.fadeOutMs);
  settings.timeoutMs    = prefs.getUInt("timeout", settings.timeoutMs);
  settings.luxThreshold = prefs.getUShort("lux", settings.luxThreshold);
  settings.ambientConfirmMs = prefs.getUInt("ambConf", settings.ambientConfirmMs);
  settings.numLedsFront = prefs.getUChar("numF", settings.numLedsFront);
  settings.numLedsSide  = prefs.getUChar("numS", settings.numLedsSide);
  settings.testMode     = prefs.getBool("testMode", settings.testMode);
  String label = prefs.getString("label", "");
  label.toCharArray(settings.deviceLabel, sizeof(settings.deviceLabel));
  settings.debugLogEnabled = prefs.getBool("dbgLog", settings.debugLogEnabled);
  settings.logMaxEntries   = constrain(prefs.getUChar("logMax", settings.logMaxEntries), (uint8_t)1, MAX_LOG_ENTRIES);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  ssid.toCharArray(settings.wifiSsid, sizeof(settings.wifiSsid));
  pass.toCharArray(settings.wifiPass, sizeof(settings.wifiPass));
  prefs.end();
  recomputeColor();
}

void saveSettings() {
  prefs.begin("bedlight", false);
  prefs.putUChar("bright", settings.brightness);
  prefs.putUShort("kelvin", settings.colorKelvin);
  prefs.putUChar("colorMode", settings.colorMode);
  prefs.putUChar("custR", settings.customR);
  prefs.putUChar("custG", settings.customG);
  prefs.putUChar("custB", settings.customB);
  prefs.putUShort("fadeIn", settings.fadeInMs);
  prefs.putUShort("fadeOut", settings.fadeOutMs);
  prefs.putUInt("timeout", settings.timeoutMs);
  prefs.putUShort("lux", settings.luxThreshold);
  prefs.putUInt("ambConf", settings.ambientConfirmMs);
  prefs.putUChar("numF", settings.numLedsFront);
  prefs.putUChar("numS", settings.numLedsSide);
  prefs.putBool("testMode", settings.testMode);
  prefs.putString("label", settings.deviceLabel);
  prefs.putBool("dbgLog", settings.debugLogEnabled);
  prefs.putUChar("logMax", settings.logMaxEntries);
  prefs.putString("ssid", settings.wifiSsid);
  prefs.putString("pass", settings.wifiPass);
  prefs.end();
  recomputeColor();
}

// ---------------------------------------------------------------------------
// Web UI - shared look & feel (dark top bar, light cards, blue accent,
// stat grid, toggle switches, range sliders) matching the reference
// dashboard the parameters were modeled after.
// ---------------------------------------------------------------------------
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
.swatch{width:28px;height:28px;border-radius:50%;border:2px solid var(--border);flex:none}
)CSS";

String pageHead(const char* activePage) {
  String h;
  h.reserve(1600);
  h += F("<!DOCTYPE html><html lang='de'><head><meta charset='UTF-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>"); h += displayName(); h += F("</title><style>");
  h += FPSTR(PAGE_CSS);
  h += F("</style></head><body><div class='topbar'><div class='brand'>&#128161; ");
  h += displayName();
  h += F("</div><nav>");

  struct NavItem { const char* href; const char* label; const char* key; };
  static const NavItem items[] = {
    {"/", "Status", "status"},
    {"/params", "Parameter", "params"},
    {"/wifi", "WLAN", "wifi"},
    {"/update", "Firmware", "update"},
  };
  for (auto &it : items) {
    h += F("<a href='"); h += it.href; h += F("'");
    if (strcmp(activePage, it.key) == 0) h += F(" class='active'");
    h += F(">"); h += it.label; h += F("</a>");
  }
  h += F("</nav></div><div class='wrap'>");
  return h;
}

String pageFoot() {
  return F("</div></body></html>");
}

// ---------------------------------------------------------------------------
// Status page
// ---------------------------------------------------------------------------
void handleRoot() {
  String h = pageHead("status");

  h += F("<div class='card'><h2>Status <span id='st-pill' class='pill ok'>...</span></h2>"
         "<div class='statgrid'>"
         "<div class='stat'><div class='k'>Bewegung vorne</div><div class='v' id='st-mF'>-</div></div>"
         "<div class='stat'><div class='k'>Bewegung Seite</div><div class='v' id='st-mS'>-</div></div>"
         "<div class='stat'><div class='k'>Licht vorne</div><div class='v' id='st-sF'>-</div></div>"
         "<div class='stat'><div class='k'>Licht Seite</div><div class='v' id='st-sS'>-</div></div>"
         "<div class='stat'><div class='k'>Helligkeit vorne</div><div class='v' id='st-bF'>-</div></div>"
         "<div class='stat'><div class='k'>Helligkeit Seite</div><div class='v' id='st-bS'>-</div></div>"
         "<div class='stat'><div class='k'>Umgebung</div><div class='v' id='st-amb'>-</div></div>"
         "<div class='stat'><div class='k'>IP-Adresse</div><div class='v' id='st-ip'>-</div></div>"
         "</div></div>");

  h += F("<div class='card'><h2>Testmodus</h2>"
         "<div class='switchrow'><span class='hint'>Beide LEDs dauerhaft an (Verkabelung pr&uuml;fen)</span>"
         "<label class='switch'><input type='checkbox' id='testToggle' onchange='setTest(this.checked)'><span class='slider2'></span></label>"
         "</div></div>");

  h += F("<div class='card'><h2>Aktionen</h2><div class='actionbar'>"
         "<button class='btn secondary small' onclick=\"if(confirm('Ger&auml;t jetzt neu starten?'))post('/action/reboot')\">Neu starten</button>"
         "</div></div>");

  h += F("<div class='card'><h2>Debug-Log</h2>"
         "<div class='switchrow'><span class='hint'>Bewegung (welcher Sensor) &amp; Helligkeit protokollieren, seriell und hier</span>"
         "<label class='switch'><input type='checkbox' id='logToggle' onchange='setDebugLog(this.checked)'><span class='slider2'></span></label>"
         "</div>");
  h += F("<div class='row' style='margin-top:14px'><div class='rowhead'><label>Max. Eintr&auml;ge</label><span class='val' id='v_logmax'>");
  h += settings.logMaxEntries;
  h += F("</span></div><input type='range' id='logMaxRange' min='1' max='");
  h += MAX_LOG_ENTRIES;
  h += F("' step='1' value='");
  h += settings.logMaxEntries;
  h += F("' oninput=\"document.getElementById('v_logmax').textContent=this.value\" onchange='saveLogMax(this.value)'>"
         "<div class='hint'>&Auml;ndern leert das Log (max. "); h += MAX_LOG_ENTRIES; h += F(")</div></div>");
  h += F("<div class='actionbar'><button class='btn secondary small' onclick='clearLog()'>Log leeren</button></div>"
         "<div id='logView' style='margin-top:12px;max-height:220px;overflow-y:auto;font-family:monospace;"
         "font-size:12.5px;background:#f8fafc;border:1px solid var(--border);border-radius:8px;padding:8px 10px'>-</div>"
         "</div>");

  h += F("<script>"
         "function post(u){fetch(u,{method:'POST'}).then(function(){setTimeout(refresh,300);});}"
         "function setTest(v){fetch('/action/testmode?v='+(v?1:0),{method:'POST'});}"
         "function stTxt(s){return s=='an'?'An':s=='faellt_ein'?'Wird heller':s=='faellt_aus'?'Wird dunkler':'Aus';}"
         "function refresh(){fetch('/api/status').then(function(r){return r.json();}).then(function(d){"
         "document.getElementById('st-mF').textContent=d.front.motion?'Ja':'Nein';"
         "document.getElementById('st-mS').textContent=d.side.motion?'Ja':'Nein';"
         "document.getElementById('st-sF').textContent=stTxt(d.front.state);"
         "document.getElementById('st-sS').textContent=stTxt(d.side.state);"
         "document.getElementById('st-bF').textContent=Math.round(d.front.brightness*100/255)+'%';"
         "document.getElementById('st-bS').textContent=Math.round(d.side.brightness*100/255)+'%';"
         "document.getElementById('st-amb').textContent=d.ambientPct+'% ('+(d.dark?'dunkel':'hell')+')';"
         "document.getElementById('st-ip').textContent=d.ip;"
         "var active=d.front.state!='aus'||d.side.state!='aus';"
         "var p=document.getElementById('st-pill');p.className='pill '+(active?'warn':'ok');p.textContent=active?'Aktiv':'Bereit';"
         "document.getElementById('testToggle').checked=d.testMode;"
         "}).catch(function(){});}"
         "function setDebugLog(v){fetch('/action/debuglog?v='+(v?1:0),{method:'POST'}).then(refreshLog);}"
         "function saveLogMax(v){fetch('/action/logmax?v='+v,{method:'POST'}).then(refreshLog);}"
         "function clearLog(){fetch('/action/logclear',{method:'POST'}).then(refreshLog);}"
         "function refreshLog(){fetch('/api/log').then(function(r){return r.json();}).then(function(d){"
         "document.getElementById('logToggle').checked=d.enabled;"
         "document.getElementById('logMaxRange').value=d.maxEntries;"
         "document.getElementById('v_logmax').textContent=d.maxEntries;"
         "var v=document.getElementById('logView');"
         "if(!d.entries.length){v.textContent=d.enabled?'(noch keine Eintr\\u00e4ge)':'(Log ist ausgeschaltet)';return;}"
         "v.innerHTML=d.entries.map(function(e){return '<div>'+e.timestamp+' \\u2014 '+e.side+', '+e.brightnessPct+'%</div>';}).join('');"
         "}).catch(function(){});}"
         "setInterval(refresh,2000);window.addEventListener('load',refresh);"
         "setInterval(refreshLog,3000);window.addEventListener('load',refreshLog);"
         "</script>");

  h += pageFoot();
  server.send(200, "text/html", h);
}

void handleActionTestMode() {
  if (server.hasArg("v")) {
    settings.testMode = server.arg("v") == "1";
    saveSettings();
  }
  server.send(200, "text/plain", "OK");
}

void handleActionReboot() {
  server.send(200, "text/plain", "Neustart...");
  delay(300);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// Parameter page (sliders)
// ---------------------------------------------------------------------------
const char KELVIN_JS[] PROGMEM = R"JS(
function kelvinToCss(k){
  var temp=k/100, r,g,b;
  if(temp<=66){r=255;}else{r=temp-60;r=329.698727446*Math.pow(r,-0.1332047592);}
  if(temp<=66){g=99.4708025861*Math.log(temp)-161.1195681661;}else{g=temp-60;g=288.1221695283*Math.pow(g,-0.0755148492);}
  if(temp>=66){b=255;}else if(temp<=19){b=0;}else{b=temp-10;b=138.5177312231*Math.log(b)-305.0447927307;}
  r=Math.max(0,Math.min(255,r));g=Math.max(0,Math.min(255,g));b=Math.max(0,Math.min(255,b));
  return 'rgb('+Math.round(r)+','+Math.round(g)+','+Math.round(b)+')';
}
function updateKelvin(v){
  document.getElementById('v_kelvin').textContent=v+' K';
  document.getElementById('swatch').style.background=kelvinToCss(v);
}
function setColorMode(v){
  document.getElementById('row_kelvin').hidden = (v != 0);
  document.getElementById('row_custom').hidden = (v != 1);
}
window.addEventListener('load',function(){
  updateKelvin(document.querySelector("input[name=kelvin]").value);
});
)JS";

void handleParams() {
  String h = pageHead("params");

  int brightnessPct = byteToPct(settings.brightness);
  int timeoutSec = settings.timeoutMs / 1000;
  int luxPct = rawToPct(settings.luxThreshold);
  int ambientConfirmSec = settings.ambientConfirmMs / 1000;

  h += F("<form method='POST' action='/save'>");

  h += F("<div class='card'><h2>Licht</h2>");

  h += F("<div class='row'><div class='rowhead'><label>Helligkeit</label><span class='val' id='v_brightness'>");
  h += brightnessPct; h += F("%</span></div>"
         "<input type='range' name='brightness' min='5' max='100' step='1' value='");
  h += brightnessPct;
  h += F("' oninput=\"document.getElementById('v_brightness').textContent=this.value+'%'\"></div>");

  h += F("<div class='row'><label>Farbmodus</label>"
         "<div style='display:flex;gap:18px;margin-top:2px'>"
         "<label style='display:flex;align-items:center;gap:6px;font-size:13px;font-weight:400;color:var(--text)'>"
         "<input type='radio' name='colorMode' value='0'");
  if (settings.colorMode == 0) h += F(" checked");
  h += F(" onchange='setColorMode(0)'> Farbtemperatur</label>"
         "<label style='display:flex;align-items:center;gap:6px;font-size:13px;font-weight:400;color:var(--text)'>"
         "<input type='radio' name='colorMode' value='1'");
  if (settings.colorMode == 1) h += F(" checked");
  h += F(" onchange='setColorMode(1)'> Eigene Farbe</label>"
         "</div></div>");

  h += F("<div class='row' id='row_kelvin'");
  if (settings.colorMode != 0) h += F(" hidden");
  h += F("><div class='rowhead'><label>Farbtemperatur</label><span class='val' id='v_kelvin'>");
  h += settings.colorKelvin; h += F(" K</span></div>"
         "<div style='display:flex;align-items:center;gap:10px'>"
         "<input type='range' name='kelvin' min='300' max='6500' step='50' value='");
  h += settings.colorKelvin;
  h += F("' style='flex:1' oninput='updateKelvin(this.value)'>"
         "<div id='swatch' class='swatch'></div></div>"
         "<div class='hint'>ganz links = reines Rot, Mitte = warmes Kerzenlicht, rechts = neutralwei&szlig;</div></div>");

  h += F("<div class='row' id='row_custom'");
  if (settings.colorMode != 1) h += F(" hidden");
  h += F("><div class='rowhead'><label>Eigene Farbe</label></div>"
         "<input type='color' name='customColor' value='#");
  char customHex[7];
  snprintf(customHex, sizeof(customHex), "%02X%02X%02X", settings.customR, settings.customG, settings.customB);
  h += customHex;
  h += F("' style='width:100%;height:42px;border:1px solid var(--border);border-radius:6px;padding:2px;background:#fff'>"
         "<div class='hint'>freie Farbwahl unabh&auml;ngig von der Farbtemperatur, z.B. Blau, Gr&uuml;n, Pink</div></div>");

  h += F("<div class='subcard'><h3>Anzahl LEDs je Seite</h3>");

  h += F("<div class='row'><div class='rowhead'><label>Vorne</label><span class='val' id='v_numFront'>");
  h += settings.numLedsFront; h += F("</span></div>"
         "<input type='range' name='numLedsFront' min='1' max='");
  h += MAX_LEDS_PER_SIDE;
  h += F("' step='1' value='");
  h += settings.numLedsFront;
  h += F("' oninput=\"document.getElementById('v_numFront').textContent=this.value\"></div>");

  h += F("<div class='row'><div class='rowhead'><label>Seite</label><span class='val' id='v_numSide'>");
  h += settings.numLedsSide; h += F("</span></div>"
         "<input type='range' name='numLedsSide' min='1' max='");
  h += MAX_LEDS_PER_SIDE;
  h += F("' step='1' value='");
  h += settings.numLedsSide;
  h += F("' oninput=\"document.getElementById('v_numSide').textContent=this.value\">"
         "<div class='hint'>wie viele WS2812 auf der Datenleitung dieser Seite in Reihe h&auml;ngen — "
         "mehr LEDs brauchen entsprechend mehr 5V-Strom (eigene Stromversorgung!)</div></div>");

  h += F("</div>"); // end subcard LED-Anzahl

  h += F("</div>"); // end card Licht

  h += F("<div class='card'><h2>Bewegung &amp; Timing</h2>");

  h += F("<div class='row'><div class='rowhead'><label>Timeout ohne Bewegung</label><span class='val' id='v_timeout'>");
  h += timeoutSec; h += F(" s</span></div>"
         "<input type='range' name='timeout' min='2' max='300' step='1' value='");
  h += timeoutSec;
  h += F("' oninput=\"document.getElementById('v_timeout').textContent=this.value+' s'\"></div>");

  h += F("<div class='subcard'><h3>Erweitert</h3>");

  h += F("<div class='row'><div class='rowhead'><label>Einblendzeit</label><span class='val' id='v_fadein'>");
  h += settings.fadeInMs; h += F(" ms</span></div>"
         "<input type='range' name='fadeIn' min='0' max='5000' step='100' value='");
  h += settings.fadeInMs;
  h += F("' oninput=\"document.getElementById('v_fadein').textContent=this.value+' ms'\"></div>");

  h += F("<div class='row'><div class='rowhead'><label>Ausblendzeit</label><span class='val' id='v_fadeout'>");
  h += settings.fadeOutMs; h += F(" ms</span></div>"
         "<input type='range' name='fadeOut' min='0' max='20000' step='250' value='");
  h += settings.fadeOutMs;
  h += F("' oninput=\"document.getElementById('v_fadeout').textContent=this.value+' ms'\"></div>");

  h += F("<div class='row'><div class='rowhead'><label>Dunkel-Schwelle</label><span class='val' id='v_lux'>");
  h += luxPct; h += F("%</span></div>"
         "<input type='range' name='lux' min='0' max='100' step='1' value='");
  h += luxPct;
  h += F("' oninput=\"document.getElementById('v_lux').textContent=this.value+'%'\">"
         "<div class='hint'>h&ouml;her = es muss dunkler sein, bevor das Licht angeht</div></div>");

  h += F("<div class='row'><div class='rowhead'><label>Best&auml;tigungszeit Umgebungslicht</label><span class='val' id='v_ambconf'>");
  h += ambientConfirmSec; h += F(" s</span></div>"
         "<input type='range' name='ambConf' min='2' max='120' step='1' value='");
  h += ambientConfirmSec;
  h += F("' oninput=\"document.getElementById('v_ambconf').textContent=this.value+' s'\">"
         "<div class='hint'>ein Hell/Dunkel-Wechsel z&auml;hlt erst, wenn er so lange durchgehend "
         "anh&auml;lt — schützt vor kurzen St&ouml;rungen durch die eigenen LEDs oder ein zweites "
         "Ger&auml;t in der N&auml;he. H&ouml;her = robuster, aber tr&auml;ger.</div></div>");

  h += F("</div>"); // end subcard
  h += F("</div>"); // end card Timing

  h += F("<button type='submit' class='btn'>Speichern</button>");
  h += F("</form>");

  h += F("<script>"); h += FPSTR(KELVIN_JS); h += F("</script>");

  h += pageFoot();
  server.send(200, "text/html", h);
}

void handleParamsSave() {
  if (server.hasArg("brightness")) settings.brightness = pctToByte(server.arg("brightness").toInt());
  if (server.hasArg("kelvin"))     settings.colorKelvin = constrain(server.arg("kelvin").toInt(), 300, 6500);
  if (server.hasArg("colorMode"))  settings.colorMode = (server.arg("colorMode").toInt() == 1) ? 1 : 0;
  if (server.hasArg("customColor")) {
    String c = server.arg("customColor"); // expected format "#rrggbb"
    if (c.length() == 7 && c.charAt(0) == '#') {
      long v = strtol(c.c_str() + 1, nullptr, 16);
      settings.customR = (uint8_t)((v >> 16) & 0xFF);
      settings.customG = (uint8_t)((v >> 8) & 0xFF);
      settings.customB = (uint8_t)(v & 0xFF);
    }
  }
  if (server.hasArg("timeout"))    settings.timeoutMs = (uint32_t)constrain(server.arg("timeout").toInt(), 0, 3600) * 1000UL;
  if (server.hasArg("fadeIn"))     settings.fadeInMs = constrain(server.arg("fadeIn").toInt(), 0, 60000);
  if (server.hasArg("fadeOut"))    settings.fadeOutMs = constrain(server.arg("fadeOut").toInt(), 0, 60000);
  if (server.hasArg("lux"))        settings.luxThreshold = pctToRaw(server.arg("lux").toInt());
  if (server.hasArg("ambConf"))    settings.ambientConfirmMs = (uint32_t)constrain(server.arg("ambConf").toInt(), 2, 120) * 1000UL;
  if (server.hasArg("numLedsFront")) settings.numLedsFront = constrain(server.arg("numLedsFront").toInt(), 1, MAX_LEDS_PER_SIDE);
  if (server.hasArg("numLedsSide"))  settings.numLedsSide  = constrain(server.arg("numLedsSide").toInt(), 1, MAX_LEDS_PER_SIDE);

  saveSettings();

  // Apply the new pixel counts immediately (updateLength reallocates the
  // strip buffer) instead of requiring a reboot.
  frontCtrl.setNumLeds(settings.numLedsFront);
  sideCtrl.setNumLeds(settings.numLedsSide);

  server.sendHeader("Location", "/params");
  server.send(303);
}

// ---------------------------------------------------------------------------
// WLAN page
// ---------------------------------------------------------------------------
void handleWifiPage() {
  String h = pageHead("wifi");

  h += F("<div class='card'><h2>Ger&auml;tename</h2>");
  h += F("<form method='POST' action='/savedevice'>");
  h += F("<div class='row'><label>Zusatz zum Ger&auml;tenamen</label>"
         "<input type='text' name='deviceLabel' maxlength='23' placeholder='z.B. vorne' value='");
  h += settings.deviceLabel;
  h += F("'><div class='hint'>wird an &bdquo;"); h += DEVICE_NAME;
  h += F("&ldquo; angeh&auml;ngt, z.B. &bdquo;"); h += DEVICE_NAME;
  h += F(" vorne&ldquo; - praktisch bei mehreren Ger&auml;ten im selben WLAN. Wird auch f&uuml;r den "
         "WLAN-Hostnamen genutzt (bedlight"); if (strlen(settings.deviceLabel) > 0) { h += "-..."; }
  h += F(".local), der erst nach einem Neustart wechselt.</div></div>");
  h += F("<button type='submit' class='btn'>Speichern</button>");
  h += F("</form></div>");

  h += F("<div class='card'><h2>WLAN</h2>");
  h += F("<form method='POST' action='/savewifi'>");
  h += F("<div class='row'><label>SSID</label>"
         "<input type='text' name='wifiSsid' maxlength='32' value='");
  h += settings.wifiSsid;
  h += F("'></div>");
  h += F("<div class='row'><label>Passwort (leer = unver&auml;ndert)</label>"
         "<input type='password' name='wifiPass' maxlength='64'></div>");
  h += F("<button type='submit' class='btn'>Speichern</button>");
  h += F("</form></div>");

  h += F("<div class='card'><h2>Aktionen</h2><div class='actionbar'>"
         "<button class='btn secondary small' onclick=\"if(confirm('Ger&auml;t jetzt neu starten?'))post('/action/reboot')\">Neu starten</button>"
         "</div></div>");

  h += F("<script>function post(u){fetch(u,{method:'POST'});}</script>");

  h += pageFoot();
  server.send(200, "text/html", h);
}

void handleWifiSave() {
  if (server.hasArg("wifiSsid")) {
    server.arg("wifiSsid").toCharArray(settings.wifiSsid, sizeof(settings.wifiSsid));
  }
  if (server.hasArg("wifiPass") && server.arg("wifiPass").length() > 0) {
    server.arg("wifiPass").toCharArray(settings.wifiPass, sizeof(settings.wifiPass));
  }
  saveSettings();

  server.sendHeader("Location", "/wifi");
  server.send(303);
}

void handleDeviceSave() {
  if (server.hasArg("deviceLabel")) {
    server.arg("deviceLabel").toCharArray(settings.deviceLabel, sizeof(settings.deviceLabel));
  }
  saveSettings();

  server.sendHeader("Location", "/wifi");
  server.send(303);
}

// ---------------------------------------------------------------------------
// Firmware / OTA page
// ---------------------------------------------------------------------------
void handleUpdatePage() {
  String h = pageHead("update");

  h += F("<div class='card'><h2>Firmware-Update</h2>"
         "<p class='hint'>Neue .bin Datei ausw&auml;hlen und hochladen. Das Ger&auml;t startet danach automatisch neu.</p>"
         "<form method='POST' action='/update' enctype='multipart/form-data'>"
         "<div class='row'><input type='file' name='firmware' accept='.bin' required></div>"
         "<button type='submit' class='btn'>Hochladen &amp; Flashen</button>"
         "</form></div>");

  h += pageFoot();
  server.send(200, "text/html", h);
}

void handleUpdateResult() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", Update.hasError() ? "Update fehlgeschlagen" : "Update OK, Neustart...");
  delay(500);
  ESP.restart();
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA success: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

// ---------------------------------------------------------------------------
// JSON status API (polled by the status page every 2s)
// ---------------------------------------------------------------------------
void handleStatusApi() {
  String j = "{";
  // Shown to the user as ambient BRIGHTNESS (matches the "hell"/"dunkel"
  // label next to it) - the divider on this build is wired "inverted"
  // (higher raw = darker, see ambientIsDark()), so this is 100 minus the
  // raw percentage, not the raw percentage itself.
  j += "\"ambientPct\":" + String(100 - rawToPct((uint16_t)ambientFiltered));
  j += ",\"dark\":" + String(ambientIsDark() ? "true" : "false");
  j += ",\"testMode\":" + String(settings.testMode ? "true" : "false");
  j += ",\"ip\":\"" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\"";
  j += ",\"front\":{\"state\":\"" + String(frontCtrl.stateName()) + "\",\"brightness\":" + String(frontCtrl.currentBrightness) + ",\"motion\":" + String(frontCtrl.motionActive ? "true" : "false") + "}";
  j += ",\"side\":{\"state\":\"" + String(sideCtrl.stateName()) + "\",\"brightness\":" + String(sideCtrl.currentBrightness) + ",\"motion\":" + String(sideCtrl.motionActive ? "true" : "false") + "}";
  j += "}";
  server.send(200, "application/json", j);
}

// ---------------------------------------------------------------------------
// Debug-log JSON API + actions (see logMotion()/logBuffer above)
// ---------------------------------------------------------------------------
void handleLogApi() {
  String j = "{\"enabled\":";
  j += settings.debugLogEnabled ? "true" : "false";
  j += ",\"maxEntries\":" + String(settings.logMaxEntries);
  j += ",\"entries\":[";
  uint8_t cap = settings.logMaxEntries;
  for (uint8_t i = 0; i < logCount; i++) {
    if (i > 0) j += ",";
    // i-th newest entry (0 = most recently logged) - inlined, see the note
    // above logClear()/logMotion() for why this isn't its own function.
    uint8_t idx = (uint8_t)((logHead + cap - 1 - i) % cap);
    LogEntry &e = logBuffer[idx];
    j += "{\"side\":\"" + String(e.side) + "\",\"brightnessPct\":" + String(e.brightnessPct) +
         ",\"timestamp\":\"" + formatTimestamp(e.ts) + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleActionDebugLog() {
  if (server.hasArg("v")) {
    settings.debugLogEnabled = server.arg("v") == "1";
    saveSettings();
  }
  server.send(200, "text/plain", "OK");
}

void handleActionLogMax() {
  if (server.hasArg("v")) {
    settings.logMaxEntries = constrain(server.arg("v").toInt(), 1, MAX_LOG_ENTRIES);
    saveSettings();
    logClear(); // cap changed - avoid stale entries mixing across two caps
  }
  server.send(200, "text/plain", "OK");
}

void handleActionLogClear() {
  logClear();
  server.send(200, "text/plain", "OK");
}

// ---------------------------------------------------------------------------
// WiFi setup: try stored credentials, fall back to a setup AP
// ---------------------------------------------------------------------------
void startAccessPoint() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP-Modus gestartet. Verbinde mit WLAN '");
  Serial.print(AP_SSID);
  Serial.print("' (Passwort: ");
  Serial.print(AP_PASS);
  Serial.print("), dann Browser: http://");
  Serial.println(ip);
  dnsServer.start(53, "*", ip); // simple captive portal: every DNS query -> our IP
}

void connectWiFi() {
  if (strlen(settings.wifiSsid) == 0) {
    startAccessPoint();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(settings.wifiSsid, settings.wifiPass);
  Serial.print("Verbinde mit WLAN '");
  Serial.print(settings.wifiSsid);
  Serial.print("'...");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" verbunden, IP: ");
    Serial.println(WiFi.localIP());
    apMode = false;

    // Kick off NTP time sync in the background (needed for real date/time
    // stamps in the debug log, see formatTimestamp()/logMotion()). Async -
    // does not block startup; entries logged before it completes fall back
    // to the "not synced yet" placeholder.
    configTzTime(TZ_INFO, NTP_SERVER);

    String host = mdnsHostname();
    if (MDNS.begin(host.c_str())) {
      Serial.print("Erreichbar unter http://");
      Serial.print(host);
      Serial.println(".local/");
    }
  } else {
    Serial.println(" fehlgeschlagen, starte Setup-AP.");
    startAccessPoint();
  }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12); // 0-4095

  loadSettings();

  frontCtrl.begin(settings.numLedsFront);
  sideCtrl.begin(settings.numLedsSide);
  ambientBegin();

  connectWiFi();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/action/testmode", HTTP_POST, handleActionTestMode);
  server.on("/action/reboot", HTTP_POST, handleActionReboot);

  server.on("/params", HTTP_GET, handleParams);
  server.on("/save", HTTP_POST, handleParamsSave);

  server.on("/wifi", HTTP_GET, handleWifiPage);
  server.on("/savewifi", HTTP_POST, handleWifiSave);
  server.on("/savedevice", HTTP_POST, handleDeviceSave);

  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);

  server.on("/api/status", HTTP_GET, handleStatusApi);
  server.on("/api/log", HTTP_GET, handleLogApi);
  server.on("/action/debuglog", HTTP_POST, handleActionDebugLog);
  server.on("/action/logmax", HTTP_POST, handleActionLogMax);
  server.on("/action/logclear", HTTP_POST, handleActionLogClear);

  server.onNotFound(handleRoot); // helps the captive portal show our page
  server.begin();

  Serial.println("Webserver gestartet.");
}

void loop() {
  server.handleClient();
  if (apMode) dnsServer.processNextRequest();

  unsigned long now = millis();
  bool dark = ambientIsDark();

  frontCtrl.update(dark, now);
  sideCtrl.update(dark, now);
}
