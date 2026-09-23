# ESP32-C3 PlatformIO project instructions

This repository is a PlatformIO-based Arduino project for an ESP32-C3 device.

## Project realities
- Board target: `esp32-c3-devkitm-1` in `platformio.ini`
- Framework: Arduino on ESP32
- C++ standard: C++17 (`build_flags` in `platformio.ini`)
- Libraries: Adafruit NeoPixel, WiFi, WebServer, Update, Preferences, DNSServer, ESPmDNS
- Main firmware lives in `src/main.cpp`

## Coding expectations
- Prefer minimal, targeted changes that preserve the existing architecture.
- Keep the ESP32-C3 pin assumptions consistent with the current hardware mapping in `src/main.cpp`.
- Use `Serial` / debug logging only when it fits the current behavior and does not add noise.
- Preserve existing WiFi, mDNS, OTA update, NVS preferences, and Adafruit NeoPixel patterns unless the task explicitly requires a change.
- Favor explicit, readable Arduino/ESP32 code over clever abstractions.

## Build and verification
- Validate changes with the project build command before claiming completion:
  - `platformio run`
- If you change the firmware logic, also reason about runtime behavior on ESP32-C3 constraints such as ADC input handling, power draw, and WiFi/OTA startup timing.

## Safety and scope
- Do not silently break the OTA/web configuration flow, NVS persistence, or LED behavior.
- Do not change pin numbers, WiFi names/password defaults, or the existing LED logic unless the user explicitly asks for that change.
- Keep compile compatibility with the existing ESP32 toolchain and Arduino framework.
