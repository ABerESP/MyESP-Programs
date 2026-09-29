# AIRQ Sensor

ESP32 air-quality display using the Waveshare ESP32 e-Paper Driver Board, the 2.7-inch 264 x 176 black-and-white Waveshare raw e-paper panel (SKU 13378), the DFRobot SEN0515 (ENS160), and an AHT20 temperature/humidity sensor. The landscape dashboard shows date and time, temperature, humidity, AQI and its five-level bar, TVOC, and eCO2. It uses a partial refresh each minute and a full refresh every 30 updates.

## Hardware connections

The firmware uses the fixed e-paper GPIO mapping documented for the Waveshare ESP32 e-Paper Driver Board:

| Signal | ESP32 GPIO |
| --- | ---: |
| E-paper BUSY | 25 |
| E-paper RST | 26 |
| E-paper DC | 27 |
| E-paper CS | 15 |
| E-paper SCK | 13 |
| E-paper MOSI / DIN | 14 |

Connect the SEN0515 and AHT20 in parallel over I2C: SDA to GPIO 21, SCL to GPIO 22, 3V3 to VCC, and GND to GND. The SEN0515 default I2C address is `0x53`; the AHT20 uses `0x38`. Use 3.3 V logic and a shared ground.

The selected panel driver is GxEPD2's `GxEPD2_270` (IL91874/GDEW027W3), with landscape rotation for the 264 x 176 panel. Check the label/controller of your display revision and adjust the display class in `src/main.cpp` if it differs. The pin assignments are board-fixed; check Waveshare's documentation if your board revision differs.

## Configuration

1. Set `WIFI_SSID` and `WIFI_PASSWORD` in `include/config.h`. Wi-Fi is used to get time from NTP; without credentials, the dashboard still shows sensor readings and `--:--` for time.
2. Set `TIME_ZONE` to a POSIX timezone string, for example `CET-1CEST,M3.5.0,M10.5.0/3` for Central European time.
3. `SIMULATE_SENSOR_DATA` is enabled by default and produces changing sample readings without sensor hardware. Set it to `0` to read the physical sensors. `SIMULATE_AQI_TEST` cycles AQI levels 1-5 every second; set it to `0` to use the ENS160 AQI. The dashboard refreshes each second and shows time in `HH:MM:SS`. The AHT20 readings are passed to the ENS160 for temperature/humidity compensation. If the AHT20 is unavailable, fallback compensation is 25 C and 50% RH.

The ENS160 reports AQI (1-5), TVOC (ppb), and eCO2 (ppm). During warm-up, the rating area displays `Aufwaermen...`.

## Build and upload

Install PlatformIO for VS Code, open this folder, then run **PlatformIO: Build** and **PlatformIO: Upload**. Open the serial monitor at 115200 baud for connection and sensor status messages.