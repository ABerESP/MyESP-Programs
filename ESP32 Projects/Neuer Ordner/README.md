# AIRQ Sensor

ESP32 air-quality display using the Waveshare ESP32 e-Paper Driver Board, the 2.7-inch 264 x 176 black-and-white Waveshare raw e-paper panel (SKU 13378), the DFRobot SEN0515 (ENS160), and an HDC1008 temperature/humidity sensor. The landscape dashboard shows date and time, temperature, humidity, AQI and its five-level bar, TVOC, eCO2, and the sensor IP address. The dashboard refresh interval is configurable; a full e-paper refresh runs every 30 minutes.

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

Connect the SEN0515 and HDC1008 in parallel over I2C: SDA to GPIO 21, SCL to GPIO 22, 3V3 to VCC, and GND to GND. The SEN0515 default I2C address is `0x53`; the HDC1008 uses `0x40`. Use 3.3 V logic and a shared ground.

Connect the WS2812 DIN to GPIO 32 and connect its ground to ESP32 GND. Power the pixel from a suitable 5 V supply; for reliable data at 5 V, use a 3.3 V-to-5 V level shifter. AQI 1-5 maps from green through yellow-green, yellow, and orange to red. The pin, pixel count, and brightness are configurable in `include/config.h`.

## Web interface

After the ESP32 joins Wi-Fi, open `http://<sensor-ip>/`. The IP is printed to serial and shown in the display header. The status page follows the compact status-and-links layout of the referenced controller page. Use **WLAN** to change credentials, **Parameter** to set display refresh seconds, WS2812 brightness, and HDC1008 fallback temperature/humidity, **MQTT** to configure the broker, and **Firmware** to upload an OTA image. Settings are stored in NVS and survive restarts.

Configuration and firmware pages use HTTP Basic Auth: username `WEB_ADMIN_USERNAME`, password `WEB_ADMIN_PASSWORD` from `include/config.h` (the default password aliases `WIFI_PASSWORD`). OTA accepts the PlatformIO binary `.pio/build/esp32dev/firmware.bin`; the board restarts after a successful update. This is plain HTTP, so keep the interface on a trusted LAN and do not port-forward it to the Internet.

MQTT is disabled until enabled in **MQTT** settings. Configure the broker host, port, optional username/password, and base topic. The firmware publishes retained AQI, temperature, and humidity states and sends Home Assistant MQTT Discovery entities under the `homeassistant` discovery prefix.

The selected panel driver is GxEPD2's `GxEPD2_270` (IL91874/GDEW027W3), with landscape rotation for the 264 x 176 panel. Check the label/controller of your display revision and adjust the display class in `src/main.cpp` if it differs. The pin assignments are board-fixed; check Waveshare's documentation if your board revision differs.

## Configuration

1. Set `WIFI_SSID` and `WIFI_PASSWORD` in `include/config.h` for initial Wi-Fi provisioning. Wi-Fi is used to get time from NTP; credentials can also be changed from the web interface.
2. Set `TIME_ZONE` to a POSIX timezone string, for example `CET-1CEST,M3.5.0,M10.5.0/3` for Central European time.
3. Simulation is configurable from **Parameter** in the web interface and saved in NVS. Choose simulated sensor readings, AQI test cycling, automatic or fixed values, and set temperature, humidity, AQI, TVOC, and eCO2. Automatic AQI test cycles every 10 seconds; TVOC/eCO2 vary every 5 seconds. The compile-time `SIMULATE_SENSOR_DATA` and `SIMULATE_AQI_TEST` settings provide first-boot defaults. The dashboard still refreshes each second and shows time in `HH:MM:SS`. HDC1008 readings are passed to the ENS160 for temperature/humidity compensation. If the HDC1008 is unavailable, the configured fallback temperature and humidity are used.

The ENS160 reports AQI (1-5), TVOC (ppb), and eCO2 (ppm). During warm-up, the rating area displays `Aufwaermen...`.

## Build and upload

Install PlatformIO for VS Code, open this folder, then run **PlatformIO: Build** and **PlatformIO: Upload**. Open the serial monitor at 115200 baud for connection and sensor status messages.