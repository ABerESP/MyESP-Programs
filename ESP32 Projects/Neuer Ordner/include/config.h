#pragma once

// Wi-Fi is used only to synchronize the clock with NTP.
#define WIFI_SSID "MyBox"
#define WIFI_PASSWORD "Ak29#00157"
#define TIME_ZONE "CET-1CEST,M3.5.0,M10.5.0/3"

// Set to 1 to simulate sensor readings instead of using the physical sensors.
#define SIMULATE_SENSOR_DATA 1

// Set to 0 to use the ENS160 AQI reading instead of cycling AQI 1-5 every 10 s.
#define SIMULATE_AQI_TEST 1

// Fixed e-paper GPIO mapping for the Waveshare ESP32 Driver Board.
#define EPD_BUSY_PIN 25
#define EPD_RST_PIN 26
#define EPD_DC_PIN 27
#define EPD_CS_PIN 15
#define EPD_SCK_PIN 13
#define EPD_MOSI_PIN 14

// ESP32 default I2C pins; HDC1008 is 0x40 and SEN0515 defaults to 0x53.
#define SENSOR_SDA_PIN 21
#define SENSOR_SCL_PIN 22
#define HDC1008_I2C_ADDRESS 0x40
#define SENSOR_I2C_ADDRESS 0x53

// ENS160 fallback compensation used until the HDC1008 returns valid readings.
#define ENS160_FALLBACK_TEMPERATURE_C 25.0f
#define ENS160_FALLBACK_HUMIDITY_PERCENT 50.0f