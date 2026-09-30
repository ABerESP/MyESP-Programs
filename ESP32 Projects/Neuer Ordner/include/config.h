#pragma once

// Wi-Fi is used only to synchronize the clock with NTP.
#define WIFI_SSID "MyBox"
#define WIFI_PASSWORD "Ak29#00157"
#define WEB_ADMIN_USERNAME "admin"
#define WEB_ADMIN_PASSWORD WIFI_PASSWORD
#define TIME_ZONE "CET-1CEST,M3.5.0,M10.5.0/3"

#define MQTT_DEFAULT_HOST ""
#define MQTT_DEFAULT_PORT 1883
#define MQTT_DEFAULT_BASE_TOPIC "airq"

// Set to 1 to simulate sensor readings instead of using the physical sensors.
#define SIMULATE_SENSOR_DATA 0

// Set to 0 to use the ENS160 AQI reading instead of cycling AQI 1-5 every 10 s.
#define SIMULATE_AQI_TEST 0

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
#define HDC1008_I2C_ADDRESS 0x43
#define SENSOR_I2C_ADDRESS 0x53

// WS2812 data output; connect pixel ground to ESP32 ground.
#define WS2812_DATA_PIN 32
#define WS2812_LED_COUNT 1
#define WS2812_BRIGHTNESS 48

// ENS160 fallback compensation used until the HDC1008 returns valid readings.
#define ENS160_FALLBACK_TEMPERATURE_C 25.0f
#define ENS160_FALLBACK_HUMIDITY_PERCENT 50.0f