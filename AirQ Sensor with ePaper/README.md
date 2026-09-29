# AirQ Sensor with ePaper

Standalone ESP-IDF C firmware starter targeting the ESP32-C6.

## Hardware status

The sensor, ePaper display, driver board, and GPIO wiring have not been selected or configured. No particular sensor/display model or pin assignment is assumed. The application currently logs a startup message; sensor acquisition and the date/time/value display refresh are placeholders.

Before implementing hardware support, choose the air-quality sensor and ePaper/display controller, confirm the board interfaces and GPIO wiring, and select suitable ESP-IDF drivers. The intended display content is the date, clock, and sensor values, refreshed once per minute.

## Build

Open this folder as an ESP-IDF project. With ESP-IDF available in the environment:

```text
idf.py set-target esp32c6
idf.py build
```

The target is also recorded in `sdkconfig.defaults`.

## Project structure

```text
.
|-- CMakeLists.txt
|-- main/
|   |-- CMakeLists.txt
|   `-- main.c
`-- sdkconfig.defaults
```
