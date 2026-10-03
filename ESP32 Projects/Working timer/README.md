# Working timer

PlatformIO starter project for an ESP8266-12F on the Waveshare E-Paper ESP8266 Driver Board with a 2.13-inch Waveshare three-color e-Paper display marked WFT0213CZ16.

The firmware waits for a start-button press, then runs an 8.5-hour countdown. The landscape display shows the remaining time, a rounded completion percentage, and 17 half-hour squares. Expired squares turn white from left to right; remaining squares stay red. The status circle is white while ready or finished and red while active. The screen refreshes every five minutes. Resetting or powering off the ESP8266 returns the timer to the ready state.

## Wiring

The Waveshare driver board has fixed connections. The panel is the 212 x 104 three-color WFT0213CZ16 variant, not the similarly sized 250 x 122 black-and-white panel:

| E-paper signal | ESP8266 GPIO | Board label |
| --- | ---: | --- |
| BUSY | 5 | D1 |
| DC | 4 | D2 |
| RST | 2 | D4 |
| CS | 15 | D8 |
| SCK | 14 | D5 |
| DIN / MOSI | 13 | D7 |
| Start button | 12 | D6 |

Connect a momentary start button between GPIO12 (D6) and GND. The firmware uses the internal pull-up resistor. The driver board is powered through its documented 3.3 V input. The display class in `src/main.cpp` must match the panel revision.

## Build and upload

Open this folder in VS Code with PlatformIO installed. Use **PlatformIO: Build** to compile and **PlatformIO: Upload** to flash the ESP8266. Open the serial monitor at 115200 baud.