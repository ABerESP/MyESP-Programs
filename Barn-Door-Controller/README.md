# Barn Door Controller

This project is a custom ESP32-based barn door controller for an automated door system. It is designed to safely move a door or shutter, detect obstruction, learn travel limits, monitor system health, and expose configuration and status through Wi‑Fi, MQTT, and a built-in web interface.

## Overview

The controller combines:

- stepper motor control with direction and pulse outputs
- Hall-effect end switches for end-stop detection
- a magnetic encoder for position tracking
- a load-cell/scale interface for weight monitoring
- persistent configuration storage in ESP32 flash memory
- runtime configuration over a local web UI and MQTT
- OTA firmware update support

This system is intended for a barn-style sliding or lifting door that needs automated motion with safety checks and adjustable operation parameters.

## Features

- automatic open/close behavior
- learning mode to determine effective travel range and safe motion values
- collision and obstruction detection using weight change and motion monitoring
- emergency stop logic for unsafe conditions
- encoder and scale fault detection with timeout monitoring
- configurable safety margins and motion parameters
- Wi‑Fi setup and persistent preferences storage
- MQTT status publishing and command handling
- Home Assistant-friendly discovery topics
- web interface for monitoring and parameter adjustments
- OTA firmware updates

## Hardware Requirements

The firmware targets an ESP32-C3 development board and expects the following hardware:

- stepper motor driver with enable, direction, and step control
- stepper motor and mechanical drive system
- Hall sensors for open/closed reference detection
- AS5600 magnetic encoder or equivalent position sensor
- NAU7802 load cell amplifier / scale module
- appropriate power supply and motor driver wiring

### I/O mapping used by the firmware

- EN pin: motor enable
- DIR pin: direction control
- STEP pin: motion pulses
- HALL1 and HALL2: door position detection
- I2C SDA/SCL: sensor bus for scale and encoder
- Wi‑Fi and MQTT network stack for remote control

## Software Behavior

The controller runs a state machine that handles:

- acceleration
- cruising
- deceleration
- stopping
- pausing
- reversing
- emergency handling
- idle/off states

During operation it continuously:

1. reads encoder position and door sensor state,
2. reads live load-cell information,
3. checks whether movement is within expected weight limits,
4. identifies abnormal load deltas or stale sensor input,
5. issues an emergency stop if the motion becomes unsafe,
6. publishes state and diagnostic information over MQTT and the local UI.

## Learn and Calibration Mode

The firmware includes a dedicated learning cycle that allows the controller to establish safe motion behavior for the real mechanical installation. During this phase it evaluates:

- maximum movement deviation
- speed safety factor
- travel behavior under load
- blank step offset parameters
- calibration reference values
- door-specific timing and safety settings

This helps adapt the controller to the exact door geometry, motor response, and load characteristics before normal operation.

## Safety Logic

The project includes several protection mechanisms:

- weight plausibility checks
- stop on excessive weight delta
- stale scale detection
- encoder loss detection
- blocking of unsafe motion under invalid conditions
- reverse protection for jammed or overloaded operation
- configurable motor shutdown delay after fault conditions

These protections are essential because the system actively drives a heavy mechanical load and must avoid damage or unsafe motion.

## Configuration and Persistence

The controller stores key values in the ESP32 Preferences area, including:

- movement speed and deceleration settings
- minimum and maximum weight thresholds
- safety parameters and fault thresholds
- learning cycle settings
- autoclose delay and timing values
- Wi‑Fi and MQTT connection parameters

Runtime settings can also be adjusted via the built-in web interface and MQTT topics.

## MQTT and Web Interface

The controller publishes operational data and accepts commands for:

- mode changes
- reset
- relearn
- tare
- calibration
- manual close
- error release
- Wi‑Fi and MQTT configuration

It also provides Home Assistant discovery messages so that selected values can be exposed as entities in a smart home setup.

## Setup Procedure

Follow this procedure for first-time installation and commissioning.

### 1. Prepare the hardware

- verify the motor wiring matches the driver input pin mapping
- check the stepper driver power and logic supply are correct
- connect the Hall sensors and confirm they are wired to the expected active level
- connect the AS5600 encoder and NAU7802 load cell amplifier to the I2C bus
- make sure the door is mechanically safe to move before energizing the system

### 2. Install the project dependencies

Open the project in PlatformIO and install the required libraries from the project configuration:

```bash
pio run
```

If libraries are not automatically fetched, rebuild the project after confirming the environment is configured correctly.

### 3. Configure the board

Set the project board and framework in [platformio.ini](platformio.ini). For this project it is configured for the ESP32-C3 DevKitM-1 board.

### 4. Flash the firmware

From the project folder, upload the firmware:

```bash
pio run -t upload
```

After the upload, open the serial monitor to verify startup logs and confirm the ESP32 connects to the configured network.

### 5. Connect to the device

On first startup the device attempts to use the stored or default Wi‑Fi and MQTT configuration. If needed, update these values through the web interface or the flashed configuration endpoints.

Important: before deployment, change the default credentials and OTA password from the values embedded in the firmware.

### 6. Verify sensor behavior

Before running the door in normal mode:

- check that the scale reports sensible readings
- confirm the encoder changes as the door moves
- verify Hall sensors trigger at the expected end positions
- check for valid weight plausibility values

### 7. Perform a learning cycle

Put the controller into learning mode and run the calibration sequence. This step allows the system to:

- determine travel limits,
- detect the natural motion profile,
- establish reference values for safe operation,
- tune stop and movement thresholds.

### 8. Commission with limited motion

After the learning run:

- test opening and closing in a slow, controlled mode,
- observe the motion and stop conditions,
- verify that the protection logic responds correctly,
- adjust the weight threshold and speed values if necessary.

### 9. Put into normal operation

Once the motion profile is stable and the safety checks behave correctly:

- disable learning mode,
- confirm the controller can handle automatic open/close cycles,
- monitor telemetry through MQTT or the web UI,
- keep the OTA and network security settings updated.

## Build and Upload

From the project folder:

```bash
pio run
pio run -t upload
```

If using VS Code with PlatformIO, use the project task runner or the PlatformIO extension to build and upload the device.

## File Layout

```text
Barn-Door-Controller/
├── src/
│   └── main.cpp
├── platformio.ini
├── Barn_Door_Controller.ino
├── README.md
└── .gitignore
```

## Important Notes

- The default Wi‑Fi, MQTT, and OTA credentials should be changed before use in a real environment.
- This is a custom automation project; mechanical fit and safety validation are the responsibility of the installer.
- Always verify end-stop and collision logic during commissioning before leaving the system in autonomous operation.
- The project assumes the door and motor system are mechanically reliable and correctly aligned.

## License

This project is shared as a personal ESP32 and PlatformIO development project. Please respect the repository owner’s distribution and reuse terms before using it in a production or public deployment.
