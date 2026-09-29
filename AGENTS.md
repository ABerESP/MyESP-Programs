# AGENTS.md

This workspace contains custom ESP32/PlatformIO projects for home automation hardware. Treat each project folder as a separate target unless the task explicitly spans multiple folders.

## Project layout

- Root overview: [README.md](README.md)
- Under-bed light: [Under-Bed-Light](Under-Bed-Light/)
- Barn door controller: [Barn-Door-Controller](Barn-Door-Controller/)

## Working conventions

- Prefer small, targeted edits that preserve the existing Arduino/ESP32 architecture.
- Keep pin assignments, GPIO mappings, and hardware assumptions aligned with each firmware project.
- Do not silently change Wi‑Fi credentials, default passwords, OTA settings, or device identities unless the user explicitly asks.
- Preserve existing motion, lighting, Wi‑Fi, preferences, OTA, MQTT, and safety logic unless the task explicitly requires a change.
- Prefer readable ESP32/Arduino code over clever abstractions.

## Build and validation

Run the relevant project build before claiming completion:

```bash
cd "Under-Bed-Light" && pio run
cd "Barn-Door-Controller" && pio run
```

Use the project folder that matches the firmware you changed.

## Project-specific notes

### Under-Bed-Light

- Board target is an ESP32-C3 board.
- Main firmware is in [Under-Bed-Light/src/main.cpp](Under-Bed-Light/src/main.cpp).
- Keep the GPIO mapping and ambient-light logic consistent with the current hardware layout.
- The web UI, Preferences storage, and LED fade behavior are core project behavior.

### Barn-Door-Controller

- Main firmware is in [Barn-Door-Controller/src/main.cpp](Barn-Door-Controller/src/main.cpp).
- Respect the state-machine logic, emergency-stop safety checks, encoder/scale supervision, and MQTT integration.
- Any change affecting motion safety should be reviewed carefully.

## Helpful references

- [README.md](README.md)
- [Under-Bed-Light/README.md](Under-Bed-Light/README.md)
- [Barn-Door-Controller/README.md](Barn-Door-Controller/README.md)
- [Under-Bed-Light/.github/copilot-instructions.md](Under-Bed-Light/.github/copilot-instructions.md)
