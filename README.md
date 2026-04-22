# ELRS Backpack Telemetry Panel (ESP32) - Multimedia Branch

ESP32 firmware for ELRS Backpack telemetry over ESP-NOW with:
- web panel
- multi-bind profiles
- board battery monitoring
- SD logging with rotation and web viewer
- buzzer alarms
- sound file selection per alarm (web playback + buzzer trigger event)

## Features

- ESP-NOW telemetry receive (`Telemetry = ESPNOW` in ELRS Lua)
- Captive portal behavior (tap "Authorization required" to open panel)
- Multi-TX bind profiles:
  - note/name
  - bind phrase
  - UID digits
  - apply saved profile
- Board battery voltage + percent on main page
- SD card status on main page
- Logs to SD:
  - start only when ARMED
  - rotation by count (`keep_files`)
  - log browser in web UI
  - configurable log channels (GPS mandatory by default)
- Buzzer alarms:
  - uplink RSSI low
  - downlink RSSI low
  - LQ low
  - telemetry timeout
- Per-alarm sound file selection from `/sound` folder on SD
- Unknown telemetry type list at bottom without duplicate spam (type + counter + last payload)

## Build / Upload

```powershell
C:\Users\sid\.platformio\penv\Scripts\platformio.exe run
C:\Users\sid\.platformio\penv\Scripts\platformio.exe run -t upload
C:\Users\sid\.platformio\penv\Scripts\platformio.exe device monitor --port COM8 --baud 115200
```

## ELRS Setup

In ELRS Lua on transmitter:
- `Backpack -> Telemetry -> ESPNOW`

In panel Bind tab:
- set current bind phrase or UID digits
- optionally save multiple profiles and apply them when switching transmitters

## Wiring

### 1) Board Battery (ESP power source monitoring)

Use ADC divider input to `GPIO34`:
- Battery+ -> resistor R1 -> ADC node -> resistor R2 -> GND
- ADC node -> `GPIO34`
- Battery GND -> ESP GND

Recommended divider:
- `R1 = 100k`, `R2 = 100k` (ratio 2.0)

Important:
- never connect LiPo directly to ADC pin
- ESP and battery monitor ground must be common

Current firmware constants:
- `BOARD_BATTERY_ADC_PIN = 34`
- `BOARD_BATTERY_DIVIDER_RATIO = 2.0`
- percent model `3.3V..4.2V`

### 2) SD Card (SPI / VSPI)

- `SD_CS` -> `GPIO5`
- `SD_SCK` -> `GPIO18`
- `SD_MISO` -> `GPIO19`
- `SD_MOSI` -> `GPIO23`
- `VCC` -> `3V3` (or per module spec)
- `GND` -> `GND`

Folders auto-created on card:
- `/logs`
- `/sound`

### 3) Buzzer

Active buzzer output pin:
- `GPIO25` -> buzzer signal
- buzzer GND -> ESP GND

For passive buzzer or external driver, adapt hardware accordingly.

### 4) Audio Output via Amplifier (optional)

This branch handles alarm sound file selection and web playback.
Physical playback on external speaker/headphones depends on your audio hardware path.

Recommended approach for real audio output from ESP32:
- I2S DAC/amp module (e.g. MAX98357A) + speaker
- then map selected alarm files to your playback pipeline

## Sound Files

Put files in:
- `/sound`

Supported by web streaming endpoint:
- `.wav` (preferred)
- `.mp3`
- `.ogg`

Recommended for stable low-power alarm set:
- mono WAV, 16-bit PCM, 16kHz or 22.05kHz

## Logging Behavior

- logging only starts when flight mode indicates ARM
- logging stops automatically when disarmed
- rotation removes oldest files when count exceeds `keep_files`
- channel selection configurable in Logging tab:
  - GPS
  - Link
  - Battery
  - Attitude
  - Unknown

## Web Endpoints

- `GET /api/telemetry`
- `GET /api/config`
- `POST /api/bind`
- `POST /api/profile_save`
- `POST /api/profile_apply`
- `POST /api/log_cfg`
- `POST /api/buzzer_cfg`
- `GET /api/logs`
- `GET /log?name=/logs/log_xxxxxx.csv`
- `GET /api/sounds`
- `GET /sound?file=<name>`

## Notes

- `STA disconnected` is expected in AP-only mode.
- Unknown telemetry section intentionally aggregates by type and count to avoid repeated spam.
