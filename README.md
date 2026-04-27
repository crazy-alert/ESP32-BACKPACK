# ELRS Backpack Telemetry Panel (ESP32) - Multimedia Branch

ESP32 firmware for ELRS Backpack telemetry over ESP-NOW with:
- web panel
- multi-bind profiles
- board battery monitoring
- SD logging with rotation and web viewer
- buzzer alarms
- sound upload/selection + playback (web + I2S to MAX98357A)

## Features

- ESP-NOW telemetry receive (`Telemetry = ESPNOW` in ELRS Lua)
- Captive portal behavior (tap "Authorization required" to open panel)
- Multi-TX bind profiles:
  - note/name
  - bind phrase
  - UID digits
  - save current bind into profile automatically
  - apply/delete saved profile
- Top battery bars on main page:
  - backpack battery
  - drone battery
  - colors: `<40% red`, `<80% yellow`, `>=80% green`
- Board battery voltage + percent on main page/card
- SD card status on main page
- Logs to SD:
  - start only when ARMED
  - rotation by count (`keep_files`)
  - log browser in web UI
  - configurable log channels (GPS mandatory by default)
  - timestamps include RTC-style UTC datetime (synced from browser on panel open)
- GPS QR from logs on demand:
  - loaded only after button press
  - prev/next navigation through saved GPS points from logs
- Buzzer alarms:
  - uplink RSSI low
  - downlink RSSI low
  - LQ low
  - telemetry timeout
  - backpack battery low
  - drone battery low
- Per-alarm sound file selection from `/sound` folder on SD
- Sound upload from Buzzer tab directly to SD (`/sound`)
- LED status:
  - solid ON when telemetry link is active
  - slow blink when no link
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
- set current bind phrase / UID digits / note
- `Save + Apply` stores current bind into saved profiles
- switch back to any saved profile with `Apply`

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

This branch supports:
- web playback for selected alarm files
- direct WAV playback to I2S amplifier (MAX98357A)

MAX98357A wiring (I2S):
- `GPIO26` -> `BCLK`
- `GPIO27` -> `LRC / WS`
- `GPIO22` -> `DIN`
- `3V3` -> `VIN`
- `GND` -> `GND`
- Speaker -> amp output

## Sound Files

Put files in:
- `/sound`

Supported by web streaming endpoint:
- `.wav` (preferred)
- `.mp3`
- `.ogg`

For direct ESP32 -> MAX98357A playback currently use:
- WAV PCM, mono, 16-bit
- sample rate 8k..48k (practical recommended: 16k or 22.05k)

## Logging Behavior

- logging only starts when flight mode indicates ARM
- logging stops automatically when disarmed
- rotation removes oldest files when count exceeds `keep_files`
- each row includes:
  - `ms` (device millis)
  - `timestamp` (UTC ISO, synced from browser)
  - `uptime`
- channel selection configurable in Logging tab:
  - GPS
  - Link
  - Battery
  - Attitude
  - Unknown

## Web Endpoints

- `GET /api/telemetry`
- `GET /api/config`
- `POST /api/time_sync` (`epoch`)
- `POST /api/bind`
- `POST /api/profile_save`
- `POST /api/profile_apply`
- `POST /api/profile_delete`
- `POST /api/log_cfg`
- `POST /api/buzzer_cfg`
- `GET /api/logs`
- `GET /log?name=/logs/log_xxxxxx.csv`
- `GET /api/gps_points?max=120`
- `GET /api/sounds`
- `POST /api/upload_sound` (multipart form-data)
- `GET /sound?file=<name>`

## Notes

- `STA disconnected` is expected in AP-only mode.
- Unknown telemetry section intentionally aggregates by type and count to avoid repeated spam.
- Time sync for RTC-like timestamps is done from browser when panel opens; if not synced, timestamp field is `-`.
