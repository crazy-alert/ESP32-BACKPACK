# ELRS Backpack Telemetry Panel (ESP32)

Firmware for ESP32 that receives ELRS Backpack telemetry over ESP-NOW and shows it in a web panel.

## Features

- ESP-NOW telemetry receive from TX Backpack (`Telemetry = ESPNOW` in ELRS Lua)
- Telemetry dashboard tab
- Bind tab:
  - Bind Phrase input
  - UID digits input (`a,b,c,d,e,f`) for manual/traditional bind cases
  - Effective UID preview
- Captive portal behavior:
  - DNS catch-all to AP IP
  - captive endpoints redirect to `/`
  - on many phones, tapping `Sign in / Authorization required` opens the panel automatically
- Settings persistence via ESP32 Preferences (NVS)

## Initial setup

1. Flash firmware to ESP32.
2. Connect to AP:
   - SSID: `ELRS-Backpack`
   - Password: `12345678`
3. Open captive notification, or `http://192.168.4.1/`.
4. In **Bind** tab:
   - enter Bind Phrase, or
   - enter UID digits from ELRS UI/web panel.
5. If phone shows `Authorization required`, tap it to open the panel.

## API

- `GET /api/telemetry`
- `GET /api/config`
- `POST /api/bind` (`bind_phrase`, `uid_text`)

## Build / upload

```powershell
C:\Users\sid\.platformio\penv\Scripts\platformio.exe run
C:\Users\sid\.platformio\penv\Scripts\platformio.exe run -t upload
C:\Users\sid\.platformio\penv\Scripts\platformio.exe device monitor --port COM8 --baud 115200
```
