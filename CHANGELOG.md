# Changelog

## 1.0 - 2026-09-29

First release.

### Added
- `Telemetry.add()`, `send()`, `watch()`, `tune()` and `printf()` for streaming values, sampling variables owned by other tasks, editing variables live from the page, and console text, over ESP-NOW from an ESP32 robot.
- Dongle sketch for any ESP32 on USB that bridges ESP-NOW to the browser.
- Browser viewer (extras/TelemetryViewer, also hosted on GitHub Pages) with live plots, recording to CSV, CSV playback, tunables, a console and link statistics. Works offline in Chrome or Edge.
- Pairing: the first computer keeps a robot, others see it as in use and can take it over on purpose. Pages only re-pair on their own with a robot they have used before.
- Wi-Fi transmit power defaults to 8.5 dBm on both sides, because Alfredo NoU3 and Rotini boards can't be heard at full power. `setTxPower()` raises it on boards that can.
- Diagnostics: `Telemetry.printStatus(Serial)` on the robot, and a Wi-Fi scan plus a packets-heard counter on the dongle.
- Examples: Basic, HighRate, Dongle, NoU3_PestoLink (ESP-NOW alongside BLE) and NoU3_LogIMU.
