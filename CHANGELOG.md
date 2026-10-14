# Changelog

## Unreleased

The link now works like UDP: every packet is sent once, in order, and a lost one is a hole the page shows, never a delay or a replay.

### Changed
- The robot waits for the Wi-Fi driver's verdict on each packet instead of a 50 ms guess, and never resends. The old retry loop could keep re-sending one packet for seconds (the dongle received it up to ten times) while the buffer filled and threw away everything new.
- Data packets default to 250 bytes; `Telemetry.setPacketSize()` raises that for throughput on a clean link.
- The frame buffer defaults to 16 KB and drops its oldest frames when full, so what's sent is always fresh.
- Streaming stops when the dongle's heartbeats stop, and resumes when they're back.
- The viewer drops any packet that isn't newer than the last one (duplicates and stale deliveries), and only moves its time axis on a robot reboot, never on a timestamp that goes backwards.

### Removed
- `setRetries()`, `setDropoutGrace()`, `setAdaptiveRate()`, `getRetransmits()`, recording through dropouts, the backlog replay, the adaptive rate ladder, and the Link panel's retransmit count.

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
