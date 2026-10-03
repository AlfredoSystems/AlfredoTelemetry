# AlfredoTelemetry

Easy, high-rate robot telemetry over ESP-NOW. Log any value from your robot in one line, then plot it live, tune variables, and record to CSV in a browser page. Made for ESP32-S2 and ESP32-S3 robots, and it runs alongside BLE (for example [PestoLink](https://pestol.ink)) on the S3.

```cpp
#include <AlfredoTelemetry.h>

void setup() {
  Telemetry.begin("MyRobot");
}

void loop() {
  Telemetry.add("battery_v", NoU3.getBatteryVoltage());
  Telemetry.add("yaw", NoU3.yaw);
  Telemetry.add("mode", driveMode);   // numbers, bools, and enums all work
}
```

```
 robot ESP32  ──ESP-NOW──►  dongle ESP32  ──USB──►  extras/TelemetryViewer/index.html
 (Telemetry)                (Dongle example)        (Chrome or Edge)
```

## Setup

1. **Dongle.** Flash `examples/Dongle` to any ESP32 that plugs into your computer. For the full data rate, use an ESP32-S2 or ESP32-S3 with native USB, and set **Tools > USB CDC On Boot > Enabled**. Boards with a USB-to-UART chip also work, but 921600 baud caps them at about 90 KB/s.
2. **Robot.** Add `Telemetry.begin("MyRobot")` to `setup()` and `Telemetry.add()` calls wherever you want. Try `examples/Basic` first.
3. **Viewer.** Open `extras/TelemetryViewer/index.html` in Chrome or Edge (Web Serial is needed; Firefox and Safari don't have it). Click **Connect** and pick the dongle's port, then choose your robot and click **Pair**. From then on, the page pairs with that robot again by itself.

The viewer is a single local page with no install and no internet needed; uPlot sits next to it in the same folder.

## Robot API

| Call | What it does |
| --- | --- |
| `Telemetry.begin(name, wifiChannel = 1)` | Starts Wi-Fi (station mode, no network needed) and ESP-NOW. The dongle must be on the same channel. |
| `Telemetry.add(name, value)` | Records a value in the current frame. Frames become rows in the viewer and CSV, with one timestamp each. |
| `Telemetry.send()` | Ends the frame. Optional: calling `add()` again for a name already in the frame starts a new frame, and a frame left open for 20 ms goes out on its own. |
| `Telemetry.watch(name, &variable)` | Samples a variable automatically at `setWatchRate()` from the telemetry task. Use it for values other tasks update, like `NoU3.yaw`. |
| `Telemetry.tune(name, &variable)` | Shows a variable in the viewer's Tunables panel, where you can change it while the robot runs. Floats, ints, and bools (bools get a checkbox). |
| `Telemetry.print()`, `println()`, `printf()` | Text for the viewer's console, sent a line at a time. |
| `Telemetry.setMaxRate(hz)` | Most frames per second from `add()`/`send()` (default 1000). Faster passes are merged, keeping each channel's latest value, so a loop running at 20 kHz doesn't flood the radio. |
| `Telemetry.setWatchRate(hz)` | `watch()` sample rate (default 100, max 1000). |
| `Telemetry.setRadioRate(rate)` | Wi-Fi bitrate, default `WIFI_PHY_RATE_12M`. Faster rates like `WIFI_PHY_RATE_24M` carry more data and take less airtime, which helps when sharing the radio with BLE. Slower rates reach farther: `WIFI_PHY_RATE_6M` gains a few dB, `WIFI_PHY_RATE_1M_L` about 8 dB at 12 times the airtime. The dongle's rate is separate and defaults to 1 Mbps, since it only sends small packets. |
| `Telemetry.setTxPower(power)` | Wi-Fi transmit power, e.g. `WIFI_POWER_11dBm` (the default). See [Transmit power](#transmit-power). The dongle has `TelemetryDongle.setTxPower()`, called before `begin()`. |
| `Telemetry.printStatus(Serial)` | Prints one line of link diagnostics over USB: whether telemetry started, the MAC address, the Wi-Fi channel and transmit power the radio is really using, HELLOs sent, pairing, and send failures. |
| `Telemetry.setBufferSize(bytes)` | Buffer between `add()` and the radio. Call before `begin()`; the default is 16 KB. |
| `Telemetry.isConnected()` / `isStreaming()` | True while the page is paired, and while it has streaming on. |
| `Telemetry.getFramesSent()` / `getFramesDropped()` | Frames queued, and frames lost because the buffer was full. |

When the page isn't streaming, `add()` returns straight away, so leaving telemetry in competition code costs almost nothing.

### Made for busy robot code

- **`add()` never waits on the radio.** It copies the value into a frame under a short spinlock, and once per frame hands the frame to a ring buffer without waiting. Looking up a channel by name usually takes a single string compare, because loops add their channels in the same order every pass.
- **All radio work runs in one task on core 0**, where Wi-Fi and BLE already run. `loop()`, NoU3's IMU tasks, and PestoLink stay on core 1. The ESP32-S2 has only one core, so there everything shares core 0.
- **A full buffer drops frames instead of blocking.** `getFramesDropped()` and the viewer's Link panel show when that happens.
- **Call `add()` and `send()` from one task.** Use `watch()` for variables owned by other tasks.

Limits: 64 channels, 32 tunables, 31-character names, and 120-character console lines.

## Viewer

- **Plots.** Plots come from your robot's channels, and the layout is saved per robot name. Click a plot to select it, then click channels in the Channels list to add or remove them, or use each plot's **+ channel** menu. Drag across a plot to zoom, which pauses the live view. Double-click to go back. Long ranges are drawn min/max per pixel, so spikes never disappear. Each plot's **+ link** menu adds the link quality the page measures (signal in dBm, packets per second, data rate, packet loss), sampled 4 times a second; these record and export like any other channel.
- **Start / Stop** (Space) turns streaming on and off on the robot itself, which frees the radio when you don't need data.
- **Record** (R) captures everything the robot sends until you stop it. Recordings show up in the Recordings panel, where **Save** downloads a CSV and **View** opens one to zoom through.
- **Save buffer** saves the last 2 minutes of live data, for when something interesting happens and you weren't recording.
- **Open CSV** loads a saved file back in for review.
- **Pause** (P) freezes the live view while data keeps arriving.
- **The Tunables panel** edits `Telemetry.tune()` variables. Press Enter to send; the value turns back to normal once the robot confirms it.
- **The Link panel** shows signal strength, packet loss, data rate, and the robot's dropped frames and buffer use.

CSV files have one row per frame. The first column is `time_s`, starting at 0, followed by one column per channel. A cell is empty when that frame didn't include the channel.

## Several robots and several computers

- **Several robots.** Every robot shows up in the Robot list, and the dongle only takes packets from the one you pair with, so streams never mix. A robot that isn't paired doesn't stream; it just announces itself twice a second. If two robots share a name, the list adds the end of each one's MAC address, like `SimBot [00:00:0a]`. Give each robot its own name anyway; saved plot layouts are kept per name.
- **Several computers, each with its own robot** (two teams at an event, say) work side by side. If both stream a lot, put each pair on its own Wi-Fi channel.
- **A robot belongs to the first computer that pairs with it.** A second computer sees the robot marked **in use** and can't take it by accident. If it pairs anyway, its dongle waits and pairs the moment the robot is free: when the first page closes, its dongle is unplugged, or it unpairs. **Take over** (next to Unpair) moves the robot to this computer on purpose; the other page then shows the robot as in use and stops getting data.
- **A page only pairs by itself with a robot it has paired with before**, matched by MAC address, so a fresh page at an event never grabs someone else's robot. **Unpair** makes the page forget the robot.

Pairing isn't password protected: anyone with this page can still pick a free robot from the list, or take one over on purpose, and change its tunables.

## Transmit power

Both the robot and the dongle transmit at **11 dBm** by default, not the ESP32's usual 19.5 dBm. This works around a known problem: on some ESP32 boards, Wi-Fi frames sent at high power can't be decoded by anything, while receiving (and BLE) still work fine. The Alfredo NoU3 and Rotini both do this. Measured between two NoU3s side by side: 8.5, 11 and 13 dBm delivered every packet; at 15 dBm 92% of sends failed; at 17 dBm and above nothing got through at all. The ESP32-C3 SuperMini and QT Py ESP32-C3 are other well-known examples; poor antenna matching is the usual suspect.

11 dBm reaches across a room or an arena, with 2 dB of margin below the point where these boards break. If your boards work at full power and you want more range, raise it on both sides:

```cpp
Telemetry.setTxPower(WIFI_POWER_19_5dBm);        // robot
TelemetryDongle.setTxPower(WIFI_POWER_19_5dBm);  // dongle, before begin()
```

## Troubleshooting

- **The robot never shows up, but its status says HELLOs are being sent.** If the dongle's **Dongle: packets heard** (in the Link panel) stays at 0, its radio isn't receiving anything from the robot. Check the transmit power first: if you raised it, go back to the 11 dBm default (see [Transmit power](#transmit-power)).
- **The robot never shows up.** Both sides must be on the same Wi-Fi channel: check `Telemetry.begin(name, channel)`, and the channel picker in the page (it changes the dongle). If the robot also joins a Wi-Fi network, ESP-NOW has to use that network's channel, so set the dongle to it.
- **Only one ESP-NOW user per sketch.** ESP-NOW allows one receive callback, so this library can't share it with other ESP-NOW code in the same sketch.
- **Slow or choppy data with a USB-to-UART dongle.** Use a native-USB S2/S3 dongle with USB CDC On Boot enabled, or use `TelemetryDongle.begin(1, 2000000)` and pick 2000000 in the page if your USB chip supports it.
- **The Link panel shows "Serial errors".** The USB connection is losing bytes. The page recovers on its own, but a board with native USB fixes it.

## Requirements

- ESP32 Arduino core 3.0 or newer (tested with 3.3.10). On 3.3+ (ESP-IDF 5.5), packets carry up to 1470 bytes with ESP-NOW v2; older cores use 250-byte packets, and the robot and dongle agree on the size automatically.
- Chrome or Edge on the computer running the viewer.

The wire format is documented in `src/AlfredoTelemetryProtocol.h`.
