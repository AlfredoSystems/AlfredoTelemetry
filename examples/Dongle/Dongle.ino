/*
  Dongle

  Flash this to the ESP32 that plugs into your computer, then open
  extras/TelemetryViewer/index.html (in Chrome or Edge) and click Connect.

  Web UI is also avalible at:
  https://www.alfredosys.com/AlfredoTelemetry/
  
  Any ESP32 works. For the full data rate use one with native USB, like an
  ESP32-S2 or ESP32-S3, and set Tools > USB CDC On Boot to "Enabled". Boards
  that only have a USB-to-UART chip are limited to about 90 KB/s at 921600
  baud; pick the same baud rate in the page.

  The Wi-Fi channel must match Telemetry.begin() on the robot. You can also
  change it from the page.
*/

#include <AlfredoTelemetryDongle.h>

void setup() {
  // TelemetryDongle.setLongRange();  // for robots that call Telemetry.setLongRange()
  TelemetryDongle.begin(1);  // Wi-Fi channel 1
}

void loop() {
  TelemetryDongle.update();
}
