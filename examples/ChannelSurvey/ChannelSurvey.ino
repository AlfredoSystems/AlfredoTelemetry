/*
  ChannelSurvey

  Finds a quiet Wi-Fi channel for telemetry. ESP-NOW stays on one channel,
  so other networks on or near it cost you packets. This sketch scans a few
  times, counts the networks around each channel (a network overlaps the
  channels within 4 of its own), and recommends the quietest of 1, 6, and 11.

  Flash it to any ESP32 at the place you'll be running (the venue's Wi-Fi is
  what matters), open the Serial Monitor at 115200 baud, and wait about
  15 seconds. Then put the channel it suggests in both sketches:

    Telemetry.begin("MyRobot", 6);   // robot
    TelemetryDongle.begin(6);        // dongle (or pick it in the page)

  The page's Scan button does a quicker, one-pass version of this.
*/

#include <WiFi.h>

const int SCANS = 3;

// Rough "busyness" per channel: each network adds its received power to its
// own channel and, with less weight, to channels it overlaps.
float busy[15];
int count[15];
float strongest[15];

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(500);
  for (int ch = 0; ch <= 14; ch++) strongest[ch] = -100;

  Serial.println("Scanning...");
  for (int pass = 0; pass < SCANS; pass++) {
    int n = WiFi.scanNetworks(false, true);  // include hidden networks
    for (int i = 0; i < n; i++) {
      int ch = WiFi.channel(i);
      float rssi = WiFi.RSSI(i);
      if (ch < 1 || ch > 14) continue;
      count[ch]++;
      if (rssi > strongest[ch]) strongest[ch] = rssi;
      float power = powf(10.0f, rssi / 10.0f);  // mW, so strong networks dominate
      for (int other = 1; other <= 14; other++) {
        int distance = abs(other - ch);
        if (distance <= 4) busy[other] += power * (1.0f - distance * 0.2f);
      }
    }
    WiFi.scanDelete();
    Serial.printf("  pass %d: %d networks\n", pass + 1, n);
  }

  Serial.println();
  Serial.println("Channel  Networks  Strongest  Busyness (dBm, overlap included)");
  for (int ch = 1; ch <= 13; ch++) {
    float busyDbm = busy[ch] > 0 ? 10.0f * log10f(busy[ch]) : -100;
    Serial.printf("  %2d       %3d      %5.0f       %5.0f%s\n", ch, count[ch] / SCANS,
                  count[ch] ? strongest[ch] : -100.0f, busyDbm, (ch == 1 || ch == 6 || ch == 11) ? "   *" : "");
  }

  // Pick the quietest of the three non-overlapping channels
  int best = 1;
  for (int ch : {6, 11}) {
    if (busy[ch] < busy[best]) best = ch;
  }
  Serial.println();
  Serial.printf("Quietest of 1, 6, 11: channel %d\n", best);
  Serial.printf("  robot:   Telemetry.begin(\"MyRobot\", %d);\n", best);
  Serial.printf("  dongle:  TelemetryDongle.begin(%d);  or pick \"Wi-Fi ch %d\" in the page\n", best, best);
  Serial.println("\nReset the board to scan again.");
}

void loop() {
  delay(1000);
}
