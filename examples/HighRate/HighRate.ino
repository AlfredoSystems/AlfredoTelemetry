/*
  HighRate

  A throughput test: 16 channels at 1 kHz (about 90 KB/s over the air).
  Watch the page's link stats for lost packets and the robot's dropped
  frames. If frames are dropped, try a faster radio rate or a lower rate.

  Hold BOOT to pause streaming data from the loop (the page stays connected).
*/

#include <AlfredoTelemetry.h>

const int CHANNELS = 16;
char names[CHANNELS][12];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < CHANNELS; i++) snprintf(names[i], sizeof(names[i]), "ch_%02d", i);

  Telemetry.setMaxRate(1000);                // frames per second (the default)
  Telemetry.setRadioRate(WIFI_PHY_RATE_24M); // more headroom than the default 12 Mbps
  Telemetry.setBufferSize(32768);
  Telemetry.begin("HighRate");
  pinMode(0, INPUT_PULLUP);
}

void loop() {
  static uint32_t nextUs = micros();
  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += 1000;

  if (digitalRead(0) == HIGH) {
    float t = micros() * 1e-6;
    for (int i = 0; i < CHANNELS; i++) {
      Telemetry.add(names[i], sin(TWO_PI * (0.2 + 0.1 * i) * t) + i * 2.5);
    }
    Telemetry.send();
  }

  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    Telemetry.printf("frames sent %lu, dropped %lu\n", Telemetry.getFramesSent(), Telemetry.getFramesDropped());
  }
}
