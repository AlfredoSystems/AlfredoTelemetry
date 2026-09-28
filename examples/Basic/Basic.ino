/*
  Basic

  The smallest AlfredoTelemetry robot. Flash the Dongle example to a second
  ESP32 plugged into your computer, open extras/TelemetryViewer/index.html,
  click Connect, and pick "BasicBot".

  - Telemetry.add() records a value. Call it for anything, as often as you
    like; each loop() pass becomes one row of data with one timestamp.
  - Telemetry.tune() lets the page change a variable while the robot runs.
  - Telemetry.printf() prints to the console on the page.
*/

#include <AlfredoTelemetry.h>

float frequency = 0.5;   // Hz, change it from the page
float amplitude = 1.0;
bool squareWave = false;

void setup() {
  Serial.begin(115200);
  Telemetry.begin("BasicBot");

  Telemetry.tune("frequency", &frequency);
  Telemetry.tune("amplitude", &amplitude);
  Telemetry.tune("square_wave", &squareWave);
}

void loop() {
  float t = micros() * 1e-6;
  float wave = amplitude * sin(TWO_PI * frequency * t);
  if (squareWave) wave = wave >= 0 ? amplitude : -amplitude;

  Telemetry.add("wave", wave);
  Telemetry.add("cosine", amplitude * cos(TWO_PI * frequency * t));
  Telemetry.add("millis", millis());
  Telemetry.add("boot_button", digitalRead(0) == LOW);
  Telemetry.send();

  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    Telemetry.printf("up %lu s, %lu frames sent, %lu dropped\n",
                     millis() / 1000, Telemetry.getFramesSent(), Telemetry.getFramesDropped());
  }

  delay(2);  // ~500 rows per second
}
