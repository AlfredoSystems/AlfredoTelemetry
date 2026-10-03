/*
  NoU3_LogIMU

  Streams NoU3 IMU data to the telemetry viewer: the gyroscope (rad/s), the
  accelerometer (g), and the VQF fused orientation - roll, pitch, and yaw
  (rad). One row per IMU sample (~104 Hz). It's the same data the
  Alfredo-NoU3 LogDataIMU example writes to an SD card, sent over ESP-NOW
  instead.

  Needs the Alfredo-NoU3 library.

  How to use:
  - Flash the Dongle example to a second ESP32 and plug it into your computer.
  - Open extras/TelemetryViewer/index.html, click Connect, and pair with
    "NoU3_IMU".
  - Click Record to start and again to stop, then Save to download the CSV.
  - Keep the robot still for a moment at power-up: NoU3.calibrateIMUs() waits
    for it to be at rest before zeroing yaw.

  Service light:
  - short blip once a second: running, looking for the dongle
  - solid: paired with the page, not streaming
  - 3 Hz flash: streaming
  - fast 10 Hz flash: telemetry failed to start

  USB debugging (optional, the robot runs fine on battery):
  - Open the Serial Monitor at 115200 baud. Once a second it prints what
    telemetry is doing: the Wi-Fi channel the radio is really on, HELLOs sent
    (the robot announcing itself to dongles), pairing, and any send failures.

  Checking the data:
  - "sample" counts IMU samples. A jump in it means rows were lost over the
    radio. The page's Link panel shows packet loss too.
  - time_s is when a sample was seen, within 1 ms of when the IMU made it.
*/

#include <Alfredo_NoU3.h>
#include <AlfredoTelemetry.h>

bool telemetryStarted = false;

// The library publishes each sample's raw readings first and its fused
// angles last, so a change in the angles marks a complete new sample. Every
// Telemetry.add() happens in this task, and each sample becomes one frame.
void taskSampleIMU(void *pvParameters) {
  float lastRoll = NAN, lastPitch = NAN, lastYaw = NAN;
  uint32_t sample = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1));

    float roll = NoU3.roll, pitch = NoU3.pitch, yaw = NoU3.yaw;
    if (roll == lastRoll && pitch == lastPitch && yaw == lastYaw) continue;
    lastRoll = roll;
    lastPitch = pitch;
    lastYaw = yaw;
    sample++;

    Telemetry.add("sample", sample);
    Telemetry.add("gyro_x_rad_s", NoU3.gyroscope_x);
    Telemetry.add("gyro_y_rad_s", NoU3.gyroscope_y);
    Telemetry.add("gyro_z_rad_s", NoU3.gyroscope_z);
    Telemetry.add("accel_x_g", NoU3.acceleration_x);
    Telemetry.add("accel_y_g", NoU3.acceleration_y);
    Telemetry.add("accel_z_g", NoU3.acceleration_z);
    Telemetry.add("roll_rad", roll);
    Telemetry.add("pitch_rad", pitch);
    Telemetry.add("yaw_rad", yaw);
    Telemetry.send();
  }
}

void setup() {
  Serial.begin(115200);
  // The IMU only needs about 6 KB/s, so trade speed for range: 6 Mbps
  // reaches a few dB farther than the 12 Mbps default.
  Telemetry.setRadioRate(WIFI_PHY_RATE_6M);
  // For the most range, use Long Range mode instead (several dB more than
  // 1 Mbps, plenty of speed for the IMU). The dongle sketch must then call
  // TelemetryDongle.setLongRange() too.
  // Telemetry.setLongRange();
  telemetryStarted = Telemetry.begin("NoU3_IMU");
  Telemetry.printStatus(Serial);

  NoU3.begin();
  NoU3.setServiceLight(LIGHT_OFF);

  Serial.println("Calibrating the IMU, keep the robot still...");
  NoU3.calibrateIMUs();
  Serial.println("IMU calibrated");

  xTaskCreatePinnedToCore(taskSampleIMU, "taskSampleIMU", 4096, NULL, 2, NULL, 1);
}

void loop() {
  unsigned long t = millis();
  bool on;
  if (!telemetryStarted) on = t % 100 < 50;                // failed to start
  else if (Telemetry.isStreaming()) on = t % 333 < 167;    // streaming
  else if (Telemetry.isConnected()) on = true;             // paired
  else on = t % 1000 < 60;                                 // looking for the dongle
  NoU3.setServiceLight(on ? LIGHT_ON : LIGHT_OFF);

  static unsigned long lastPrintTime = 0;
  if (t - lastPrintTime >= 1000) {
    lastPrintTime = t;
    Telemetry.printStatus(Serial);
  }

  delay(1);
}
