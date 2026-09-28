/**
 * A NoU3 holonomic robot driven with PestoLink (BLE) that also streams
 * telemetry over ESP-NOW. Both share the ESP32-S3's radio: ESP-IDF's
 * coexistence scheduler takes turns between them, so driving stays smooth
 * while the page plots the IMU, the joystick, and the motor outputs.
 *
 * Needs the Alfredo-NoU3 and PestoLink-Receive libraries. Flash the Dongle
 * example to a second ESP32, open extras/TelemetryViewer/index.html, and
 * pick "NoU3_Telemetry". Drive from https://pestol.ink as usual.
 */

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <AlfredoTelemetry.h>

NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

// Tunables: change these from the page while driving
float driveScale = 1.0;
float rotationScale = 0.7;
bool fieldCentric = true;

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE
    PestoLink.begin("NoU3_Bluetooth");
    Serial.begin(115200);

    // Same name idea for the telemetry page. Wi-Fi channel 1 must match the dongle.
    Telemetry.begin("NoU3_Telemetry", 1);

    NoU3.begin();

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);

    // The IMU is read in its own task at 104 Hz, so let the telemetry task
    // sample it instead of adding it from loop().
    Telemetry.setWatchRate(104);
    Telemetry.watch("yaw", &NoU3.yaw);
    Telemetry.watch("roll", &NoU3.roll);
    Telemetry.watch("pitch", &NoU3.pitch);
    Telemetry.watch("gyro_z", &NoU3.gyroscope_z);
    Telemetry.watch("accel_x", &NoU3.acceleration_x);
    Telemetry.watch("accel_y", &NoU3.acceleration_y);

    Telemetry.tune("drive_scale", &driveScale);
    Telemetry.tune("rotation_scale", &rotationScale);
    Telemetry.tune("field_centric", &fieldCentric);

    // loop() runs thousands of times a second; 200 rows per second is plenty
    Telemetry.setMaxRate(200);

    NoU3.setServiceLight(LIGHT_CALIBRATING);
    NoU3.calibrateIMUs();
}

void loop() {
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    bool connected = PestoLink.isConnected();
    static bool wasConnected = false;
    if (connected != wasConnected) {
        Telemetry.println(connected ? "PestoLink connected" : "PestoLink disconnected");
        wasConnected = connected;
    }

    float powerX = 0, powerY = 0, rotationPower = 0;
    if (connected) {
        float fieldPowerX = PestoLink.getAxis(0) * driveScale;
        float fieldPowerY = -1 * PestoLink.getAxis(1) * driveScale;
        rotationPower = -1 * PestoLink.getAxis(2) * rotationScale;

        if (fieldCentric) {
            // Rotate joystick vector to be robot-centric
            float heading = NoU3.yaw;
            float cosA = cos(heading);
            float sinA = sin(heading);
            powerX = fieldPowerX * cosA + fieldPowerY * sinA;
            powerY = -fieldPowerX * sinA + fieldPowerY * cosA;
        } else {
            powerX = fieldPowerX;
            powerY = fieldPowerY;
        }

        drivetrain.holonomicDrive(powerX, powerY, rotationPower);
        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.stopMotors();
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    Telemetry.add("power_x", powerX);
    Telemetry.add("power_y", powerY);
    Telemetry.add("power_rotation", rotationPower);
    Telemetry.add("pesto_connected", connected);
    Telemetry.add("battery_v", batteryVoltage);
    Telemetry.send();
}
