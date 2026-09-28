#pragma once

// The USB side of AlfredoTelemetry. Flash the Dongle example to any ESP32
// that plugs into your computer, then open extras/TelemetryViewer/index.html.
//
// The dongle is a bridge: it finds robots, pairs with the one the page picks,
// keeps the link alive, and forwards the robot's packets over USB. The page
// does the decoding.
//
// For the full data rate, use a board with native USB (ESP32-S2, ESP32-S3,
// ESP32-C3, ...) and set Tools > USB CDC On Boot to "Enabled". A board with a
// USB-to-UART chip works too, limited to what `baud` can carry (921600 baud
// is about 90 KB/s).

#include <Arduino.h>
#include <esp_wifi_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

#include "AlfredoTelemetryProtocol.h"

class AlfredoTelemetryDongle {
    public:
        // Starts Serial, Wi-Fi, and ESP-NOW. Use the same Wi-Fi channel as
        // the robot (the page can change it later).
        bool begin(uint8_t wifiChannel = 1, unsigned long baud = 921600);

        // Call from loop() as often as possible.
        void update();

        // Radio bitrate for packets to the robot (they're all small).
        void setRadioRate(wifi_phy_rate_t rate) { _radioRate = rate; }

    private:
        void readHost();
        void handleHostFrame(uint8_t *frame, size_t length);
        void writeFrame(uint8_t type, const uint8_t *a, size_t aLength, const uint8_t *b = nullptr, size_t bLength = 0);
        void flushOut();
        void sendStatus();
        void setTarget(const uint8_t *mac, bool takeover);
        void radioSend(uint8_t type, const uint8_t *payload, size_t length);
        void log(const char *text);

        wifi_phy_rate_t _radioRate = WIFI_PHY_RATE_12M;
        uint8_t _wifiChannel = 1;
        bool _started = false;
        uint8_t _state = atlm::DONGLE_IDLE;
        bool _stream = false;
        bool _takeover = false;  // keep asking a busy robot, with CONNECT_TAKEOVER
        uint16_t _seq = 0;
        uint8_t _ownMac[6] = {0};

        uint32_t _lastPongMs = 0;
        uint32_t _lastStatusMs = 0;
        uint32_t _lastConnectMs = 0;
        uint32_t _lastHeartbeatMs = 0;
        bool _hostWasAlive = false;

        uint8_t _in[2 * atlm::MAX_PAYLOAD];  // COBS frame from the host being received
        size_t _inLength = 0;
        bool _inOverflow = false;

        uint8_t _out[4096];  // encoded frames waiting for Serial.write()
        size_t _outLength = 0;
};

extern AlfredoTelemetryDongle TelemetryDongle;
