#pragma once

// AlfredoTelemetry: high rate robot telemetry over ESP-NOW.
//
// The robot calls Telemetry.add() with whatever it wants to see, and a USB
// dongle (the Dongle example, on any ESP32) passes it to the viewer page in
// extras/TelemetryViewer. See README.md for the whole setup.
//
//   Telemetry.begin("MyRobot");            // in setup()
//   Telemetry.add("yaw", NoU3.yaw);        // in loop(), as often as you like
//   Telemetry.watch("yaw", &NoU3.yaw);     // or: sample a variable automatically
//   Telemetry.tune("kP", &kP);             // edit a variable live from the page
//   Telemetry.printf("mode %d\n", mode);   // text shows up in the page's console
//
// All of the radio work happens in a background task on core 0, so add()
// only copies a value into a buffer and never waits on the radio.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>
#include <type_traits>

#include "AlfredoTelemetryProtocol.h"

#if ESP_IDF_VERSION_MAJOR < 5
#error "AlfredoTelemetry needs the ESP32 Arduino core 3.0 or newer"
#endif

class AlfredoTelemetry : public Print {
    public:
        // Starts Wi-Fi (station mode, no access point needed) and ESP-NOW.
        // The dongle must use the same Wi-Fi channel. If the robot is
        // already connected to a Wi-Fi network, ESP-NOW has to use that
        // network's channel instead, and wifiChannel is ignored.
        bool begin(const char *robotName, uint8_t wifiChannel = 1);

        // Records a value for this loop pass. Numbers, bools, and enums all
        // work. Values added between two send() calls share one timestamp;
        // calling add() again for a name that is already in the frame starts
        // a new one, so send() is optional in a normal loop().
        template <typename T>
        void add(const char *name, T value) {
            static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
                          "Telemetry.add() takes a number, a bool, or an enum");
            addValue(name, typeOf<T>(), encode(value));
        }

        // Ends the current frame. Frames sent faster than setMaxRate() are
        // merged, keeping the latest value of every channel.
        void send();

        // Samples a variable automatically at setWatchRate(), from the
        // telemetry task. Use this for values that other tasks update, like
        // NoU3.yaw. The variable must outlive the sketch (a global or static).
        template <typename T>
        bool watch(const char *name, T *variable) {
            static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
                          "Telemetry.watch() takes a pointer to a number, a bool, or an enum");
            return addWatch(name, typeOf<T>(), kindOf<T>(), (const volatile void *)variable);
        }

        // Lets the viewer page read and change a variable while the robot
        // runs, for tuning gains, trims, and offsets. The page shows the
        // current value, so changes the robot makes itself show up too.
        template <typename T>
        bool tune(const char *name, T *variable) {
            static_assert(std::is_arithmetic<T>::value && !std::is_const<T>::value,
                          "Telemetry.tune() takes a pointer to a number or a bool");
            return addTunable(name, typeOf<T>(), kindOf<T>(), (volatile void *)variable);
        }

        // Settings. Call these before or after begin().
        void setMaxRate(float hz);    // frames per second from add()/send(), default 1000
        void setWatchRate(float hz);  // watch() sample rate, default 100
        // Radio bitrate. Faster rates move more data and take less airtime
        // (which matters when sharing the radio with BLE); slower rates reach
        // farther. Default WIFI_PHY_RATE_12M. The dongle's rate doesn't need to match.
        void setRadioRate(wifi_phy_rate_t rate);
        // Wi-Fi transmit power, e.g. WIFI_POWER_11dBm (the default). Some
        // ESP32 boards, including the Alfredo NoU3 and Rotini, send frames
        // nothing can decode above about 13 dBm, while BLE and receiving still
        // work at any power. If your board's RF works at full power
        // (WIFI_POWER_19_5dBm), raise this for more range.
        void setTxPower(wifi_power_t power);
        // Size of the buffer that absorbs radio hiccups. Call before begin().
        // Default 16384 bytes, about 150 ms of 20 channels at 1 kHz.
        void setBufferSize(size_t bytes);

        // True while a dongle (with the viewer page open) is paired.
        bool isConnected() { return _connected; }
        // True while the page has streaming turned on. add() does nothing
        // when this is false, so telemetry costs almost nothing when unused.
        bool isStreaming() { return _streaming; }

        uint32_t getFramesSent() { return _framesSent; }
        uint32_t getFramesDropped() { return _framesDropped; }

        // Prints one line of link diagnostics: whether telemetry started (and
        // why not), the MAC address, the Wi-Fi channel the radio is really on,
        // HELLOs sent, pairing, frames, and send failures. For debugging over
        // USB: call it from loop() about once a second.
        void printStatus(Print &out);

        // Print support: Telemetry.print(), println(), and printf() send text
        // to the page's console, one line at a time.
        size_t write(uint8_t c) override;
        size_t write(const uint8_t *buffer, size_t size) override;
        using Print::write;

    private:
        // How the telemetry task reads a watched or tuned variable
        enum Kind : uint8_t { K_F32, K_F64, K_I8, K_I16, K_I32, K_I64, K_U8, K_U16, K_U32, K_U64, K_BOOL };

        template <typename T>
        static constexpr uint8_t typeOf() {
            typedef typename std::remove_cv<T>::type U;
            return std::is_same<U, bool>::value ? atlm::TYPE_BOOL
                 : std::is_floating_point<U>::value ? atlm::TYPE_FLOAT
                 : std::is_enum<U>::value ? atlm::TYPE_INT
                 : std::is_signed<U>::value ? atlm::TYPE_INT
                 : atlm::TYPE_UINT;
        }

        template <typename T>
        static constexpr uint8_t kindOf() {
            typedef typename std::remove_cv<T>::type U;
            return std::is_same<U, bool>::value ? K_BOOL
                 : std::is_floating_point<U>::value ? (sizeof(U) == 4 ? K_F32 : K_F64)
                 : (std::is_enum<U>::value || std::is_signed<U>::value)
                     ? (sizeof(U) == 1 ? K_I8 : sizeof(U) == 2 ? K_I16 : sizeof(U) == 4 ? K_I32 : K_I64)
                     : (sizeof(U) == 1 ? K_U8 : sizeof(U) == 2 ? K_U16 : sizeof(U) == 4 ? K_U32 : K_U64);
        }

        template <typename T>
        static uint32_t encode(T value) {
            if constexpr (std::is_same<T, bool>::value) return value ? 1 : 0;
            else if constexpr (std::is_floating_point<T>::value) return floatBits((float)value);
            else if constexpr (std::is_enum<T>::value || std::is_signed<T>::value) return (uint32_t)(int32_t)value;
            else return (uint32_t)value;
        }

        static uint32_t floatBits(float f) {
            uint32_t bits;
            memcpy(&bits, &f, 4);
            return bits;
        }

        struct Channel {
            char name[atlm::MAX_NAME_LENGTH + 1];
            uint8_t type;
        };
        struct Watch {
            const volatile void *variable;
            uint8_t id;
            uint8_t kind;
        };
        struct Tunable {
            char name[atlm::MAX_NAME_LENGTH + 1];
            volatile void *variable;
            uint8_t type;
            uint8_t kind;
        };
        void addValue(const char *name, uint8_t type, uint32_t bits);
        bool addWatch(const char *name, uint8_t type, uint8_t kind, const volatile void *variable);
        bool addTunable(const char *name, uint8_t type, uint8_t kind, volatile void *variable);
        int channelId(const char *name, uint8_t type);
        void endFrame(bool force);
        size_t encodeFrameLocked(uint8_t *out);
        void queueFrame(const uint8_t *item, size_t length);

        static void taskEntry(void *arg);
        void taskLoop();
        void handlePacket(const uint8_t *mac, const uint8_t *data, int length);
        void updateLink(uint32_t now);
        void setPeer(const uint8_t *mac);
        void applyRadioRate();
        void sampleWatches();
        void sendHello();
        void sendStatus();
        void sendSchema();
        void sendTunables();
        void sendText();
        void sendData();
        bool radioSend(const uint8_t *mac, uint8_t type, const uint8_t *payload, size_t length);
        uint32_t readVariable(const volatile void *variable, uint8_t kind);
        void writeVariable(volatile void *variable, uint8_t kind, uint8_t type, uint32_t bits);

        // Settings
        char _name[atlm::MAX_NAME_LENGTH + 1] = "";
        uint8_t _wifiChannel = 1;
        wifi_phy_rate_t _radioRate = WIFI_PHY_RATE_12M;
        wifi_power_t _txPower = WIFI_POWER_11dBm;
        size_t _bufferSize = 16384;
        uint32_t _minFrameUs = 1000;
        uint32_t _watchPeriodUs = 10000;

        // State shared with the telemetry task
        bool _started = false;
        volatile bool _connected = false;
        volatile bool _streaming = false;
        volatile bool _streamRequested = false;
        volatile uint16_t _schemaVersion = 0;
        volatile uint16_t _tunablesVersion = 0;
        volatile uint32_t _framesSent = 0;
        volatile uint32_t _framesDropped = 0;
        volatile uint32_t _txFailures = 0;
        volatile uint32_t _txSuccesses = 0;
        volatile esp_err_t _lastSendError = ESP_OK;  // ESP_ERR_TIMEOUT: no send callback, ESP_FAIL: not delivered
        const char *_error = nullptr;                // why begin() failed
        esp_err_t _errorCode = ESP_OK;
        esp_err_t _channelError = ESP_OK;
        esp_err_t _txPowerError = ESP_OK;
        portMUX_TYPE _lock = portMUX_INITIALIZER_UNLOCKED;

        Channel _channels[atlm::MAX_CHANNELS];
        uint8_t _channelCount = 0;
        uint8_t _lastChannel = 0;
        Watch _watches[atlm::MAX_CHANNELS];
        uint8_t _watchCount = 0;
        Tunable _tunables[atlm::MAX_TUNABLES];
        uint8_t _tunableCount = 0;

        // The frame being built by add()
        uint32_t _frameValues[atlm::MAX_CHANNELS];
        uint64_t _frameMask = 0;  // channels with a value in this frame
        uint64_t _passMask = 0;   // channels added since the last frame boundary
        uint32_t _frameStartUs = 0;
        uint32_t _lastAddUs = 0;
        uint32_t _lastFrameUs = 0;

        // Text being built by write()
        char _line[120];
        uint8_t _lineLength = 0;

        RingbufHandle_t _frameBuffer = nullptr;
        RingbufHandle_t _textBuffer = nullptr;
        TaskHandle_t _task = nullptr;

        // Owned by the telemetry task
        uint8_t _dongleMac[6] = {0};
        bool _hasPeer = false;
        uint16_t _payloadSize = atlm::V1_PAYLOAD;
        uint16_t _seq = 0;
        uint16_t _helloSeq = 0;
        uint32_t _lastHeardMs = 0;
        uint32_t _lastHelloMs = 0;
        uint32_t _lastStatusMs = 0;
        uint32_t _lastTunablesMs = 0;
        uint32_t _nextWatchUs = 0;
        uint16_t _schemaSent = 0xFFFF;
        bool _tunablesRequested = false;
        uint8_t _packet[atlm::MAX_PAYLOAD];
        size_t _packetLength = 0;
        uint32_t _packetStartMs = 0;
        uint8_t _tx[atlm::MAX_PAYLOAD];
};

extern AlfredoTelemetry Telemetry;
