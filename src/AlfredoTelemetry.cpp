#include "AlfredoTelemetry.h"

#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

using namespace atlm;

AlfredoTelemetry Telemetry;

namespace {

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const size_t HEADER_SIZE = sizeof(PacketHeader);
const size_t MAX_FRAME_SIZE = 5 + 5 * MAX_CHANNELS;  // u32 time, u8 count, count x (u8 id, u32 value)
const uint32_t DATA_FLUSH_MS = 10;     // longest a partly full data packet waits for more frames
const uint32_t STALE_FRAME_US = 20000;  // an open frame with no add() for this long is sent on its own
const size_t TEXT_BUFFER_SIZE = 2048;
const uint32_t PROBE_PERIOD_MS = 250;    // how often to try the radio while the link is lost
const uint16_t RATE_WINDOW = 20;         // adaptive rate: packets per decision
const uint16_t RATE_FAILS_TO_DROP = 6;   // first-try failures per window that step the rate down
const uint16_t RATE_CLEAN_TO_RISE = 150; // first-try successes in a row that step it back up (a few seconds)

// Control packets from the dongle are small; anything bigger isn't ours.
struct RxPacket {
    uint8_t mac[6];
    uint8_t length;
    uint8_t data[32];
};

QueueHandle_t rxQueue = nullptr;
SemaphoreHandle_t txDone = nullptr;
volatile bool txOk = false;

// Runs in the Wi-Fi task, so it only copies the packet out.
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
    if (rxQueue == nullptr || length < (int)HEADER_SIZE || length > 32 || data[0] != MAGIC) return;
    if (data[1] < PKT_CONNECT || data[1] > PKT_DISCONNECT) return;  // e.g. other robots' HELLOs
    RxPacket packet;
    memcpy(packet.mac, info->src_addr, 6);
    packet.length = length;
    memcpy(packet.data, data, length);
    xQueueSend(rxQueue, &packet, 0);
}

void sentDone(bool ok) {
    txOk = ok;
    if (txDone) xSemaphoreGive(txDone);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) { sentDone(status == ESP_NOW_SEND_SUCCESS); }
#else
void onSent(const uint8_t *, esp_now_send_status_t status) { sentDone(status == ESP_NOW_SEND_SUCCESS); }
#endif

wifi_phy_mode_t phyModeFor(wifi_phy_rate_t rate) {
    if (rate <= WIFI_PHY_RATE_11M_S) return WIFI_PHY_MODE_11B;
    if (rate <= WIFI_PHY_RATE_9M) return WIFI_PHY_MODE_11G;
    if (rate <= WIFI_PHY_RATE_MCS7_SGI) return WIFI_PHY_MODE_HT20;
    return WIFI_PHY_MODE_LR;
}

void putU16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
void putU32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

// Approximate speed of a wifi_phy_rate_t, to order the adaptive-rate ladder
uint32_t rateKbps(wifi_phy_rate_t rate) {
    static const uint16_t legacy[16] = {1000, 2000, 5500, 11000, 0, 2000, 5500, 11000, 48000, 24000, 12000, 6000, 54000, 36000, 18000, 9000};
    static const uint16_t mcs[8] = {6500, 13000, 19500, 26000, 39000, 52000, 58500, 65000};
    uint8_t r = rate;
    if (r < 16) return legacy[r];
    if (r >= 0x10 && r <= 0x1F) return mcs[(r - 0x10) & 7];
    if (r == WIFI_PHY_RATE_LORA_250K) return 250;
    if (r == WIFI_PHY_RATE_LORA_500K) return 500;
    return 1000;
}

}  // namespace

bool AlfredoTelemetry::begin(const char *robotName, uint8_t wifiChannel) {
    if (_started) return true;
    strlcpy(_name, (robotName && robotName[0]) ? robotName : "Robot", sizeof(_name));
    _wifiChannel = wifiChannel;

    // Station mode is all ESP-NOW needs. Keep AP mode if the sketch uses it.
    wifi_mode_t mode = WiFi.getMode();
    if (!(mode & WIFI_MODE_STA)) {
        if (!WiFi.mode((wifi_mode_t)(mode | WIFI_MODE_STA))) {
            _error = "Wi-Fi didn't start";
            log_e("Telemetry: %s", _error);
            return false;
        }
    }
    // Wi-Fi power save is left at the core's default on purpose: when BLE
    // runs too, ESP-IDF requires modem sleep for coexistence and turning it
    // off can abort. The robot mostly transmits, which modem sleep doesn't delay.
    if (!WiFi.isConnected()) _channelError = esp_wifi_set_channel(_wifiChannel, WIFI_SECOND_CHAN_NONE);
    _txPowerError = esp_wifi_set_max_tx_power(_txPower);  // only works once Wi-Fi has started
    if (_longRange) applyProtocol();

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        _error = "esp_now_init failed";
        _errorCode = err;
        log_e("Telemetry: %s: %s", _error, esp_err_to_name(err));
        return false;
    }

    rxQueue = xQueueCreate(16, sizeof(RxPacket));
    txDone = xSemaphoreCreateBinary();
    // The frame buffer absorbs radio hiccups and whole dropouts. With PSRAM
    // it can hold minutes of data; otherwise 16 KB is a few seconds.
    if (_bufferSize == 0) {
        if (psramFound()) _bufferSize = 1u << 20;
        else _bufferSize = ESP.getFreeHeap() > 200000 ? 65536 : 16384;  // about 10 s or 2.5 s of the IMU example
    }
    if (_bufferSize > 65536 && psramFound()) _frameBuffer = xRingbufferCreateWithCaps(_bufferSize, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    if (_frameBuffer == nullptr) {
        if (_bufferSize > 65536) _bufferSize = 16384;  // no PSRAM after all
        _frameBuffer = xRingbufferCreate(_bufferSize, RINGBUF_TYPE_NOSPLIT);
    }
    _textBuffer = xRingbufferCreate(TEXT_BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (!rxQueue || !txDone || !_frameBuffer || !_textBuffer) {
        _error = "out of memory";
        log_e("Telemetry: %s", _error);
        return false;
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;  // whatever channel the radio is on
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(BROADCAST_MAC)) {
        err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            _error = "couldn't add the broadcast peer";
            _errorCode = err;
            log_e("Telemetry: %s: %s", _error, esp_err_to_name(err));
            return false;
        }
    }

    _started = true;
    // Core 0 is where Wi-Fi and BLE already live; loop() and most robot
    // tasks (NoU3's IMU, PestoLink) run on core 1. The ESP32-S2 only has core 0.
    xTaskCreatePinnedToCore(taskEntry, "taskTelemetry", 4096, this, 2, &_task, 0);
    return true;
}

void AlfredoTelemetry::setMaxRate(float hz) { _minFrameUs = hz > 0 ? (uint32_t)(1000000.0f / hz) : 0; }

void AlfredoTelemetry::setWatchRate(float hz) {
    if (hz <= 0) return;
    if (hz > 1000) hz = 1000;  // the telemetry task runs on the 1 ms FreeRTOS tick
    _watchPeriodUs = (uint32_t)(1000000.0f / hz);
}

void AlfredoTelemetry::setRadioRate(wifi_phy_rate_t rate) {
    _radioRate = rate;
    buildLadder();
    if (_hasPeer) applyRadioRate();  // takes effect on the current link too
}

void AlfredoTelemetry::setRetries(uint8_t retries) { _retries = retries > 10 ? 10 : retries; }

void AlfredoTelemetry::setDropoutGrace(float seconds) { _graceMs = seconds <= 0 ? 0 : (uint32_t)(seconds * 1000.0f); }

void AlfredoTelemetry::setAdaptiveRate(bool on) {
    _adaptive = on;
    buildLadder();
    if (_hasPeer) applyRadioRate();
}

// The rates the adaptive logic may fall back to, fastest first
void AlfredoTelemetry::buildLadder() {
    _ladderCount = 0;
    _ladder[_ladderCount++] = _radioRate;
    if (_adaptive) {
        if (_radioRate == WIFI_PHY_RATE_LORA_500K) {
            _ladder[_ladderCount++] = WIFI_PHY_RATE_LORA_250K;
        } else if (_radioRate != WIFI_PHY_RATE_LORA_250K) {
            const wifi_phy_rate_t steps[] = {WIFI_PHY_RATE_6M, WIFI_PHY_RATE_2M_L, WIFI_PHY_RATE_1M_L};
            for (wifi_phy_rate_t step : steps) {
                if (rateKbps(step) < rateKbps(_radioRate)) _ladder[_ladderCount++] = step;
            }
        }
    }
    _rateIndex = 0;
    _activeRate = _radioRate;
    _firstTryCount = _firstTryFails = _cleanStreak = 0;
}

// Called once per data packet with whether the first try was acknowledged.
void AlfredoTelemetry::noteDataResult(bool firstTryOk) {
    if (_ladderCount < 2) return;
    if (firstTryOk) {
        if (_cleanStreak < 0xFFFF) _cleanStreak++;
    } else {
        _cleanStreak = 0;
        _firstTryFails++;
    }
    if (++_firstTryCount >= RATE_WINDOW) {
        if (_firstTryFails >= RATE_FAILS_TO_DROP && _rateIndex + 1 < _ladderCount) {
            _rateIndex++;
            _activeRate = _ladder[_rateIndex];
            applyRadioRate();
        }
        _firstTryCount = _firstTryFails = 0;
    }
    if (_rateIndex > 0 && _cleanStreak >= RATE_CLEAN_TO_RISE) {
        _rateIndex--;
        _activeRate = _ladder[_rateIndex];
        applyRadioRate();
        _cleanStreak = 0;
    }
}

void AlfredoTelemetry::setLongRange(bool on) {
    _longRange = on;
    bool lrRate = _radioRate == WIFI_PHY_RATE_LORA_250K || _radioRate == WIFI_PHY_RATE_LORA_500K;
    if (on && !lrRate) _radioRate = WIFI_PHY_RATE_LORA_250K;
    if (!on && lrRate) _radioRate = WIFI_PHY_RATE_12M;
    buildLadder();
    if (_started) {
        applyProtocol();
        if (_hasPeer) applyRadioRate();
    }
}

// 802.11b/g/n plus LR: the radio still hears normal frames (HELLOs, and
// dongles that don't use LR), and can send and receive LR frames.
void AlfredoTelemetry::applyProtocol() {
    uint8_t protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
    if (_longRange) protocol |= WIFI_PROTOCOL_LR;
    _protocolError = esp_wifi_set_protocol(WIFI_IF_STA, protocol);
}

void AlfredoTelemetry::applyRadioRate() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    esp_now_rate_config_t rate = {};
    rate.phymode = phyModeFor(_activeRate);
    rate.rate = _activeRate;
    _rateError = esp_now_set_peer_rate_config(_dongleMac, &rate);
#else
    _rateError = esp_wifi_config_espnow_rate(WIFI_IF_STA, _activeRate);
#endif
}

void AlfredoTelemetry::setTxPower(wifi_power_t power) {
    _txPower = power;
    if (_started) _txPowerError = esp_wifi_set_max_tx_power(power);
}

void AlfredoTelemetry::setBufferSize(size_t bytes) {
    if (!_started && bytes >= 1024) _bufferSize = bytes;
}

// ---------------------------------------------------------------- channels

int AlfredoTelemetry::channelId(const char *name, uint8_t type) {
    if (name == nullptr || name[0] == '\0') return -1;

    // Channels are only ever appended, so reading them without the lock is
    // safe. A loop() adds its channels in the same order every pass, so the
    // channel after the last one used is almost always the right one.
    int count = _channelCount;
    int hint = _lastChannel + 1;
    if (hint < count && strncmp(_channels[hint].name, name, MAX_NAME_LENGTH) == 0) {
        _lastChannel = hint;
        return hint;
    }
    for (int i = 0; i < count; i++) {
        if (strncmp(_channels[i].name, name, MAX_NAME_LENGTH) == 0) {
            _lastChannel = i;
            return i;
        }
    }

    // New channel. Check again under the lock in case another task just added it.
    int id = -1;
    portENTER_CRITICAL(&_lock);
    for (int i = count; i < _channelCount; i++) {
        if (strncmp(_channels[i].name, name, MAX_NAME_LENGTH) == 0) id = i;
    }
    if (id < 0 && _channelCount < MAX_CHANNELS) {
        id = _channelCount;
        strlcpy(_channels[id].name, name, sizeof(_channels[id].name));
        _channels[id].type = type;
        _channelCount++;
        _schemaVersion = _schemaVersion + 1;
    }
    portEXIT_CRITICAL(&_lock);
    if (id >= 0) _lastChannel = id;
    return id;
}

void AlfredoTelemetry::addValue(const char *name, uint8_t type, uint32_t bits) {
    if (!_started) return;
    int id = channelId(name, type);
    if (id < 0 || !_streaming) return;

    // Values are stored with the channel's type, so a channel first added
    // as a float stays a float.
    uint8_t channelType = _channels[id].type;
    if (channelType != type) {
        if (channelType == TYPE_FLOAT) {
            float f;
            if (type == TYPE_INT) f = (float)(int32_t)bits;
            else f = (float)bits;
            bits = floatBits(f);
        } else if (type == TYPE_FLOAT) {
            float f;
            memcpy(&f, &bits, 4);
            bits = channelType == TYPE_UINT ? (uint32_t)f : (uint32_t)(int32_t)f;
        }
    }

    uint8_t item[MAX_FRAME_SIZE];
    size_t length = 0;
    uint64_t bit = 1ULL << id;

    portENTER_CRITICAL(&_lock);
    uint32_t now = micros();  // inside the lock, so frame times never go backwards
    // Seeing a channel twice means a new pass of loop() started, so the
    // previous frame is complete.
    if (_passMask & bit) {
        _passMask = 0;
        if (now - _lastFrameUs >= _minFrameUs) {
            length = encodeFrameLocked(item);
            _lastFrameUs = now;
        } else {
            _frameStartUs = now;  // too soon: merge this pass into the frame
        }
    }
    if (_frameMask == 0) _frameStartUs = now;
    _frameMask |= bit;
    _passMask |= bit;
    _frameValues[id] = bits;
    _lastAddUs = now;
    portEXIT_CRITICAL(&_lock);

    if (length) queueFrame(item, length);
}

void AlfredoTelemetry::send() {
    if (_started && _streaming) endFrame(false);
}

void AlfredoTelemetry::endFrame(bool force) {
    uint8_t item[MAX_FRAME_SIZE];
    size_t length = 0;

    portENTER_CRITICAL(&_lock);
    uint32_t now = micros();
    _passMask = 0;
    if (_frameMask != 0) {
        if (force || now - _lastFrameUs >= _minFrameUs) {
            length = encodeFrameLocked(item);
            _lastFrameUs = now;
        } else {
            _frameStartUs = now;
        }
    }
    portEXIT_CRITICAL(&_lock);

    if (length) queueFrame(item, length);
}

// Encodes the open frame and clears it. Call with _lock held.
size_t AlfredoTelemetry::encodeFrameLocked(uint8_t *out) {
    putU32(out, _frameStartUs);
    uint8_t count = 0;
    size_t p = 5;
    uint64_t mask = _frameMask;
    while (mask) {
        int id = __builtin_ctzll(mask);
        mask &= mask - 1;
        out[p] = id;
        memcpy(out + p + 1, &_frameValues[id], 4);
        p += 5;
        count++;
    }
    out[4] = count;
    _frameMask = 0;
    return p;
}

void AlfredoTelemetry::queueFrame(const uint8_t *item, size_t length) {
    if (xRingbufferSend(_frameBuffer, item, length, 0) == pdTRUE) _framesSent = _framesSent + 1;
    else _framesDropped = _framesDropped + 1;
}

bool AlfredoTelemetry::addWatch(const char *name, uint8_t type, uint8_t kind, const volatile void *variable) {
    if (variable == nullptr) return false;
    int id = channelId(name, type);
    if (id < 0) return false;
    bool ok = false;
    portENTER_CRITICAL(&_lock);
    if (_watchCount < MAX_CHANNELS) {
        _watches[_watchCount] = {variable, (uint8_t)id, kind};
        _watchCount++;
        ok = true;
    }
    portEXIT_CRITICAL(&_lock);
    return ok;
}

bool AlfredoTelemetry::addTunable(const char *name, uint8_t type, uint8_t kind, volatile void *variable) {
    if (variable == nullptr || name == nullptr || name[0] == '\0') return false;
    bool ok = false;
    portENTER_CRITICAL(&_lock);
    if (_tunableCount < MAX_TUNABLES) {
        Tunable &t = _tunables[_tunableCount];
        strlcpy(t.name, name, sizeof(t.name));
        t.variable = variable;
        t.type = type;
        t.kind = kind;
        _tunableCount++;
        _tunablesVersion = _tunablesVersion + 1;
        _tunablesRequested = true;
        ok = true;
    }
    portEXIT_CRITICAL(&_lock);
    return ok;
}

uint32_t AlfredoTelemetry::readVariable(const volatile void *v, uint8_t kind) {
    switch (kind) {
        case K_F32: return floatBits(*(const volatile float *)v);
        case K_F64: return floatBits((float)*(const volatile double *)v);
        case K_I8: return (uint32_t)(int32_t) * (const volatile int8_t *)v;
        case K_I16: return (uint32_t)(int32_t) * (const volatile int16_t *)v;
        case K_I32: return (uint32_t) * (const volatile int32_t *)v;
        case K_I64: return (uint32_t)(int32_t) * (const volatile int64_t *)v;
        case K_U8: return *(const volatile uint8_t *)v;
        case K_U16: return *(const volatile uint16_t *)v;
        case K_U32: return *(const volatile uint32_t *)v;
        case K_U64: return (uint32_t) * (const volatile uint64_t *)v;
        case K_BOOL: return *(const volatile bool *)v ? 1 : 0;
    }
    return 0;
}

// The page sends float bits for float tunables and an int32 for the rest.
void AlfredoTelemetry::writeVariable(volatile void *v, uint8_t kind, uint8_t type, uint32_t bits) {
    float f;
    memcpy(&f, &bits, 4);
    int32_t i = (int32_t)bits;
    if (type == TYPE_FLOAT) {
        if (kind == K_F32) *(volatile float *)v = f;
        else if (kind == K_F64) *(volatile double *)v = f;
        return;
    }
    switch (kind) {
        case K_I8: *(volatile int8_t *)v = i; break;
        case K_I16: *(volatile int16_t *)v = i; break;
        case K_I32: *(volatile int32_t *)v = i; break;
        case K_I64: *(volatile int64_t *)v = i; break;
        case K_U8: *(volatile uint8_t *)v = bits; break;
        case K_U16: *(volatile uint16_t *)v = bits; break;
        case K_U32: *(volatile uint32_t *)v = bits; break;
        case K_U64: *(volatile uint64_t *)v = bits; break;
        case K_BOOL: *(volatile bool *)v = bits != 0; break;
    }
}

// ---------------------------------------------------------------- text

size_t AlfredoTelemetry::write(uint8_t c) {
    if (!_started || c == '\r') return 1;
    char line[sizeof(_line)];
    size_t length = 0;

    portENTER_CRITICAL(&_lock);
    if (c != '\n') _line[_lineLength++] = c;
    if (c == '\n' || _lineLength >= sizeof(_line)) {
        length = _lineLength;
        memcpy(line, _line, length);
        _lineLength = 0;
    }
    portEXIT_CRITICAL(&_lock);

    // Empty lines are kept: the page shows one per packet item.
    if ((length || c == '\n') && (_connected || _streaming)) {
        if (length == 0) line[length++] = ' ';
        xRingbufferSend(_textBuffer, line, length, 0);
    }
    return 1;
}

size_t AlfredoTelemetry::write(const uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) write(buffer[i]);
    return size;
}

// ---------------------------------------------------------------- telemetry task

void AlfredoTelemetry::taskEntry(void *arg) { ((AlfredoTelemetry *)arg)->taskLoop(); }

void AlfredoTelemetry::taskLoop() {
    RxPacket packet;
    while (true) {
        // Waiting on the receive queue is the task's 1 ms tick.
        if (xQueueReceive(rxQueue, &packet, 1) == pdTRUE) {
            do {
                handlePacket(packet.mac, packet.data, packet.length);
            } while (xQueueReceive(rxQueue, &packet, 0) == pdTRUE);
        }

        uint32_t now = millis();
        updateLink(now);

        if (!_connected) {
            if (now - _lastHelloMs >= HELLO_PERIOD_MS) sendHello();
            if (_streaming) {
                // Link lost but within the grace period: keep recording, and
                // keep trying the radio so the backlog goes as soon as it can.
                sampleWatches();
                if (_frameMask && micros() - _lastAddUs > STALE_FRAME_US) endFrame(true);
                sendData();
            }
            continue;
        }

        // The schema goes out before any data that uses new channels.
        if (now - _lastHelloMs >= PAIRED_HELLO_PERIOD_MS) sendHello();
        if (_schemaSent != _schemaVersion) sendSchema();
        if (_tunablesRequested || (_tunableCount && now - _lastTunablesMs >= TUNABLES_PERIOD_MS)) sendTunables();
        if (now - _lastStatusMs >= STATUS_PERIOD_MS) sendStatus();

        if (_streaming) {
            sampleWatches();
            if (_frameMask && micros() - _lastAddUs > STALE_FRAME_US) endFrame(true);
            sendData();
        }
        sendText();
    }
}

void AlfredoTelemetry::handlePacket(const uint8_t *mac, const uint8_t *data, int length) {
    const uint8_t type = data[1];
    const uint8_t *payload = data + HEADER_SIZE;
    const int payloadLength = length - HEADER_SIZE;
    // The paired dongle, even while its heartbeats have lapsed: one heartbeat
    // re-links without a new CONNECT.
    const bool fromDongle = _hasPeer && memcmp(mac, _dongleMac, 6) == 0;
    uint32_t now = millis();

    if (type == PKT_CONNECT) {
        // The first dongle keeps the robot. Another dongle only gets it once
        // this link times out, or by asking for a takeover (the page's Take
        // over button). The HELLO below tells it the robot is busy.
        bool takeover = payloadLength >= 4 && (payload[3] & CONNECT_TAKEOVER);
        if (_connected && !fromDongle && !takeover) return;
        uint16_t donglePayload = V1_PAYLOAD;
        if (payloadLength >= 3) memcpy(&donglePayload, payload + 1, 2);
        setPeer(mac);
        _payloadSize = donglePayload < MAX_PAYLOAD ? donglePayload : MAX_PAYLOAD;
        if (_longRange && _payloadSize > V1_PAYLOAD) _payloadSize = V1_PAYLOAD;  // 250 B is 8 ms of airtime at 250 kbps
        if (_payloadSize < 64) _payloadSize = 64;
        // A backlog recorded during a dropout is for this same dongle; keep it.
        if (!fromDongle || _packetLength > _payloadSize - HEADER_SIZE) discardFrames();
        _connected = true;
        _linkLost = false;
        _lastHeardMs = now;
        _schemaSent = 0xFFFF;
        _tunablesRequested = true;
        _lastStatusMs = now - STATUS_PERIOD_MS;  // send STATUS right away
        _lastHelloMs = now - PAIRED_HELLO_PERIOD_MS;  // and a HELLO, so other dongles see who has the robot
        return;
    }
    // Everything else only counts from the paired dongle. A dongle that
    // thinks it's paired after the robot reset hears our HELLO and reconnects.
    if (!fromDongle) return;
    _lastHeardMs = now;

    if (type == PKT_HEARTBEAT && payloadLength >= 1) {
        bool want = payload[0] & FLAG_STREAM;
        if (want && !_streaming) {
            portENTER_CRITICAL(&_lock);
            _frameMask = 0;
            _passMask = 0;
            portEXIT_CRITICAL(&_lock);
            _nextWatchUs = micros();
            discardFrames();
        }
        _streamRequested = want;
        _connected = true;
        _linkLost = false;
    } else if (type == PKT_REQUEST && payloadLength >= 1) {
        if (payload[0] & REQUEST_SCHEMA) _schemaSent = 0xFFFF;
        if (payload[0] & REQUEST_TUNABLES) _tunablesRequested = true;
    } else if (type == PKT_SET_TUNABLE && payloadLength >= 5) {
        uint8_t id = payload[0];
        uint32_t bits;
        memcpy(&bits, payload + 1, 4);
        if (id < _tunableCount) {
            Tunable &t = _tunables[id];
            writeVariable(t.variable, t.kind, t.type, bits);
        }
        _tunablesRequested = true;  // echo the new value back
    } else if (type == PKT_DISCONNECT) {
        // An explicit release (the page closed or unpaired): stop right away.
        _connected = false;
        _streamRequested = false;
        _linkLost = false;
    }
}

void AlfredoTelemetry::updateLink(uint32_t now) {
    if (_connected && now - _lastHeardMs > LINK_TIMEOUT_MS) {
        _connected = false;
        if (_streamRequested) {
            _linkLost = true;
            _linkLostMs = now;
            _nextProbeMs = now;
        }
    }
    if (_linkLost && now - _linkLostMs > _graceMs) {
        _linkLost = false;
        _streamRequested = false;
    }
    bool streaming = _streamRequested && (_connected || _linkLost);
    if (_streaming && !streaming) discardFrames();
    _streaming = streaming;
}

// Throws away queued frames and the packet being built
void AlfredoTelemetry::discardFrames() {
    if (_carryItem) {
        vRingbufferReturnItem(_frameBuffer, _carryItem);
        _carryItem = nullptr;
    }
    size_t size;
    void *item;
    while ((item = xRingbufferReceive(_frameBuffer, &size, 0))) vRingbufferReturnItem(_frameBuffer, item);
    _packetLength = 0;
}

void AlfredoTelemetry::setPeer(const uint8_t *mac) {
    if (_hasPeer && memcmp(mac, _dongleMac, 6) == 0) return;
    if (_hasPeer) esp_now_del_peer(_dongleMac);
    memcpy(_dongleMac, mac, 6);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    _hasPeer = esp_now_add_peer(&peer) == ESP_OK;
    if (_hasPeer) applyRadioRate();
}

void AlfredoTelemetry::sampleWatches() {
    uint8_t count = _watchCount;
    if (count == 0) return;
    uint32_t now = micros();
    if ((int32_t)(now - _nextWatchUs) < 0) return;
    _nextWatchUs += _watchPeriodUs;
    if ((int32_t)(now - _nextWatchUs) >= 0) _nextWatchUs = now + _watchPeriodUs;  // fell behind, skip ahead

    uint8_t item[MAX_FRAME_SIZE];
    putU32(item, now);
    item[4] = count;
    size_t p = 5;
    for (int i = 0; i < count; i++) {
        item[p] = _watches[i].id;
        putU32(item + p + 1, readVariable(_watches[i].variable, _watches[i].kind));
        p += 5;
    }
    queueFrame(item, p);
}

// ---------------------------------------------------------------- radio packets

bool AlfredoTelemetry::radioSend(const uint8_t *mac, uint8_t type, const uint8_t *payload, size_t length, uint8_t retries,
                                 bool *firstTryOk) {
    // HELLOs have their own count so the host's loss count only sees packets meant for it.
    // The sequence number only advances once a packet is delivered, so a
    // packet sent again later keeps its number and the page doesn't count it
    // as lost. (A delivered-but-unacknowledged packet arrives twice; the page
    // merges rows with the same timestamp.)
    PacketHeader header = {MAGIC, type, type == PKT_HELLO ? _helloSeq : _seq};
    memcpy(_tx, &header, HEADER_SIZE);
    if (payload != _tx + HEADER_SIZE) memcpy(_tx + HEADER_SIZE, payload, length);
    if (firstTryOk) *firstTryOk = false;

    for (uint8_t attempt = 0;; attempt++) {
        // One packet in flight at a time: wait for the send callback, which
        // comes after the dongle's ACK (or after the driver gives up retrying).
        xSemaphoreTake(txDone, 0);
        esp_err_t err = esp_now_send(mac, _tx, HEADER_SIZE + length);
        if (err == ESP_OK) {
            if (xSemaphoreTake(txDone, pdMS_TO_TICKS(50)) != pdTRUE) err = ESP_ERR_TIMEOUT;
            else if (!txOk) err = ESP_FAIL;
        }
        if (err == ESP_OK) {
            _txSuccesses = _txSuccesses + 1;
            if (type == PKT_HELLO) _helloSeq++; else _seq++;
            if (attempt == 0 && firstTryOk) *firstTryOk = true;
            return true;
        }
        _lastSendError = err;
        _txFailures = _txFailures + 1;
        if (attempt >= retries) return false;
        // The driver already retried quickly; waiting a little longer each
        // time rides out a brief fade or a burst of interference.
        _retransmits = _retransmits + 1;
        vTaskDelay(pdMS_TO_TICKS(20 * (attempt + 1)));
    }
}

void AlfredoTelemetry::printStatus(Print &out) {
    if (!_started) {
        if (_error) out.printf("Telemetry: NOT STARTED - %s%s%s\n", _error, _errorCode != ESP_OK ? ": " : "",
                               _errorCode != ESP_OK ? esp_err_to_name(_errorCode) : "");
        else out.println("Telemetry: not started (call Telemetry.begin())");
        return;
    }
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    uint8_t channel = 0;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&channel, &second);

    out.printf("Telemetry '%s': mac %02x:%02x:%02x:%02x:%02x:%02x, ch %u", _name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel);
    if (channel != _wifiChannel) out.printf(" (asked for %u%s)", _wifiChannel, WiFi.isConnected() ? ", using the Wi-Fi network's" : "");
    if (_channelError != ESP_OK) out.printf(" [set channel failed: %s]", esp_err_to_name(_channelError));
    int8_t power = 0;
    esp_wifi_get_max_tx_power(&power);
    out.printf(", tx %.1f dBm", power / 4.0f);
    if (_txPowerError != ESP_OK) out.printf(" [set tx power failed: %s]", esp_err_to_name(_txPowerError));
    if (_longRange) out.printf(", long range %s", _radioRate == WIFI_PHY_RATE_LORA_500K ? "500k" : "250k");
    if (_protocolError != ESP_OK) out.printf(" [set protocol failed: %s]", esp_err_to_name(_protocolError));
    out.printf(", rate 0x%02x", _activeRate);
    if (_activeRate != _radioRate) out.printf(" (stepped down from 0x%02x)", _radioRate);
    if (_rateError != ESP_OK) out.printf(" [set rate failed: %s]", esp_err_to_name(_rateError));
    out.printf(", buffer %u KB%s", (unsigned)(_bufferSize / 1024), psramFound() ? " in PSRAM" : "");
    out.printf(", HELLOs %u", _helloSeq);
    if (_connected) {
        out.printf(" | paired with %02x:%02x:%02x:%02x:%02x:%02x, %s", _dongleMac[0], _dongleMac[1], _dongleMac[2], _dongleMac[3],
                   _dongleMac[4], _dongleMac[5], _streaming ? "streaming" : "not streaming");
    } else if (_streaming) {
        out.printf(" | link lost %lu s ago, still recording", (unsigned long)((millis() - _linkLostMs) / 1000));
    } else {
        out.print(" | not paired");
    }
    out.printf(" | frames %lu sent, %lu dropped | sends %lu ok, %lu failed, %lu retries", (unsigned long)_framesSent,
               (unsigned long)_framesDropped, (unsigned long)_txSuccesses, (unsigned long)_txFailures, (unsigned long)_retransmits);
    if (_lastSendError != ESP_OK) {
        esp_err_t e = _lastSendError;
        out.printf(" (last: %s)", e == ESP_ERR_TIMEOUT ? "no send callback" : e == ESP_FAIL ? "not delivered" : esp_err_to_name(e));
    }
    out.println();
}

void AlfredoTelemetry::sendHello() {
    _lastHelloMs = millis();
    uint8_t *p = _tx + HEADER_SIZE;
    uint8_t nameLength = strlen(_name);
    p[0] = PROTOCOL_VERSION;
    p[1] = _connected ? HELLO_PAIRED : 0;
    putU16(p + 2, MAX_PAYLOAD);
    p[4] = nameLength;
    memcpy(p + 5, _name, nameLength);
    if (_connected) memcpy(p + 5 + nameLength, _dongleMac, 6);
    else memset(p + 5 + nameLength, 0, 6);
    radioSend(BROADCAST_MAC, PKT_HELLO, p, 5 + nameLength + 6);
}

void AlfredoTelemetry::sendStatus() {
    _lastStatusMs = millis();
    uint8_t *p = _tx + HEADER_SIZE;
    uint8_t nameLength = strlen(_name);
    // Bytes between the free and write positions are in use. (The "current
    // free size" call can't be used here: for this kind of ring buffer it
    // never reports more than half the buffer, even when it's empty.)
    UBaseType_t freePos, readPos, writePos, acquirePos, itemsWaiting;
    vRingbufferGetInfo(_frameBuffer, &freePos, &readPos, &writePos, &acquirePos, &itemsWaiting);
    size_t used = (writePos + _bufferSize - freePos) % _bufferSize;
    if (used == 0 && (itemsWaiting > 0 || readPos != freePos)) used = _bufferSize;  // full, not empty
    uint8_t bufferUse = (used * 100) / _bufferSize;
    putU16(p + 0, _schemaVersion);
    putU16(p + 2, _tunablesVersion);
    putU32(p + 4, millis());
    putU32(p + 8, _framesSent);
    putU32(p + 12, _framesDropped);
    putU32(p + 16, _txFailures);
    p[20] = _streaming;
    p[21] = bufferUse;
    putU16(p + 22, _payloadSize);
    p[24] = _channelCount;
    p[25] = _tunableCount;
    putU32(p + 26, _retransmits);
    p[30] = _activeRate;
    p[31] = _rateIndex > 0 ? 1 : 0;  // bit 0: rate stepped down
    p[32] = nameLength;
    memcpy(p + 33, _name, nameLength);
    radioSend(_dongleMac, PKT_STATUS, p, 33 + nameLength);
}

// Sends every channel, split over as many packets as it takes.
void AlfredoTelemetry::sendSchema() {
    uint16_t version = _schemaVersion;
    int count = _channelCount;
    int i = 0;
    do {
        uint8_t *p = _tx + HEADER_SIZE;
        size_t room = _payloadSize - HEADER_SIZE;
        putU16(p, version);
        p[2] = count;
        size_t length = 3;
        while (i < count) {
            size_t nameLength = strlen(_channels[i].name);
            if (length + 3 + nameLength > room) break;
            p[length++] = i;
            p[length++] = _channels[i].type;
            p[length++] = nameLength;
            memcpy(p + length, _channels[i].name, nameLength);
            length += nameLength;
            i++;
        }
        if (!radioSend(_dongleMac, PKT_SCHEMA, p, length, _retries)) return;  // retried next pass
    } while (i < count);
    _schemaSent = version;
}

void AlfredoTelemetry::sendTunables() {
    _lastTunablesMs = millis();
    _tunablesRequested = false;
    uint16_t version = _tunablesVersion;
    int count = _tunableCount;
    int i = 0;
    do {
        uint8_t *p = _tx + HEADER_SIZE;
        size_t room = _payloadSize - HEADER_SIZE;
        putU16(p, version);
        p[2] = count;
        size_t length = 3;
        while (i < count) {
            const Tunable &t = _tunables[i];
            size_t nameLength = strlen(t.name);
            if (length + 7 + nameLength > room) break;
            p[length++] = i;
            p[length++] = t.type;
            putU32(p + length, readVariable(t.variable, t.kind));
            length += 4;
            p[length++] = nameLength;
            memcpy(p + length, t.name, nameLength);
            length += nameLength;
            i++;
        }
        radioSend(_dongleMac, PKT_TUNABLES, p, length, _retries);
    } while (i < count);
}

void AlfredoTelemetry::sendText() {
    uint8_t *p = _tx + HEADER_SIZE;
    size_t room = _payloadSize - HEADER_SIZE;
    size_t length = 0;
    // Each line is one item; lines are separated by '\n' in the packet.
    while (length + sizeof(_line) + 1 <= room) {
        size_t size;
        char *item = (char *)xRingbufferReceive(_textBuffer, &size, 0);
        if (item == nullptr) break;
        memcpy(p + length, item, size);
        length += size;
        p[length++] = '\n';
        vRingbufferReturnItem(_textBuffer, item);
    }
    if (length) radioSend(_dongleMac, PKT_TEXT, p, length, _retries);
}

// Packs queued frames into DATA packets. A packet goes out when it's full,
// or DATA_FLUSH_MS after its first frame, whichever comes first. A packet
// that can't be sent is kept and tried again, so nothing is lost in order.
void AlfredoTelemetry::sendData() {
    const size_t room = _payloadSize - HEADER_SIZE;
    // Only a few packets per call, so heartbeats and requests from the
    // dongle keep getting handled even when frames arrive faster than the
    // radio can send them.
    for (int sent = 0; sent < 4;) {
        if (_packetLength > 0 && room - _packetLength < 10) {  // full
            if (!flushPacket()) return;
            sent++;
            continue;
        }
        if (_carryItem == nullptr) {
            size_t size;
            _carryItem = (uint8_t *)xRingbufferReceive(_frameBuffer, &size, 0);
            if (_carryItem == nullptr) break;
            _carryLeft = size >= 5 ? _carryItem[4] : 0;
            _carryValues = _carryItem + 5;
        }
        if (_carryLeft == 0) {
            vRingbufferReturnItem(_frameBuffer, _carryItem);
            _carryItem = nullptr;
            continue;
        }
        if (_packetLength == 0) {
            _packetLength = 2;  // room for the schema version
            _packetStartMs = millis();
        }
        // A frame that doesn't fit in what's left of the packet is split;
        // each piece carries the frame's timestamp.
        size_t free = room - _packetLength;
        uint8_t k = free >= 10 ? (free - 5) / 5 : 0;
        if (k == 0) {
            if (!flushPacket()) return;
            sent++;
            continue;
        }
        if (k > _carryLeft) k = _carryLeft;
        memcpy(_packet + _packetLength, _carryItem, 4);
        _packet[_packetLength + 4] = k;
        memcpy(_packet + _packetLength + 5, _carryValues, k * 5);
        _packetLength += 5 + k * 5;
        _carryValues += k * 5;
        _carryLeft -= k;
    }
    if (_packetLength > 2 && millis() - _packetStartMs >= DATA_FLUSH_MS) flushPacket();
}

// Sends the packet being built. Returns false if it couldn't be delivered;
// the packet stays for the next try. While the link is lost, it only tries
// every PROBE_PERIOD_MS so the radio isn't busy failing.
bool AlfredoTelemetry::flushPacket() {
    if (_packetLength <= 2) {
        _packetLength = 0;
        return true;
    }
    uint32_t now = millis();
    if (!_connected && (int32_t)(now - _nextProbeMs) < 0) return false;
    putU16(_packet, _schemaSent);
    bool firstTry = false;
    bool ok = radioSend(_dongleMac, PKT_DATA, _packet, _packetLength, _connected ? _retries : 0, &firstTry);
    noteDataResult(firstTry);
    if (!ok) {
        if (!_connected) _nextProbeMs = now + PROBE_PERIOD_MS;
        return false;
    }
    _packetLength = 0;
    return true;
}
