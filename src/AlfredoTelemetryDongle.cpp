#include "AlfredoTelemetryDongle.h"

#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

using namespace atlm;

AlfredoTelemetryDongle TelemetryDongle;

namespace {

const size_t HEADER_SIZE = sizeof(PacketHeader);
const size_t RX_BUFFER_SIZE = 32768;
const uint32_t CONNECT_PERIOD_MS = 250;
const uint32_t HOST_STATUS_PERIOD_MS = 250;

// Shared between the Wi-Fi task (onReceive/onSent) and update()
RingbufHandle_t rxBuffer = nullptr;
volatile uint8_t targetMac[6] = {0};
volatile bool hasTarget = false;
volatile bool robotUnpaired = false;  // the target robot's HELLO says it's free
volatile bool robotBusy = false;      // the target robot's HELLO says it's paired with another dongle
uint8_t ownMac[6] = {0};
volatile uint32_t lastRobotMs = 0;
volatile uint32_t rxPackets = 0;
volatile uint32_t radioHeard = 0;  // every ESP-NOW packet, from anyone: shows whether the radio hears anything
volatile uint32_t rxDropped = 0;
volatile uint32_t txFailures = 0;

uint8_t crcTable[256];

void buildCrcTable() {
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
        crcTable[i] = crc;
    }
}

uint8_t crc8(uint8_t crc, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) crc = crcTable[crc ^ data[i]];
    return crc;
}

bool isTarget(const uint8_t *mac) {
    if (!hasTarget) return false;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != targetMac[i]) return false;
    }
    return true;
}

// Copies a packet into the buffer as a serial frame body: [type][prefix][packet]
void queueForHost(uint8_t type, const uint8_t *prefix, size_t prefixLength, const uint8_t *packet, size_t packetLength) {
    void *item = nullptr;
    size_t size = 1 + prefixLength + packetLength;
    if (xRingbufferSendAcquire(rxBuffer, &item, size, 0) != pdTRUE) {
        rxDropped = rxDropped + 1;
        return;
    }
    uint8_t *p = (uint8_t *)item;
    p[0] = type;
    memcpy(p + 1, prefix, prefixLength);
    memcpy(p + 1 + prefixLength, packet, packetLength);
    xRingbufferSendComplete(rxBuffer, item);
}

// Runs in the Wi-Fi task, so it only copies the packet out.
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
    radioHeard = radioHeard + 1;
    if (rxBuffer == nullptr || length < (int)HEADER_SIZE || data[0] != MAGIC) return;
    int8_t rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
    uint8_t type = data[1];
    bool fromTarget = isTarget(info->src_addr);

    if (type == PKT_HELLO) {
        uint8_t prefix[7];
        memcpy(prefix, info->src_addr, 6);
        prefix[6] = (uint8_t)rssi;
        queueForHost(SER_SEEN, prefix, 7, data, length);
        if (fromTarget) {
            // A paired robot's HELLO ends with its dongle's mac.
            const uint8_t *p = data + HEADER_SIZE;
            int payloadLength = length - HEADER_SIZE;
            bool paired = payloadLength >= 2 && (p[1] & HELLO_PAIRED);
            if (!paired) robotUnpaired = true;
            else if (payloadLength >= 5 + p[4] + 6 && memcmp(p + 5 + p[4], ownMac, 6) != 0) robotBusy = true;
        }
    } else if (fromTarget && type < PKT_CONNECT) {
        lastRobotMs = millis();
        rxPackets = rxPackets + 1;
        uint8_t prefix[1] = {(uint8_t)rssi};
        queueForHost(SER_PACKET, prefix, 1, data, length);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *, esp_now_send_status_t status) {
#endif
    if (status != ESP_NOW_SEND_SUCCESS) txFailures = txFailures + 1;
}

wifi_phy_mode_t phyModeFor(wifi_phy_rate_t rate) {
    if (rate <= WIFI_PHY_RATE_11M_S) return WIFI_PHY_MODE_11B;
    if (rate <= WIFI_PHY_RATE_9M) return WIFI_PHY_MODE_11G;
    if (rate <= WIFI_PHY_RATE_MCS7_SGI) return WIFI_PHY_MODE_HT20;
    return WIFI_PHY_MODE_LR;
}

// COBS, written straight into the output buffer
struct CobsWriter {
    uint8_t *out;
    size_t codeIndex = 0;
    size_t pos = 1;
    uint8_t code = 1;

    explicit CobsWriter(uint8_t *buffer) : out(buffer) {}

    void put(uint8_t b) {
        if (b != 0) {
            out[pos++] = b;
            code++;
        }
        if (b == 0 || code == 0xFF) {
            out[codeIndex] = code;
            codeIndex = pos++;
            code = 1;
        }
    }
    void put(const uint8_t *data, size_t length) {
        for (size_t i = 0; i < length; i++) put(data[i]);
    }
    size_t finish() {
        out[codeIndex] = code;
        out[pos++] = 0;
        return pos;
    }
};

// Decodes a COBS frame in place (without its 0x00 delimiter). Returns the decoded length, or 0 if malformed.
size_t cobsDecode(uint8_t *buffer, size_t length) {
    size_t r = 0, w = 0;
    while (r < length) {
        uint8_t code = buffer[r++];
        if (code == 0) return 0;
        for (uint8_t i = 1; i < code; i++) {
            if (r >= length) return 0;
            buffer[w++] = buffer[r++];
        }
        if (code != 0xFF && r < length) buffer[w++] = 0;
    }
    return w;
}

void putU16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
void putU32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

}  // namespace

bool AlfredoTelemetryDongle::begin(uint8_t wifiChannel, unsigned long baud) {
    if (_started) return true;
    _wifiChannel = wifiChannel;
    buildCrcTable();

    // Bigger USB buffers let update() hand off whole packets at a time. With
    // native USB the baud rate is ignored.
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setRxBufferSize(4096);
    Serial.setTxTimeoutMs(20);  // don't stall for long when nothing is reading
#endif
#if !ARDUINO_USB_CDC_ON_BOOT || ARDUINO_USB_MODE
    Serial.setTxBufferSize(8192);  // HWCDC and UART
#endif
#if !ARDUINO_USB_CDC_ON_BOOT
    Serial.setRxBufferSize(4096);
#endif
    Serial.begin(baud);

    rxBuffer = xRingbufferCreate(RX_BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (rxBuffer == nullptr) return false;

    if (!WiFi.mode(WIFI_STA)) {
        log("Wi-Fi failed to start");
        return false;
    }
    WiFi.setSleep(false);  // the dongle never runs BLE, so keep the radio awake for the lowest latency
    esp_wifi_set_channel(_wifiChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(_txPower);  // see setTxPower(); only works once Wi-Fi has started
    esp_wifi_get_mac(WIFI_IF_STA, _ownMac);
    memcpy(ownMac, _ownMac, 6);

    if (esp_now_init() != ESP_OK) {
        log("esp_now_init failed");
        return false;
    }
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSent);

    _started = true;
    return true;
}

void AlfredoTelemetryDongle::update() {
    if (!_started) return;
    readHost();

    uint32_t now = millis();
    bool hostAlive = _lastPongMs != 0 && now - _lastPongMs < HOST_TIMEOUT_MS;

    if (_state != DONGLE_IDLE) {
        if (robotUnpaired) {
            // The robot is free (it reset, timed out, or was released): pair right away.
            robotUnpaired = false;
            _state = DONGLE_CONNECTING;
            lastRobotMs = 0;
            _lastConnectMs = now - CONNECT_PERIOD_MS;
        }
        if (robotBusy) {
            // Another dongle has the robot. Wait for a HELLO saying it's free,
            // unless the page asked for a takeover.
            robotBusy = false;
            if (!_takeover) {
                _state = DONGLE_BUSY;
                lastRobotMs = 0;
            }
        }
        // The robot only sends STATUS and data to its paired dongle, so hearing
        // them means it's ours, even if an older HELLO said otherwise.
        bool robotAlive = lastRobotMs != 0 && now - lastRobotMs < LINK_TIMEOUT_MS;
        if (_state != DONGLE_LINKED && robotAlive) {
            _state = DONGLE_LINKED;
            _takeover = false;
        }
        if (_state == DONGLE_LINKED && !robotAlive) _state = DONGLE_CONNECTING;

        if (hostAlive) {
            if (_state == DONGLE_CONNECTING && now - _lastConnectMs >= CONNECT_PERIOD_MS) {
                _lastConnectMs = now;
                uint8_t payload[4] = {PROTOCOL_VERSION};
                putU16(payload + 1, MAX_PAYLOAD);
                payload[3] = _takeover ? CONNECT_TAKEOVER : 0;
                radioSend(PKT_CONNECT, payload, 4);
            }
            if (_state == DONGLE_LINKED && now - _lastHeartbeatMs >= HEARTBEAT_PERIOD_MS) {
                _lastHeartbeatMs = now;
                uint8_t flags = _stream ? FLAG_STREAM : 0;
                radioSend(PKT_HEARTBEAT, &flags, 1);
            }
        } else if (_hostWasAlive) {
            // The page went away: release the robot so it stops streaming,
            // and so another host can have it.
            if (_state == DONGLE_LINKED) radioSend(PKT_DISCONNECT, nullptr, 0);
            _state = DONGLE_CONNECTING;
            _takeover = false;
            lastRobotMs = 0;
        }
    }
    _hostWasAlive = hostAlive;

    if (_scanRequested) {
        _scanRequested = false;
        scan();
    }

    // Forward everything the radio received.
    size_t size;
    uint8_t *item;
    int forwarded = 0;
    while (forwarded < 64 && (item = (uint8_t *)xRingbufferReceive(rxBuffer, &size, 0))) {
        writeFrame(item[0], item + 1, size - 1);
        vRingbufferReturnItem(rxBuffer, item);
        forwarded++;
    }

    if (now - _lastStatusMs >= HOST_STATUS_PERIOD_MS) {
        _lastStatusMs = now;
        sendStatus();
    }
    flushOut();
}

void AlfredoTelemetryDongle::sendStatus() {
    uint8_t p[34];
    p[0] = PROTOCOL_VERSION;
    uint8_t channel = _wifiChannel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&channel, &second);  // what the radio is really on
    p[1] = channel;
    p[2] = _state;
    for (int i = 0; i < 6; i++) p[3 + i] = targetMac[i];
    putU32(p + 9, rxPackets);
    putU32(p + 13, rxDropped);
    putU32(p + 17, txFailures);
    putU16(p + 21, MAX_PAYLOAD);
    memcpy(p + 23, _ownMac, 6);
    p[29] = _stream;
    putU32(p + 30, radioHeard);
    writeFrame(SER_STATUS, p, sizeof(p));
}

// Lists nearby Wi-Fi networks, strongest first. Access points beacon all the
// time, so an empty or very weak list means the radio or antenna isn't
// receiving. Takes a few seconds, during which the robot link pauses.
void AlfredoTelemetryDongle::scan() {
    log("Scanning Wi-Fi (about 3 s)...");
    int count = WiFi.scanNetworks();
    esp_wifi_set_channel(_wifiChannel, WIFI_SECOND_CHAN_NONE);  // the scan leaves the radio on another channel
    _lastPongMs = millis();  // the page couldn't answer while this blocked

    char line[96];
    if (count <= 0) {
        log(count == 0 ? "Scan: no networks heard. If there is Wi-Fi nearby, the dongle's antenna isn't receiving."
                       : "Scan failed");
        WiFi.scanDelete();
        return;
    }
    int perChannel[15] = {0};
    for (int i = 0; i < count; i++) {
        int ch = WiFi.channel(i);
        if (ch >= 1 && ch <= 14) perChannel[ch]++;
    }
    snprintf(line, sizeof(line), "Scan: %d networks. Strongest:", count);
    log(line);
    // Strongest first; print at most 8
    bool shown[64] = {false};
    int limit = count < 64 ? count : 64;
    for (int n = 0; n < 8 && n < limit; n++) {
        int best = -1;
        for (int i = 0; i < limit; i++) {
            if (!shown[i] && (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best))) best = i;
        }
        shown[best] = true;
        String ssid = WiFi.SSID(best);
        snprintf(line, sizeof(line), "  %4ld dBm  ch %2ld  %s", (long)WiFi.RSSI(best), (long)WiFi.channel(best),
                 ssid.length() ? ssid.c_str() : "(hidden)");
        log(line);
    }
    int p = snprintf(line, sizeof(line), "Networks per channel:");
    for (int ch = 1; ch <= 13 && p < (int)sizeof(line) - 8; ch++) {
        if (perChannel[ch]) p += snprintf(line + p, sizeof(line) - p, " %d:%d", ch, perChannel[ch]);
    }
    log(line);
    WiFi.scanDelete();
}

void AlfredoTelemetryDongle::log(const char *text) {
    writeFrame(SER_LOG, (const uint8_t *)text, strlen(text));
    flushOut();
}

// ---------------------------------------------------------------- host link

void AlfredoTelemetryDongle::writeFrame(uint8_t type, const uint8_t *a, size_t aLength, const uint8_t *b, size_t bLength) {
    size_t length = 2 + aLength + bLength;        // type, data, crc
    size_t worstCase = length + length / 254 + 2;  // COBS overhead and the delimiter
    if (worstCase > sizeof(_out)) return;
    if (_outLength + worstCase > sizeof(_out)) flushOut();

    uint8_t crc = crc8(0, &type, 1);
    crc = crc8(crc, a, aLength);
    if (b) crc = crc8(crc, b, bLength);

    CobsWriter cobs(_out + _outLength);
    cobs.put(type);
    cobs.put(a, aLength);
    if (b) cobs.put(b, bLength);
    cobs.put(crc);
    _outLength += cobs.finish();
}

void AlfredoTelemetryDongle::flushOut() {
    if (_outLength == 0) return;
    Serial.write(_out, _outLength);
    _outLength = 0;
}

void AlfredoTelemetryDongle::readHost() {
    int available = Serial.available();
    while (available-- > 0) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == 0) {
            if (!_inOverflow && _inLength > 0) {
                size_t length = cobsDecode(_in, _inLength);
                if (length >= 2 && crc8(0, _in, length - 1) == _in[length - 1]) handleHostFrame(_in, length - 1);
            }
            _inLength = 0;
            _inOverflow = false;
        } else if (_inLength < sizeof(_in)) {
            _in[_inLength++] = c;
        } else {
            _inOverflow = true;
        }
    }
}

void AlfredoTelemetryDongle::handleHostFrame(uint8_t *frame, size_t length) {
    _lastPongMs = millis();  // any valid frame means the page is there
    uint8_t type = frame[0];
    uint8_t *payload = frame + 1;
    size_t payloadLength = length - 1;

    if (type == SER_CONNECT && payloadLength >= 6) {
        setTarget(payload, payloadLength >= 7 && (payload[6] & CONNECT_TAKEOVER));
    } else if (type == SER_STREAM && payloadLength >= 1) {
        _stream = payload[0] != 0;
        _lastHeartbeatMs = 0;  // tell the robot now
    } else if (type == SER_FORWARD && payloadLength >= 1) {
        if (_state != DONGLE_IDLE) radioSend(payload[0], payload + 1, payloadLength - 1);
    } else if (type == SER_SCAN) {
        _scanRequested = true;  // done from update(), after this frame is handled
    } else if (type == SER_CHANNEL && payloadLength >= 1) {
        if (payload[0] >= 1 && payload[0] <= 14 && esp_wifi_set_channel(payload[0], WIFI_SECOND_CHAN_NONE) == ESP_OK) {
            _wifiChannel = payload[0];
        }
    }
}

// ---------------------------------------------------------------- radio

void AlfredoTelemetryDongle::setTarget(const uint8_t *mac, bool takeover) {
    static const uint8_t none[6] = {0};
    bool disconnect = memcmp(mac, none, 6) == 0;
    uint8_t oldMac[6];
    for (int i = 0; i < 6; i++) oldMac[i] = targetMac[i];

    if (hasTarget) {
        if (!disconnect && memcmp(mac, oldMac, 6) == 0) {
            // Already our robot. A takeover request starts asking again.
            if (takeover && _state != DONGLE_LINKED) {
                _takeover = true;
                _state = DONGLE_CONNECTING;
                _lastConnectMs = millis() - CONNECT_PERIOD_MS;
            }
            return;
        }
        if (_state == DONGLE_LINKED) radioSend(PKT_DISCONNECT, nullptr, 0);
        hasTarget = false;
        esp_now_del_peer(oldMac);
    }
    for (int i = 0; i < 6; i++) targetMac[i] = mac[i];
    lastRobotMs = 0;
    robotUnpaired = false;
    robotBusy = false;
    _takeover = takeover;
    if (disconnect) {
        _state = DONGLE_IDLE;
        return;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        log("could not add the robot as an ESP-NOW peer");
        _state = DONGLE_IDLE;
        return;
    }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    esp_now_rate_config_t rate = {};
    rate.phymode = phyModeFor(_radioRate);
    rate.rate = _radioRate;
    esp_now_set_peer_rate_config(mac, &rate);
#else
    esp_wifi_config_espnow_rate(WIFI_IF_STA, _radioRate);
#endif
    hasTarget = true;
    _state = DONGLE_CONNECTING;
    _lastConnectMs = millis() - CONNECT_PERIOD_MS;
}

void AlfredoTelemetryDongle::radioSend(uint8_t type, const uint8_t *payload, size_t length) {
    if (!hasTarget || length > 64) return;
    uint8_t packet[HEADER_SIZE + 64];
    PacketHeader header = {MAGIC, type, _seq++};
    memcpy(packet, &header, HEADER_SIZE);
    if (length) memcpy(packet + HEADER_SIZE, payload, length);
    uint8_t mac[6];
    for (int i = 0; i < 6; i++) mac[i] = targetMac[i];
    if (esp_now_send(mac, packet, HEADER_SIZE + length) != ESP_OK) txFailures = txFailures + 1;
}
