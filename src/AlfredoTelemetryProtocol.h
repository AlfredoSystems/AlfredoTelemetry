#pragma once

// Wire format shared by the robot (AlfredoTelemetry), the USB dongle
// (AlfredoTelemetryDongle), and the host page (extras/TelemetryViewer).
// Everything is little-endian. If you change anything here, change the
// matching constants at the top of the viewer's <script> as well.

#include <stdint.h>
#include <esp_now.h>

namespace atlm {

const uint8_t PROTOCOL_VERSION = 1;
const uint8_t MAGIC = 0xA7;

const int MAX_NAME_LENGTH = 31;  // channel, tunable, and robot names (without the terminator)
const int MAX_CHANNELS = 64;
const int MAX_TUNABLES = 32;

#ifdef ESP_NOW_MAX_DATA_LEN_V2
const int MAX_PAYLOAD = ESP_NOW_MAX_DATA_LEN_V2;  // ESP-NOW v2 (ESP-IDF 5.4+): 1470 bytes
#else
const int MAX_PAYLOAD = ESP_NOW_MAX_DATA_LEN;     // ESP-NOW v1: 250 bytes
#endif
const int V1_PAYLOAD = 250;

// Every radio packet starts with this header.
struct __attribute__((packed)) PacketHeader {
    uint8_t magic;
    uint8_t type;
    uint16_t seq;  // per sender, counts every packet so the host can measure loss
};

// ---- Radio packets, robot -> dongle ----------------------------------------
// HELLO   (broadcast, 2 Hz while unpaired and 1 Hz while paired, so other
//         hosts can see a robot that's in use; seq counts HELLOs separately)
//         u8 protocol, u8 flags (HELLO_PAIRED), u16 max payload,
//         u8 name length, name, u8[6] mac of the paired dongle (zeros if none)
// STATUS  (to the dongle while paired, 2 Hz)
//         u16 schema version, u16 tunables version, u32 uptime ms,
//         u32 frames sent, u32 frames dropped, u32 tx failures,
//         u8 streaming, u8 buffer use %, u16 payload size,
//         u8 channel count, u8 tunable count, u32 retransmits,
//         u8 active radio rate (wifi_phy_rate_t), u8 flags (bit 0: rate stepped down),
//         u8 name length, name
// SCHEMA  u16 schema version, u8 channel count,
//         then per channel: u8 id, u8 type, u8 name length, name
// DATA    u16 schema version, then frames: u32 micros, u8 n, n x (u8 id, 4 byte value)
// TEXT    raw characters
// TUNABLES u16 tunables version, u8 tunable count,
//         then per tunable: u8 id, u8 type, 4 byte value, u8 name length, name
const uint8_t PKT_HELLO = 0x01;
const uint8_t PKT_STATUS = 0x02;
const uint8_t PKT_SCHEMA = 0x03;
const uint8_t PKT_DATA = 0x04;
const uint8_t PKT_TEXT = 0x05;
const uint8_t PKT_TUNABLES = 0x06;

// ---- Radio packets, dongle -> robot ----------------------------------------
// CONNECT    u8 protocol, u16 max payload the dongle can receive, u8 flags
//            (CONNECT_TAKEOVER). A paired robot ignores CONNECT from other
//            dongles unless it has CONNECT_TAKEOVER.
// HEARTBEAT  u8 flags (FLAG_STREAM)
// REQUEST    u8 flags (REQUEST_SCHEMA | REQUEST_TUNABLES)
// SET_TUNABLE u8 id, 4 byte value (float bits for float tunables)
// DISCONNECT (no payload)
const uint8_t PKT_CONNECT = 0x10;
const uint8_t PKT_HEARTBEAT = 0x11;
const uint8_t PKT_REQUEST = 0x12;
const uint8_t PKT_SET_TUNABLE = 0x13;
const uint8_t PKT_DISCONNECT = 0x14;

const uint8_t FLAG_STREAM = 0x01;
const uint8_t HELLO_PAIRED = 0x01;
const uint8_t CONNECT_TAKEOVER = 0x01;
const uint8_t REQUEST_SCHEMA = 0x01;
const uint8_t REQUEST_TUNABLES = 0x02;

// Value types, used by channels and tunables
const uint8_t TYPE_FLOAT = 0;
const uint8_t TYPE_INT = 1;
const uint8_t TYPE_UINT = 2;
const uint8_t TYPE_BOOL = 3;

// ---- Serial frames, dongle <-> host ----------------------------------------
// Each frame is [type][payload][crc8], COBS encoded, and ends with a 0x00 byte.
// crc8 is polynomial 0x07, initial value 0, over type and payload.

// dongle -> host
// SEEN    u8[6] robot mac, i8 rssi, the robot's HELLO packet (header included)
// PACKET  i8 rssi, a packet from the paired robot (header included)
// STATUS  u8 protocol, u8 wifi channel, u8 state (DONGLE_*), u8[6] target mac,
//         u32 packets received, u32 packets dropped (dongle buffer full),
//         u32 tx failures, u16 max payload, u8[6] dongle mac, u8 streaming requested,
//         u32 ESP-NOW packets heard from anyone,
//         u8 rate, u8 sig_mode, u8 mcs of the last DATA packet (rx_ctrl fields, 0xFF = none yet),
//         u8 dongle Wi-Fi protocol bitmap (WIFI_PROTOCOL_*; 0x8 = LR enabled)
// LOG     raw characters from the dongle itself
const uint8_t SER_SEEN = 0x81;
const uint8_t SER_PACKET = 0x82;
const uint8_t SER_STATUS = 0x83;
const uint8_t SER_LOG = 0x84;

// host -> dongle
// PONG     (no payload) reply to every dongle STATUS, so the dongle knows a host is listening
// CONNECT  u8[6] robot mac (all zeros to disconnect), u8 flags (CONNECT_TAKEOVER)
// STREAM   u8 on
// FORWARD  u8 radio packet type, payload: sent to the paired robot as-is
// CHANNEL  u8 wifi channel
// SCAN     (no payload) scan for Wi-Fi networks and report them as LOG lines,
//          to check that the dongle's radio and antenna receive anything at all
const uint8_t SER_PONG = 0x01;
const uint8_t SER_CONNECT = 0x02;
const uint8_t SER_STREAM = 0x03;
const uint8_t SER_FORWARD = 0x04;
const uint8_t SER_CHANNEL = 0x05;
const uint8_t SER_SCAN = 0x06;

const uint8_t DONGLE_IDLE = 0;
const uint8_t DONGLE_CONNECTING = 1;
const uint8_t DONGLE_LINKED = 2;
const uint8_t DONGLE_BUSY = 3;  // the robot is paired with another dongle; waiting for it to be free

// Timing, in milliseconds
const uint32_t HELLO_PERIOD_MS = 500;
const uint32_t PAIRED_HELLO_PERIOD_MS = 1000;
const uint32_t STATUS_PERIOD_MS = 500;
const uint32_t TUNABLES_PERIOD_MS = 1000;
const uint32_t HEARTBEAT_PERIOD_MS = 200;
const uint32_t LINK_TIMEOUT_MS = 1500;
const uint32_t HOST_TIMEOUT_MS = 2500;

}  // namespace atlm
