/*
 * GRS Ground Station
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 *
 * Wire protocol shared between the GCS and the
 * ESP32 catapult firmware. Kept dependency-free (no STL, no Arduino
 * headers) so this exact file can be dropped into both codebases.
 */

#ifndef CATAPULTPROTOCOL_H
#define CATAPULTPROTOCOL_H

#include <cstddef>
#include <stdint.h>

#define CATAPULT_MAGIC   0xC7
#define CATAPULT_PORT    5800   // default TCP port the ESP32 listens on

enum CatapultMsgType : uint8_t {
    MSG_HEARTBEAT   = 0x01, // catapult -> GCS, periodic, param = status bitmask
    MSG_ARM         = 0x02, // GCS -> catapult
    MSG_ARM_ACK     = 0x03, // catapult -> GCS, param = status bitmask
    MSG_DISARM      = 0x04, // GCS -> catapult
    MSG_DISARM_ACK  = 0x05, // catapult -> GCS, param = status bitmask
    MSG_FIRE_AT     = 0x06, // GCS -> catapult, param = countdown in ms
    MSG_FIRE_ACK    = 0x07, // catapult -> GCS, param = local millis() at actual release
    MSG_ABORT       = 0x08, // GCS -> catapult, cancels a pending countdown, disarms
    MSG_ABORT_ACK   = 0x09, // catapult -> GCS
    MSG_FAULT       = 0x0A, // catapult -> GCS, unsolicited, param = status bitmask
    MSG_PING        = 0x0B  // GCS -> catapult, idle keepalive (no ack expected). Any
                             // message from the GCS -- this one included -- resets the
                             // catapult's own "GCS went away" watchdog.
};

// Bits used in the `param` field for HEARTBEAT / *_ACK / FAULT messages.
enum CatapultStatusBit : uint32_t {
    STATUS_COCKED        = 1u << 0, // mechanical launch arm is pretensioned/latched
    STATUS_ARMED         = 1u << 1, // firmware has accepted MSG_ARM and will act on FIRE_AT
    STATUS_SAFETY_PIN_IN = 1u << 2, // physical safety pin/switch present -> refuses to arm
    STATUS_COUNTDOWN     = 1u << 3, // a FIRE_AT countdown is currently running
    STATUS_LOW_BATTERY   = 1u << 4,
    STATUS_GCS_TIMEOUT   = 1u << 5  // set right before firmware self-disarms on watchdog expiry
};

#pragma pack(push, 1)
struct CatapultPacket {
    uint8_t  magic;   // must equal CATAPULT_MAGIC
    uint8_t  type;    // CatapultMsgType
    uint16_t seq;     // set by sender, echoed back in the corresponding ACK
    uint32_t param;   // meaning depends on `type`, see above
    uint8_t  crc8;    // CRC-8 (poly 0x07) over magic..param, computed last
};
#pragma pack(pop)

static_assert(sizeof(CatapultPacket) == 9, "CatapultPacket must stay wire-compatible");

inline uint8_t catapultCrc8(const uint8_t* data, const int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

inline CatapultPacket catapultMakePacket(const CatapultMsgType type, const uint16_t seq, const uint32_t param) {
    CatapultPacket pkt{};
    pkt.magic = CATAPULT_MAGIC;
    pkt.type  = static_cast<uint8_t>(type);
    pkt.seq   = seq;
    pkt.param = param;
    pkt.crc8  = catapultCrc8(reinterpret_cast<const uint8_t*>(&pkt), offsetof(CatapultPacket, crc8));
    return pkt;
}

inline bool catapultValidatePacket(const CatapultPacket& pkt) {
    if (pkt.magic != CATAPULT_MAGIC) return false;
    return pkt.crc8 == catapultCrc8(reinterpret_cast<const uint8_t*>(&pkt), offsetof(CatapultPacket, crc8));
}

#endif //CATAPULTPROTOCOL_H
