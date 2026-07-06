/*
 * link_protocol.h - Jetson<->MCU framed link, C side.
 *
 * Byte-for-byte identical to rover_hw_bridge/rover_hw_bridge/link_protocol.py.
 * Frame: [SOF=0xAA][MSG_ID][LEN][PAYLOAD...][CRC8(poly 0x07 over ID+LEN+PAYLOAD)]
 * All multi-byte payload fields are little-endian.
 */
#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define LINK_SOF          0xAAu
#define MSG_VEL_CMD       0x01u   /* Jetson->MCU: float32 vx, float32 wz          */
#define MSG_FEEDBACK      0x02u   /* MCU->Jetson: f32 vL, f32 vR, f32 cur, u8 flt */
#define MSG_HEARTBEAT     0x03u   /* MCU->Jetson: u32 uptime_ms                   */

#define FAULT_NONE        0x00u
#define FAULT_OVERCURRENT 0x01u
#define FAULT_STALL       0x02u
#define FAULT_WATCHDOG    0x04u

static inline uint8_t link_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* Build a frame into out[] (must hold 4 + payload_len bytes). Returns length. */
static inline size_t link_frame(uint8_t msg_id, const uint8_t *payload,
                                uint8_t payload_len, uint8_t *out)
{
    out[0] = LINK_SOF;
    out[1] = msg_id;
    out[2] = payload_len;
    for (uint8_t i = 0; i < payload_len; ++i)
        out[3 + i] = payload[i];
    out[3 + payload_len] = link_crc8(&out[1], (size_t)(2 + payload_len));
    return (size_t)(4 + payload_len);
}

typedef struct { float vx; float wz; } vel_cmd_t;

/* Incremental parser state (feed bytes from your UART/CAN ISR). */
typedef struct {
    uint8_t buf[64];
    uint8_t len;
} link_parser_t;

/* Returns 1 and fills *cmd when a valid VEL_CMD is decoded, else 0. */
int link_parse_byte(link_parser_t *p, uint8_t byte, vel_cmd_t *cmd);

#endif /* LINK_PROTOCOL_H */
