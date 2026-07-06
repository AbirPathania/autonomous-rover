"""Framed Jetson<->MCU link protocol (serial/CAN-agnostic).

This is deliberately hardware-ready: the exact same framing runs over a UInt8
topic in SITL, but the ``frame()`` / ``StreamParser`` pair works byte-for-byte
over a real pyserial or python-can link with no change. Mirror this format in the
STM32 firmware (see firmware/stm32/link_protocol.h).

Frame:
    [SOF=0xAA][MSG_ID:u8][LEN:u8][PAYLOAD:LEN bytes][CRC8]
    CRC8 (poly 0x07) is computed over MSG_ID + LEN + PAYLOAD.

Messages (little-endian payloads):
    VEL_CMD  (0x01): float32 vx, float32 wz            -> Jetson to MCU
    FEEDBACK (0x02): float32 v_left, float32 v_right,
                     float32 bus_current, u8 fault      -> MCU to Jetson
    HEARTBEAT(0x03): u32 uptime_ms                      -> MCU to Jetson
"""
import struct

SOF = 0xAA
MSG_VEL_CMD = 0x01
MSG_FEEDBACK = 0x02
MSG_HEARTBEAT = 0x03

# Fault bitmask (FEEDBACK.fault)
FAULT_NONE = 0x00
FAULT_OVERCURRENT = 0x01
FAULT_STALL = 0x02
FAULT_WATCHDOG = 0x04


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def frame(msg_id: int, payload: bytes) -> bytes:
    body = bytes([msg_id, len(payload)]) + payload
    return bytes([SOF]) + body + bytes([crc8(body)])


def pack_vel_cmd(vx: float, wz: float) -> bytes:
    return frame(MSG_VEL_CMD, struct.pack('<ff', vx, wz))


def pack_feedback(v_left: float, v_right: float, current: float, fault: int) -> bytes:
    return frame(MSG_FEEDBACK, struct.pack('<fffB', v_left, v_right, current, fault & 0xFF))


def pack_heartbeat(uptime_ms: int) -> bytes:
    return frame(MSG_HEARTBEAT, struct.pack('<I', uptime_ms & 0xFFFFFFFF))


def parse_payload(msg_id: int, payload: bytes):
    """Decode a validated payload into a dict, or None on length mismatch."""
    try:
        if msg_id == MSG_VEL_CMD:
            vx, wz = struct.unpack('<ff', payload)
            return {'type': 'vel_cmd', 'vx': vx, 'wz': wz}
        if msg_id == MSG_FEEDBACK:
            vl, vr, cur, fault = struct.unpack('<fffB', payload)
            return {'type': 'feedback', 'v_left': vl, 'v_right': vr,
                    'current': cur, 'fault': fault}
        if msg_id == MSG_HEARTBEAT:
            (uptime,) = struct.unpack('<I', payload)
            return {'type': 'heartbeat', 'uptime_ms': uptime}
    except struct.error:
        return None
    return None


class StreamParser:
    """Incremental byte-stream parser; feed bytes, get decoded messages."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        """Append bytes and yield every complete, CRC-valid message decoded."""
        self._buf.extend(data)
        out = []
        while True:
            # Find start-of-frame.
            start = self._buf.find(SOF)
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 3:
                break  # need SOF, id, len
            msg_id = self._buf[1]
            length = self._buf[2]
            total = 3 + length + 1  # SOF + id + len + payload + crc
            if len(self._buf) < total:
                break  # wait for more bytes
            body = bytes(self._buf[1:3 + length])
            crc = self._buf[3 + length]
            if crc8(body) == crc:
                decoded = parse_payload(msg_id, bytes(self._buf[3:3 + length]))
                if decoded is not None:
                    out.append(decoded)
                del self._buf[:total]
            else:
                # Bad CRC: drop this SOF and resync.
                del self._buf[0]
        return out
