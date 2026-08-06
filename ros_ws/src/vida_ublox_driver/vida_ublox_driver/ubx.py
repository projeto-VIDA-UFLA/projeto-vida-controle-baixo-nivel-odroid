"""Codificação e decodificação mínima do protocolo binário u-blox UBX."""

from dataclasses import dataclass
import struct
from typing import Iterable, List


SYNC = b'\xb5\x62'


@dataclass(frozen=True)
class UbxFrame:
    msg_class: int
    msg_id: int
    payload: bytes


def checksum(data: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def encode(msg_class: int, msg_id: int, payload: bytes = b'') -> bytes:
    body = bytes((msg_class, msg_id)) + struct.pack('<H', len(payload)) + payload
    return SYNC + body + checksum(body)


def cfg_msg(msg_class: int, msg_id: int, rate: int = 1) -> bytes:
    """Configura a taxa da mensagem na porta corrente."""
    return encode(0x06, 0x01, bytes((msg_class, msg_id, rate)))


class UbxStreamParser:
    """Parser incremental, tolerante a NMEA e bytes espúrios no mesmo fluxo."""

    def __init__(self, max_payload: int = 4096) -> None:
        self._buffer = bytearray()
        self._max_payload = max_payload

    def feed(self, data: bytes) -> List[UbxFrame]:
        self._buffer.extend(data)
        frames: List[UbxFrame] = []

        while True:
            start = self._buffer.find(SYNC)
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == SYNC[:1] else b''
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 8:
                break

            payload_length = struct.unpack_from('<H', self._buffer, 4)[0]
            if payload_length > self._max_payload:
                del self._buffer[0]
                continue

            frame_length = 6 + payload_length + 2
            if len(self._buffer) < frame_length:
                break

            candidate = bytes(self._buffer[:frame_length])
            body = candidate[2:-2]
            if checksum(body) != candidate[-2:]:
                del self._buffer[0]
                continue

            frames.append(UbxFrame(candidate[2], candidate[3], candidate[6:-2]))
            del self._buffer[:frame_length]

        return frames


def signed_24(value: int) -> int:
    value &= 0xFFFFFF
    return value - (1 << 24) if value & 0x800000 else value


def iter_esf_raw(payload: bytes) -> Iterable[tuple[int, int, int]]:
    """Retorna (tipo, valor assinado de 24 bits, sensor_time_tag)."""
    if len(payload) < 4 or (len(payload) - 4) % 8:
        return
    for offset in range(4, len(payload), 8):
        word, time_tag = struct.unpack_from('<II', payload, offset)
        yield (word >> 24, signed_24(word), time_tag)

