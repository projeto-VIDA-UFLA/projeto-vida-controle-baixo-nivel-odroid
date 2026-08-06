import struct

from vida_ublox_driver.ubx import (
    UbxStreamParser,
    encode,
    iter_esf_raw,
    signed_24,
)


def test_incremental_parser_ignores_nmea():
    expected_payload = bytes(range(20))
    encoded = encode(0x01, 0x07, expected_payload)
    parser = UbxStreamParser()
    assert parser.feed(b'$GNGGA,ignored*00\r\n' + encoded[:5]) == []
    frames = parser.feed(encoded[5:])
    assert len(frames) == 1
    assert frames[0].msg_class == 0x01
    assert frames[0].msg_id == 0x07
    assert frames[0].payload == expected_payload


def test_parser_rejects_bad_checksum_and_recovers():
    bad = bytearray(encode(0x01, 0x02, b'bad'))
    bad[-1] ^= 0xFF
    good = encode(0x10, 0x03, b'good')
    frames = UbxStreamParser().feed(bytes(bad) + good)
    assert len(frames) == 1
    assert frames[0].payload == b'good'


def test_signed_24():
    assert signed_24(0x000001) == 1
    assert signed_24(0x7FFFFF) == 8388607
    assert signed_24(0xFFFFFF) == -1
    assert signed_24(0x800000) == -8388608


def test_iter_esf_raw():
    word = (16 << 24) | 0xFFFFFE  # accel X, valor -2
    payload = bytes(4) + struct.pack('<II', word, 1234)
    assert list(iter_esf_raw(payload)) == [(16, -2, 1234)]

