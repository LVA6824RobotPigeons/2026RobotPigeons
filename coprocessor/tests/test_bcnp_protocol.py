from __future__ import annotations

import struct
import unittest

from coprocessor.protocol import bcnp


class BcnpProtocolTest(unittest.TestCase):
    def test_handshake_round_trip(self) -> None:
        schema_hash = 0x1234ABCD
        handshake = bcnp.build_handshake(schema_hash)
        self.assertTrue(bcnp.is_handshake_magic(handshake))
        self.assertEqual(schema_hash, bcnp.read_handshake_schema_hash(handshake))

    def test_encode_decode_round_trip(self) -> None:
        payload = struct.pack(">III", 11, 22, 33)
        packet = bcnp.encode_packet(bcnp.MSG_AUTO_HEARTBEAT, 0, 1, payload)
        decoded = bcnp.decode_packet(packet)
        self.assertTrue(decoded.is_ok)
        self.assertEqual(bcnp.MSG_AUTO_HEARTBEAT, decoded.message_type)
        self.assertEqual(1, decoded.message_count)
        self.assertEqual(payload, decoded.payload)

    def test_decode_crc_mismatch(self) -> None:
        payload = bytes(bcnp.WIRE_SIZES[bcnp.MSG_AUTO_ABORT])
        packet = bytearray(bcnp.encode_packet(bcnp.MSG_AUTO_ABORT, 0, 1, payload))
        packet[-1] ^= 0x55
        decoded = bcnp.decode_packet(packet)
        self.assertFalse(decoded.is_ok)
        self.assertEqual(bcnp.DecodeError.CRC_MISMATCH, decoded.error)


if __name__ == "__main__":
    unittest.main()

