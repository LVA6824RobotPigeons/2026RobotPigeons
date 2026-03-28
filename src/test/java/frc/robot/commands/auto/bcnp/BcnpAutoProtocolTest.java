package frc.robot.commands.auto.bcnp;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.junit.jupiter.api.Test;

class BcnpAutoProtocolTest {
    @Test
    void handshake_roundTripsSchemaHash() {
        final int schemaHash = 0x1234ABCD;
        final byte[] handshake = BcnpAutoProtocol.buildHandshake(schemaHash);

        assertTrue(BcnpAutoProtocol.isHandshakeMagic(handshake));
        assertEquals(schemaHash, BcnpAutoProtocol.readHandshakeSchemaHash(handshake));
    }

    @Test
    void encodeDecode_roundTripsHeartbeatPacket() {
        final ByteBuffer payload = ByteBuffer.allocate(BcnpAutoProtocol.WIRE_AUTO_HEARTBEAT).order(ByteOrder.BIG_ENDIAN);
        payload.putInt(11);
        payload.putInt(22);
        payload.putInt(33);

        final byte[] packet = BcnpAutoProtocol.encodePacket(
            BcnpAutoProtocol.MSG_AUTO_HEARTBEAT,
            0,
            1,
            payload.array()
        );
        final BcnpAutoProtocol.DecodedPacket decoded = BcnpAutoProtocol.decodePacket(packet, 0, packet.length);

        assertTrue(decoded.isOk());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_HEARTBEAT, decoded.messageType());
        assertEquals(1, decoded.messageCount());
        assertArrayEquals(payload.array(), decoded.payload());
    }

    @Test
    void decodePacket_detectsCrcMismatch() {
        final byte[] payload = new byte[BcnpAutoProtocol.WIRE_AUTO_ABORT];
        final byte[] packet = BcnpAutoProtocol.encodePacket(
            BcnpAutoProtocol.MSG_AUTO_ABORT,
            0,
            1,
            payload
        );
        packet[packet.length - 1] ^= 0x55;

        final BcnpAutoProtocol.DecodedPacket decoded = BcnpAutoProtocol.decodePacket(packet, 0, packet.length);
        assertFalse(decoded.isOk());
        assertEquals(BcnpAutoProtocol.DecodeError.CRC_MISMATCH, decoded.error());
    }

    @Test
    void decodeAutoPlanResponsePayload_parsesFields() {
        final ByteBuffer payload = ByteBuffer.allocate(BcnpAutoProtocol.WIRE_AUTO_PLAN_RESPONSE).order(ByteOrder.BIG_ENDIAN);
        payload.putInt(1234);
        payload.putShort((short) 2);
        payload.putShort((short) 5);
        payload.putInt(0x00ABCDEF);
        payload.putShort((short) 103);
        payload.put((byte) 1);
        payload.put((byte) 0); // reserved0
        payload.putShort((short) 830);
        payload.putShort((short) 0); // reserved1

        final var decoded = BcnpAutoProtocol.decodeAutoPlanResponsePayload(payload.array());
        assertTrue(decoded.isPresent());
        assertEquals(1234L, decoded.get().planId());
        assertEquals(2, decoded.get().flags());
        assertEquals(5, decoded.get().phaseCount());
        assertEquals(0x00ABCDEFL, decoded.get().planChecksum());
        assertEquals(103, decoded.get().objectiveId());
        assertEquals(1, decoded.get().policySource());
        assertEquals(830, decoded.get().globalConfidencePermille());
    }

    @Test
    void decodeAutoHeartbeatPayload_rejectsWrongWireSize() {
        final var decoded = BcnpAutoProtocol.decodeAutoHeartbeatPayload(new byte[4]);
        assertTrue(decoded.isEmpty());
    }

    @Test
    void encodeAutoWorldUpdate_clampsSignedAndUnsignedShortFields() {
        final byte[] packet = BcnpAutoProtocol.encodeAutoWorldUpdate(
            999,
            true,
            100_000,
            80_000,
            999_999,
            -999_999,
            1_500,
            -2_500,
            999_999
        );
        final BcnpAutoProtocol.DecodedPacket decoded = BcnpAutoProtocol.decodePacket(packet, 0, packet.length);
        assertTrue(decoded.isOk());

        final ByteBuffer view = ByteBuffer.wrap(decoded.payload()).order(ByteOrder.BIG_ENDIAN);
        assertEquals(0xFF, Byte.toUnsignedInt(view.get()));
        assertEquals(1, Byte.toUnsignedInt(view.get()));
        assertEquals(0xFFFF, Short.toUnsignedInt(view.getShort()));
        assertEquals(0xFFFF, Short.toUnsignedInt(view.getShort()));
        assertEquals(Short.MAX_VALUE, view.getShort());
        assertEquals(Short.MIN_VALUE, view.getShort());
    }

    @Test
    void encodeAutoOpponentUpdate_clampsByteAndShortFields() {
        final byte[] packet = BcnpAutoProtocol.encodeAutoOpponentUpdate(
            -5,
            1_000,
            100,
            200,
            500_000,
            -500_000,
            2_000
        );
        final BcnpAutoProtocol.DecodedPacket decoded = BcnpAutoProtocol.decodePacket(packet, 0, packet.length);
        assertTrue(decoded.isOk());

        final ByteBuffer view = ByteBuffer.wrap(decoded.payload()).order(ByteOrder.BIG_ENDIAN);
        assertEquals(0, Byte.toUnsignedInt(view.get()));
        assertEquals(0xFF, Byte.toUnsignedInt(view.get()));
        assertEquals(100, view.getInt());
        assertEquals(200, view.getInt());
        assertEquals(Short.MAX_VALUE, view.getShort());
        assertEquals(Short.MIN_VALUE, view.getShort());
        assertEquals(2000, Short.toUnsignedInt(view.getShort()));
    }
}
