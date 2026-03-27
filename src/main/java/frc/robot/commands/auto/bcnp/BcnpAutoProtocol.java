package frc.robot.commands.auto.bcnp;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Optional;
import java.util.zip.CRC32;

public final class BcnpAutoProtocol {
    private BcnpAutoProtocol() {
    }

    public static final int PROTOCOL_MAJOR = 3;
    public static final int PROTOCOL_MINOR = 4;

    public static final int HEADER_SIZE = 7;
    public static final int CRC_SIZE = 4;
    public static final int HANDSHAKE_SIZE = 8;

    public static final int FLAG_CLEAR_QUEUE = 0x01;

    // Message IDs reserved for robot autonomous planner control.
    public static final int MSG_AUTO_PLAN_REQUEST = 2001;
    public static final int MSG_AUTO_PLAN_RESPONSE = 2002;
    public static final int MSG_AUTO_PHASE_COMMAND = 2003;
    public static final int MSG_AUTO_PHASE_ACK = 2004;
    public static final int MSG_AUTO_HEARTBEAT = 2005;
    public static final int MSG_AUTO_ABORT = 2006;
    public static final int MSG_AUTO_TELEMETRY = 2007;
    public static final int MSG_AUTO_SHOT_HINT = 2008;
    public static final int MSG_AUTO_WORLD_UPDATE = 2009;
    public static final int MSG_AUTO_WAYPOINT_DELTA = 2010;
    public static final int MSG_AUTO_OPPONENT_UPDATE = 2011;

    // Fixed wire sizes for autonomous message payloads, kept in sync with
    // src/main/deploy/bcnp/messages.json.
    public static final int WIRE_AUTO_PLAN_REQUEST = 16;
    public static final int WIRE_AUTO_PLAN_RESPONSE = 20;
    public static final int WIRE_AUTO_PHASE_COMMAND = 32;
    public static final int WIRE_AUTO_PHASE_ACK = 12;
    public static final int WIRE_AUTO_HEARTBEAT = 12;
    public static final int WIRE_AUTO_ABORT = 8;
    public static final int WIRE_AUTO_TELEMETRY = 32;
    public static final int WIRE_AUTO_SHOT_HINT = 16;
    public static final int WIRE_AUTO_WORLD_UPDATE = 20;
    public static final int WIRE_AUTO_WAYPOINT_DELTA = 20;
    public static final int WIRE_AUTO_OPPONENT_UPDATE = 18;

    public static int wireSize(int messageType) {
        return switch (messageType) {
            case MSG_AUTO_PLAN_REQUEST -> WIRE_AUTO_PLAN_REQUEST;
            case MSG_AUTO_PLAN_RESPONSE -> WIRE_AUTO_PLAN_RESPONSE;
            case MSG_AUTO_PHASE_COMMAND -> WIRE_AUTO_PHASE_COMMAND;
            case MSG_AUTO_PHASE_ACK -> WIRE_AUTO_PHASE_ACK;
            case MSG_AUTO_HEARTBEAT -> WIRE_AUTO_HEARTBEAT;
            case MSG_AUTO_ABORT -> WIRE_AUTO_ABORT;
            case MSG_AUTO_TELEMETRY -> WIRE_AUTO_TELEMETRY;
            case MSG_AUTO_SHOT_HINT -> WIRE_AUTO_SHOT_HINT;
            case MSG_AUTO_WORLD_UPDATE -> WIRE_AUTO_WORLD_UPDATE;
            case MSG_AUTO_WAYPOINT_DELTA -> WIRE_AUTO_WAYPOINT_DELTA;
            case MSG_AUTO_OPPONENT_UPDATE -> WIRE_AUTO_OPPONENT_UPDATE;
            default -> 0;
        };
    }

    public static byte[] buildHandshake(int schemaHash) {
        final ByteBuffer buffer = ByteBuffer.allocate(HANDSHAKE_SIZE).order(ByteOrder.BIG_ENDIAN);
        buffer.put((byte) 'B');
        buffer.put((byte) 'C');
        buffer.put((byte) 'N');
        buffer.put((byte) 'P');
        buffer.putInt(schemaHash);
        return buffer.array();
    }

    public static boolean isHandshakeMagic(byte[] handshake) {
        return handshake.length >= HANDSHAKE_SIZE
                && handshake[0] == 'B'
                && handshake[1] == 'C'
                && handshake[2] == 'N'
                && handshake[3] == 'P';
    }

    public static int readHandshakeSchemaHash(byte[] handshake) {
        return ByteBuffer.wrap(handshake).order(ByteOrder.BIG_ENDIAN).getInt(4);
    }

    public static byte[] encodePacket(int messageType, int flags, int messageCount, byte[] payload) {
        final int payloadLength = payload.length;
        final int totalLength = HEADER_SIZE + payloadLength + CRC_SIZE;

        final ByteBuffer packet = ByteBuffer.allocate(totalLength).order(ByteOrder.BIG_ENDIAN);
        packet.put((byte) PROTOCOL_MAJOR);
        packet.put((byte) PROTOCOL_MINOR);
        packet.put((byte) flags);
        packet.putShort((short) messageType);
        packet.putShort((short) messageCount);
        packet.put(payload);

        final CRC32 crc32 = new CRC32();
        crc32.update(packet.array(), 0, HEADER_SIZE + payloadLength);
        packet.putInt((int) crc32.getValue());
        return packet.array();
    }

    public static DecodedPacket decodePacket(byte[] buffer, int offset, int length) {
        if (length < HEADER_SIZE + CRC_SIZE) {
            return DecodedPacket.error(DecodeError.INCOMPLETE, 0);
        }

        final ByteBuffer view = ByteBuffer.wrap(buffer, offset, length).order(ByteOrder.BIG_ENDIAN);
        final int major = Byte.toUnsignedInt(view.get());
        final int minor = Byte.toUnsignedInt(view.get());
        final int flags = Byte.toUnsignedInt(view.get());
        final int messageType = Short.toUnsignedInt(view.getShort());
        final int messageCount = Short.toUnsignedInt(view.getShort());

        if (major != PROTOCOL_MAJOR || minor != PROTOCOL_MINOR) {
            return DecodedPacket.error(DecodeError.UNSUPPORTED_VERSION, 1);
        }

        final int wireSize = wireSize(messageType);
        if (wireSize <= 0) {
            return DecodedPacket.error(DecodeError.UNKNOWN_MESSAGE_TYPE, 1);
        }

        final int payloadLength = wireSize * messageCount;
        final int frameLength = HEADER_SIZE + payloadLength + CRC_SIZE;
        if (length < frameLength) {
            return DecodedPacket.error(DecodeError.INCOMPLETE, 0);
        }

        final CRC32 crc32 = new CRC32();
        crc32.update(buffer, offset, HEADER_SIZE + payloadLength);
        final int expectedCrc = (int) crc32.getValue();
        final int packetCrc = ByteBuffer.wrap(buffer, offset + HEADER_SIZE + payloadLength, CRC_SIZE)
                .order(ByteOrder.BIG_ENDIAN)
                .getInt();
        if (expectedCrc != packetCrc) {
            return DecodedPacket.error(DecodeError.CRC_MISMATCH, frameLength);
        }

        final byte[] payload = new byte[payloadLength];
        System.arraycopy(buffer, offset + HEADER_SIZE, payload, 0, payloadLength);

        return new DecodedPacket(
                DecodeError.NONE,
                frameLength,
                messageType,
                messageCount,
                flags,
                payload);
    }

    public static Optional<AutoHeartbeatPayload> decodeAutoHeartbeatPayload(byte[] payload) {
        if (payload.length != WIRE_AUTO_HEARTBEAT) {
            return Optional.empty();
        }
        final ByteBuffer view = ByteBuffer.wrap(payload).order(ByteOrder.BIG_ENDIAN);
        return Optional.of(
                new AutoHeartbeatPayload(
                        Integer.toUnsignedLong(view.getInt()),
                        Integer.toUnsignedLong(view.getInt()),
                        Integer.toUnsignedLong(view.getInt())));
    }

    public static Optional<AutoPlanResponsePayload> decodeAutoPlanResponsePayload(byte[] payload) {
        if (payload.length != WIRE_AUTO_PLAN_RESPONSE) {
            return Optional.empty();
        }
        final ByteBuffer view = ByteBuffer.wrap(payload).order(ByteOrder.BIG_ENDIAN);
        final long planId = Integer.toUnsignedLong(view.getInt());
        final int flags = Short.toUnsignedInt(view.getShort());
        final int phaseCount = Short.toUnsignedInt(view.getShort());
        final long planChecksum = Integer.toUnsignedLong(view.getInt());
        final int objectiveId = Short.toUnsignedInt(view.getShort());
        final int policySource = Byte.toUnsignedInt(view.get());
        view.get(); // reserved0
        final int globalConfidencePermille = Short.toUnsignedInt(view.getShort());
        view.getShort(); // reserved1
        return Optional.of(
                new AutoPlanResponsePayload(
                        planId,
                        flags,
                        phaseCount,
                        planChecksum,
                        objectiveId,
                        policySource,
                        globalConfidencePermille));
    }

    public enum DecodeError {
        NONE,
        INCOMPLETE,
        UNSUPPORTED_VERSION,
        UNKNOWN_MESSAGE_TYPE,
        CRC_MISMATCH
    }

    public record DecodedPacket(
            DecodeError error,
            int consumedBytes,
            int messageType,
            int messageCount,
            int flags,
            byte[] payload) {
        static DecodedPacket error(DecodeError error, int consumedBytes) {
            return new DecodedPacket(error, consumedBytes, 0, 0, 0, new byte[0]);
        }

        public boolean isOk() {
            return error == DecodeError.NONE;
        }
    }

    public record AutoHeartbeatPayload(long sessionId, long sequence, long timestampMs) {
    }

    public record AutoPlanResponsePayload(
            long planId,
            int flags,
            int phaseCount,
            long planChecksum,
            int objectiveId,
            int policySource,
            int globalConfidencePermille) {
    }

    public record AutoShotHintPayload(
            int aimOffsetMrad,
            int shooterRpm,
            int hoodPositionPermille,
            int fireWindowMrad,
            int confidencePermille,
            int distanceToHubMm,
            int flags) {
    }

    public static Optional<AutoShotHintPayload> decodeAutoShotHintPayload(byte[] payload) {
        if (payload.length != WIRE_AUTO_SHOT_HINT) {
            return Optional.empty();
        }
        final ByteBuffer view = ByteBuffer.wrap(payload).order(ByteOrder.BIG_ENDIAN);
        final int aimOffsetMrad = view.getShort();
        final int shooterRpm = Short.toUnsignedInt(view.getShort());
        final int hoodPositionPermille = Short.toUnsignedInt(view.getShort());
        final int fireWindowMrad = Short.toUnsignedInt(view.getShort());
        final int confidencePermille = Short.toUnsignedInt(view.getShort());
        final int distanceToHubMm = Short.toUnsignedInt(view.getShort());
        final int flags = Short.toUnsignedInt(view.getShort());
        view.getShort(); // reserved
        return Optional.of(new AutoShotHintPayload(
                aimOffsetMrad, shooterRpm, hoodPositionPermille,
                fireWindowMrad, confidencePermille, distanceToHubMm, flags));
    }

    public record AutoWaypointDeltaPayload(
            long planId,
            int phaseSeq,
            int waypointIndex,
            int waypointCount,
            int xMm,
            int yMm,
            int headingMrad,
            int maxVelocityMmS) {
    }

    public static Optional<AutoWaypointDeltaPayload> decodeAutoWaypointDeltaPayload(byte[] payload) {
        if (payload.length != WIRE_AUTO_WAYPOINT_DELTA) {
            return Optional.empty();
        }
        final ByteBuffer view = ByteBuffer.wrap(payload).order(ByteOrder.BIG_ENDIAN);
        final long planId = Integer.toUnsignedLong(view.getInt());
        final int phaseSeq = Short.toUnsignedInt(view.getShort());
        final int waypointIndex = Byte.toUnsignedInt(view.get());
        final int waypointCount = Byte.toUnsignedInt(view.get());
        final int xMm = view.getInt();
        final int yMm = view.getInt();
        final int headingMrad = view.getShort();
        final int maxVelocityMmS = Short.toUnsignedInt(view.getShort());
        return Optional.of(new AutoWaypointDeltaPayload(
                planId, phaseSeq, waypointIndex, waypointCount,
                xMm, yMm, headingMrad, maxVelocityMmS));
    }

    public static byte[] encodeAutoWorldUpdate(
            int fuelHeld,
            boolean lastShotSuccess,
            int phaseSeqCompleted,
            int eventFlags,
            int robotVxMmS,
            int robotVyMmS,
            int poseXmm,
            int poseYmm,
            int headingMrad) {
        final ByteBuffer payload = ByteBuffer.allocate(WIRE_AUTO_WORLD_UPDATE).order(ByteOrder.BIG_ENDIAN);
        payload.put((byte) (fuelHeld & 0xFF));
        payload.put((byte) (lastShotSuccess ? 1 : 0));
        payload.putShort((short) phaseSeqCompleted);
        payload.putShort((short) eventFlags);
        payload.putShort((short) robotVxMmS);
        payload.putShort((short) robotVyMmS);
        payload.putInt(poseXmm);
        payload.putInt(poseYmm);
        payload.putShort((short) headingMrad);
        payload.putShort((short) 0); // reserved
        return encodePacket(MSG_AUTO_WORLD_UPDATE, 0, 1, payload.array());
    }

    public static byte[] encodeAutoOpponentUpdate(
            int trackIndex,
            int trackCount,
            int poseXmm,
            int poseYmm,
            int velocityXmmS,
            int velocityYmmS,
            int confidencePermille) {
        final ByteBuffer payload = ByteBuffer.allocate(WIRE_AUTO_OPPONENT_UPDATE).order(ByteOrder.BIG_ENDIAN);
        payload.put((byte) (trackIndex & 0xFF));
        payload.put((byte) (trackCount & 0xFF));
        payload.putInt(poseXmm);
        payload.putInt(poseYmm);
        payload.putShort((short) velocityXmmS);
        payload.putShort((short) velocityYmmS);
        payload.putShort((short) confidencePermille);
        payload.putShort((short) 0); // reserved
        return encodePacket(MSG_AUTO_OPPONENT_UPDATE, 0, 1, payload.array());
    }
}
