package frc.robot.commands.auto.bcnp;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.NotYetConnectedException;
import java.nio.channels.SocketChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.auto.AutoLinkState;
import frc.robot.commands.auto.AutoPlannerClient;

// BCNP TCP session manager for offboard autonomous planning.

public final class BcnpTcpPlannerClient implements AutoPlannerClient {
    private static final int kReadBufferBytes = 4096;
    private static final int kPendingBufferBytes = 32768;

    private final String host;
    private final int port;
    private final int expectedSchemaHash;
    private final long connectRetryMs;
    private final long heartbeatPeriodMs;
    private final long heartbeatTimeoutMs;

    private SocketChannel channel;
    private AutoLinkState linkState = AutoLinkState.DISCONNECTED;
    private String lastFault = "none";

    private long lastConnectAttemptMs = 0;
    private long lastRxMs = 0;
    private long lastTxHeartbeatMs = 0;
    private long lastTxPlanRequestMs = 0;

    private boolean handshakeSent = false;
    private boolean handshakeValidated = false;
    private final byte[] handshakeRx = new byte[BcnpAutoProtocol.HANDSHAKE_SIZE];
    private int handshakeRxLen = 0;

    private final ByteBuffer readBuffer = ByteBuffer.allocate(kReadBufferBytes);
    private final byte[] pending = new byte[kPendingBufferBytes];
    private int pendingLength = 0;

    private final int sessionId = ThreadLocalRandom.current().nextInt(1, Integer.MAX_VALUE);
    private int heartbeatSequence = 0;
    private long lastRemoteHeartbeatSequence = -1;

    private int selectedProfile = 0;
    private Pose2d latestPose = Pose2d.kZero;
    private Alliance latestAlliance = Alliance.Blue;
    private volatile RemotePlan latestRemotePlan;

    // Dynamic planning data from coprocessor
    private volatile BcnpAutoProtocol.AutoShotHintPayload latestShotHint;
    private long shotHintReceivedAtMs = 0;
    private final Map<Integer, List<BcnpAutoProtocol.AutoWaypointDeltaPayload>> waypointsByPhase = new HashMap<>();

    public BcnpTcpPlannerClient(
            String host,
            int port,
            int expectedSchemaHash,
            long connectRetryMs,
            long heartbeatPeriodMs,
            long heartbeatTimeoutMs) {
        this.host = host;
        this.port = port;
        this.expectedSchemaHash = expectedSchemaHash;
        this.connectRetryMs = connectRetryMs;
        this.heartbeatPeriodMs = heartbeatPeriodMs;
        this.heartbeatTimeoutMs = heartbeatTimeoutMs;
    }

    @Override
    public AutoLinkState linkState() {
        return linkState;
    }

    @Override
    public boolean isHealthy() {
        return linkState == AutoLinkState.HEALTHY;
    }

    @Override
    public Optional<RemotePlan> latestPlan() {
        return Optional.ofNullable(latestRemotePlan);
    }

    @Override
    public String lastFault() {
        return lastFault;
    }

    @Override
    public double heartbeatAgeSeconds() {
        if (lastRxMs <= 0) {
            return -1.0;
        }
        return (System.currentTimeMillis() - lastRxMs) / 1000.0;
    }

    public void setPlanRequestContext(int selectedProfile, Pose2d pose, Alliance alliance) {
        this.selectedProfile = selectedProfile;
        this.latestPose = pose;
        this.latestAlliance = alliance;
    }

    // Latest shot hint from the coprocessor, if available and fresh.

    public Optional<BcnpAutoProtocol.AutoShotHintPayload> latestShotHint(long freshnessWindowMs) {
        final BcnpAutoProtocol.AutoShotHintPayload hint = latestShotHint;
        if (hint == null)
            return Optional.empty();
        if (System.currentTimeMillis() - shotHintReceivedAtMs > freshnessWindowMs)
            return Optional.empty();
        return Optional.of(hint);
    }

    // Coprocessor waypoints for specific phase sequence number.
    public List<BcnpAutoProtocol.AutoWaypointDeltaPayload> waypointsForPhase(int phaseSeq) {
        synchronized (waypointsByPhase) {
            return Collections.unmodifiableList(
                    waypointsByPhase.getOrDefault(phaseSeq, Collections.emptyList()));
        }
    }

    // Send opponent tracked positions to the coprocessor.
    public void sendOpponentUpdates(List<OpponentTrack> tracks) {
        if (channel == null || !handshakeValidated)
            return;
        try {
            if (tracks.isEmpty()) {
                final byte[] packet = BcnpAutoProtocol.encodeAutoOpponentUpdate(
                        0, 0, 0, 0, 0, 0, 0);
                channel.write(ByteBuffer.wrap(packet));
                return;
            }
            for (int i = 0; i < tracks.size(); i++) {
                final OpponentTrack track = tracks.get(i);
                final byte[] packet = BcnpAutoProtocol.encodeAutoOpponentUpdate(
                        i,
                        tracks.size(),
                        (int) Math.round(track.pose().getX() * 1000.0),
                        (int) Math.round(track.pose().getY() * 1000.0),
                        (int) Math.round(track.vxMps() * 1000.0),
                        (int) Math.round(track.vyMps() * 1000.0),
                        track.confidencePermille());
                channel.write(ByteBuffer.wrap(packet));
            }
        } catch (IOException e) {
            disconnect();
        }
    }

    public record OpponentTrack(
            Pose2d pose,
            double vxMps,
            double vyMps,
            int confidencePermille) {
    }

    // Send world update w/ current robot state.
    public void sendWorldUpdate(
            int fuelHeld,
            boolean lastShotSuccess,
            int phaseSeqCompleted,
            int eventFlags,
            Pose2d pose,
            double vxMps,
            double vyMps) {
        if (channel == null || !handshakeValidated)
            return;
        try {
            final byte[] packet = BcnpAutoProtocol.encodeAutoWorldUpdate(
                    fuelHeld,
                    lastShotSuccess,
                    phaseSeqCompleted,
                    eventFlags,
                    (int) Math.round(vxMps * 1000.0),
                    (int) Math.round(vyMps * 1000.0),
                    (int) Math.round(pose.getX() * 1000.0),
                    (int) Math.round(pose.getY() * 1000.0),
                    (int) Math.round(pose.getRotation().getRadians() * 1000.0));
            writeAllOrFail(packet, "TX_WORLD_UPDATE_PARTIAL");
        } catch (IOException e) {
            setFault("TX_WORLD_UPDATE", e.getMessage());
            disconnect();
        }
    }

    public void updatePoseContext(Pose2d pose, Alliance alliance) {
        this.latestPose = pose;
        this.latestAlliance = alliance;
    }

    @Override
    public void periodic() {
        final long now = System.currentTimeMillis();

        if (channel == null || !channel.isOpen()) {
            maybeConnect(now);
            return;
        }

        try {
            if (linkState == AutoLinkState.CONNECTING) {
                finishConnect(now);
                if (!isChannelReadyForIo()) {
                    return;
                }
            }

            readIncoming(now);
            if (!handshakeValidated) {
                processHandshake(now);
                return;
            }

            processPackets(now);
            sendPeriodicHeartbeat(now);
            sendPeriodicPlanRequest(now);
            enforceHeartbeatTimeout(now);
        } catch (IOException e) {
            setFault("IO_EXCEPTION", e.getMessage());
            disconnect();
        }
    }

    @Override
    public void close() {
        disconnect();
    }

    private void maybeConnect(long now) {
        if (now - lastConnectAttemptMs < connectRetryMs) {
            return;
        }
        lastConnectAttemptMs = now;

        try {
            channel = SocketChannel.open();
            channel.configureBlocking(false);
            channel.connect(new InetSocketAddress(host, port));
            linkState = AutoLinkState.CONNECTING;
            resetSessionState();
        } catch (IOException e) {
            setFault("CONNECT_FAILED", e.getMessage());
            disconnect();
        }
    }

    private void finishConnect(long now) throws IOException {
        if (channel.finishConnect()) {
            linkState = AutoLinkState.CONNECTING;
            sendHandshake();
            lastRxMs = now;
        }
    }

    private void readIncoming(long now) throws IOException {
        readBuffer.clear();
        final int read;
        try {
            read = channel.read(readBuffer);
        } catch (NotYetConnectedException e) {
            return;
        }
        if (read < 0) {
            setFault("REMOTE_CLOSED", "Planner endpoint closed connection.");
            disconnect();
            return;
        }
        if (read == 0) {
            return;
        }

        lastRxMs = now;
        readBuffer.flip();
        appendPending(readBuffer.array(), readBuffer.position(), readBuffer.remaining());
    }

    private void processHandshake(long now) {
        final int needed = BcnpAutoProtocol.HANDSHAKE_SIZE - handshakeRxLen;
        if (pendingLength < needed) {
            return;
        }

        System.arraycopy(pending, 0, handshakeRx, handshakeRxLen, needed);
        handshakeRxLen += needed;
        consumePending(needed);

        if (!BcnpAutoProtocol.isHandshakeMagic(handshakeRx)) {
            setFault("BAD_HANDSHAKE_MAGIC", "Remote endpoint did not provide BCNP handshake magic.");
            disconnect();
            return;
        }

        final int remoteSchemaHash = BcnpAutoProtocol.readHandshakeSchemaHash(handshakeRx);
        if (remoteSchemaHash != expectedSchemaHash) {
            setFault(
                    "SCHEMA_HASH_MISMATCH",
                    "Expected schema hash 0x" + Integer.toHexString(expectedSchemaHash)
                            + " but received 0x" + Integer.toHexString(remoteSchemaHash));
            disconnect();
            return;
        }

        handshakeValidated = true;
        linkState = AutoLinkState.HEALTHY;
        lastRxMs = now;
    }

    private void processPackets(long now) {
        while (pendingLength >= BcnpAutoProtocol.HEADER_SIZE + BcnpAutoProtocol.CRC_SIZE) {
            final BcnpAutoProtocol.DecodedPacket decoded = BcnpAutoProtocol.decodePacket(pending, 0, pendingLength);
            if (decoded.error() == BcnpAutoProtocol.DecodeError.INCOMPLETE) {
                return;
            }
            if (!decoded.isOk()) {
                setFault("DECODE_" + decoded.error().name(), "Malformed BCNP packet received.");
                if (decoded.consumedBytes() > 0) {
                    consumePending(decoded.consumedBytes());
                } else {
                    disconnect();
                    return;
                }
                continue;
            }

            consumePending(decoded.consumedBytes());
            handleDecodedPacket(decoded, now);
        }
    }

    private void handleDecodedPacket(BcnpAutoProtocol.DecodedPacket decoded, long now) {
        switch (decoded.messageType()) {
            case BcnpAutoProtocol.MSG_AUTO_HEARTBEAT -> {
                final Optional<BcnpAutoProtocol.AutoHeartbeatPayload> heartbeatPayload = BcnpAutoProtocol
                        .decodeAutoHeartbeatPayload(decoded.payload());
                if (heartbeatPayload.isEmpty()) {
                    setFault("HEARTBEAT_PAYLOAD_SIZE", "Heartbeat payload size mismatch.");
                    disconnect();
                    return;
                }

                final long remoteSequence = heartbeatPayload.get().sequence();
                if (lastRemoteHeartbeatSequence >= 0 && remoteSequence <= lastRemoteHeartbeatSequence) {
                    setFault(
                            "HEARTBEAT_SEQUENCE_STALE",
                            "Expected strictly increasing heartbeat sequence but received " + remoteSequence
                                    + " after " + lastRemoteHeartbeatSequence);
                    disconnect();
                    return;
                }
                lastRemoteHeartbeatSequence = remoteSequence;
                linkState = AutoLinkState.HEALTHY;
                lastRxMs = now;
            }
            case BcnpAutoProtocol.MSG_AUTO_PLAN_RESPONSE -> {
                final Optional<BcnpAutoProtocol.AutoPlanResponsePayload> planPayload = BcnpAutoProtocol
                        .decodeAutoPlanResponsePayload(decoded.payload());
                if (planPayload.isEmpty()) {
                    setFault("PLAN_RESPONSE_PAYLOAD_SIZE", "Plan response payload size mismatch.");
                    disconnect();
                    return;
                }
                final BcnpAutoProtocol.AutoPlanResponsePayload payload = planPayload.get();
                latestRemotePlan = new RemotePlan(
                        selectedProfile,
                        payload.planId(),
                        payload.flags(),
                        payload.phaseCount(),
                        payload.planChecksum(),
                        payload.objectiveId(),
                        payload.policySource(),
                        payload.globalConfidencePermille(),
                        now);
                linkState = AutoLinkState.HEALTHY;
                lastRxMs = now;
            }
            case BcnpAutoProtocol.MSG_AUTO_ABORT -> {
                setFault("PLANNER_ABORT", "Planner sent AUTO abort packet.");
            }
            case BcnpAutoProtocol.MSG_AUTO_SHOT_HINT -> {
                final Optional<BcnpAutoProtocol.AutoShotHintPayload> shotHintPayload = BcnpAutoProtocol
                        .decodeAutoShotHintPayload(decoded.payload());
                if (shotHintPayload.isPresent()) {
                    latestShotHint = shotHintPayload.get();
                    shotHintReceivedAtMs = now;
                }
                lastRxMs = now;
            }
            case BcnpAutoProtocol.MSG_AUTO_WAYPOINT_DELTA -> {
                final Optional<BcnpAutoProtocol.AutoWaypointDeltaPayload> waypointPayload = BcnpAutoProtocol
                        .decodeAutoWaypointDeltaPayload(decoded.payload());
                if (waypointPayload.isPresent()) {
                    final BcnpAutoProtocol.AutoWaypointDeltaPayload wp = waypointPayload.get();
                    synchronized (waypointsByPhase) {
                        // If this is the first waypoint for a new plan, clear old data
                        if (wp.waypointIndex() == 0 && wp.phaseSeq() == 1) {
                            waypointsByPhase.clear();
                        }
                        waypointsByPhase
                                .computeIfAbsent(wp.phaseSeq(), k -> new ArrayList<>())
                                .add(wp);
                    }
                }
                lastRxMs = now;
            }
            default -> {
                // Unknown-to-client packet types are tolerated when schema IDs are known.
                lastRxMs = now;
            }
        }
    }

    private void sendHandshake() throws IOException {
        if (handshakeSent) {
            return;
        }
        final byte[] handshake = BcnpAutoProtocol.buildHandshake(expectedSchemaHash);
        writeAllOrFail(handshake, "TX_HANDSHAKE_PARTIAL");
        handshakeSent = true;
    }

    private void sendPeriodicHeartbeat(long now) throws IOException {
        if (now - lastTxHeartbeatMs < heartbeatPeriodMs) {
            return;
        }

        final ByteBuffer payload = ByteBuffer.allocate(BcnpAutoProtocol.WIRE_AUTO_HEARTBEAT)
                .order(ByteOrder.BIG_ENDIAN);
        payload.putInt(sessionId);
        payload.putInt(heartbeatSequence++);
        payload.putInt((int) now);

        final byte[] packet = BcnpAutoProtocol.encodePacket(
                BcnpAutoProtocol.MSG_AUTO_HEARTBEAT,
                0,
                1,
                payload.array());
        writeAllOrFail(packet, "TX_HEARTBEAT_PARTIAL");
        lastTxHeartbeatMs = now;
    }

    private void sendPeriodicPlanRequest(long now) throws IOException {
        if (now - lastTxPlanRequestMs < 1000) {
            return;
        }

        final ByteBuffer payload = ByteBuffer.allocate(BcnpAutoProtocol.WIRE_AUTO_PLAN_REQUEST)
                .order(ByteOrder.BIG_ENDIAN);
        payload.putShort((short) selectedProfile);
        payload.put((byte) (latestAlliance == Alliance.Red ? 1 : 0));
        payload.put((byte) 0); // reserved
        payload.putInt((int) Math.round(latestPose.getX() * 1000.0)); // millimeters
        payload.putInt((int) Math.round(latestPose.getY() * 1000.0)); // millimeters
        payload.putInt((int) Math.round(latestPose.getRotation().getRadians() * 1000.0)); // milliradians

        final byte[] packet = BcnpAutoProtocol.encodePacket(
                BcnpAutoProtocol.MSG_AUTO_PLAN_REQUEST,
                0,
                1,
                payload.array());
        writeAllOrFail(packet, "TX_PLAN_REQUEST_PARTIAL");
        lastTxPlanRequestMs = now;
    }

    private void enforceHeartbeatTimeout(long now) {
        if (lastRxMs <= 0) {
            return;
        }
        if (now - lastRxMs > heartbeatTimeoutMs) {
            linkState = AutoLinkState.DEGRADED;
            setFault("HEARTBEAT_TIMEOUT", "Planner heartbeat timed out.");
            disconnect();
        }
    }

    private void appendPending(byte[] src, int srcOffset, int length) {
        if (length <= 0) {
            return;
        }
        if (pendingLength + length > pending.length) {
            setFault("PENDING_OVERFLOW", "Incoming BCNP stream exceeded pending buffer.");
            disconnect();
            return;
        }
        System.arraycopy(src, srcOffset, pending, pendingLength, length);
        pendingLength += length;
    }

    private void consumePending(int consumedBytes) {
        if (consumedBytes <= 0 || consumedBytes > pendingLength) {
            return;
        }
        final int remaining = pendingLength - consumedBytes;
        if (remaining > 0) {
            System.arraycopy(pending, consumedBytes, pending, 0, remaining);
        }
        pendingLength = remaining;
    }

    private void writeAllOrFail(byte[] bytes, String partialFaultCode) throws IOException {
        final ByteBuffer buffer = ByteBuffer.wrap(bytes);
        final int written = channel.write(buffer);
        if (written != bytes.length) {
            setFault(partialFaultCode, "Non-blocking socket accepted only partial packet write.");
            disconnect();
        }
    }

    private void resetSessionState() {
        handshakeSent = false;
        handshakeValidated = false;
        handshakeRxLen = 0;
        pendingLength = 0;
        Arrays.fill(handshakeRx, (byte) 0);
        lastTxHeartbeatMs = 0;
        lastTxPlanRequestMs = 0;
        lastRemoteHeartbeatSequence = -1;
        latestRemotePlan = null;
        latestShotHint = null;
        synchronized (waypointsByPhase) {
            waypointsByPhase.clear();
        }
    }

    private void setFault(String code, String detail) {
        lastFault = code + ": " + detail;
    }

    private void disconnect() {
        linkState = AutoLinkState.DISCONNECTED;
        if (channel != null) {
            try {
                channel.close();
            } catch (IOException ignored) {
            }
            channel = null;
        }
    }

    private boolean isChannelReadyForIo() {
        return channel != null && channel.isOpen() && channel.isConnected();
    }
}
