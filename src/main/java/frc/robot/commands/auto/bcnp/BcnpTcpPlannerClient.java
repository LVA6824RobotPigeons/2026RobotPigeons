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
    private static final int kFieldLengthMm = 16541;
    private static final int kFieldWidthMm = 8211;
    private static final int kMaxPhaseSeq = 16;
    private static final int kMaxWaypointsPerPhase = 64;
    private static final int kMaxTotalWaypoints = 512;
    private static final int kMinWaypointVelocityMmS = 100;
    private static final int kMaxWaypointVelocityMmS = 5000;
    private static final int kMinShooterRpm = 1000;
    private static final int kMaxShooterRpm = 6000;
    private static final int kMinHoodPermille = 0;
    private static final int kMaxHoodPermille = 1000;
    private static final int kMinMrad = -3142;
    private static final int kMaxMrad = 3142;
    private static final int kMinConfidencePermille = 0;
    private static final int kMaxConfidencePermille = 1000;
    private static final int kMaxDistanceToHubMm = 20000;

    private final String host;
    private final int port;
    private final int expectedSchemaHash;
    private final long connectRetryMs;
    private final long heartbeatPeriodMs;
    private final long heartbeatTimeoutMs;
    private final BcnpValidationMode validationMode;

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
    private long latestWaypointPlanId = -1;

    private long rejectedPacketCount = 0;
    private long clampedPacketCount = 0;
    private long unsupportedPacketCount = 0;

    public BcnpTcpPlannerClient(
            String host,
            int port,
            int expectedSchemaHash,
            long connectRetryMs,
            long heartbeatPeriodMs,
            long heartbeatTimeoutMs) {
        this(host, port, expectedSchemaHash, connectRetryMs, heartbeatPeriodMs, heartbeatTimeoutMs, BcnpValidationMode.STRICT);
    }

    public BcnpTcpPlannerClient(
            String host,
            int port,
            int expectedSchemaHash,
            long connectRetryMs,
            long heartbeatPeriodMs,
            long heartbeatTimeoutMs,
            BcnpValidationMode validationMode) {
        this.host = host;
        this.port = port;
        this.expectedSchemaHash = expectedSchemaHash;
        this.connectRetryMs = connectRetryMs;
        this.heartbeatPeriodMs = heartbeatPeriodMs;
        this.heartbeatTimeoutMs = heartbeatTimeoutMs;
        this.validationMode = validationMode;
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

    public BcnpValidationMode validationMode() {
        return validationMode;
    }

    public long rejectedPacketCount() {
        return rejectedPacketCount;
    }

    public long clampedPacketCount() {
        return clampedPacketCount;
    }

    public long unsupportedPacketCount() {
        return unsupportedPacketCount;
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
                final Optional<BcnpAutoProtocol.AutoPlanResponsePayload> sanitized = sanitizePlanResponse(planPayload.get());
                if (sanitized.isEmpty()) {
                    return;
                }
                final BcnpAutoProtocol.AutoPlanResponsePayload payload = sanitized.get();
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
                    final Optional<BcnpAutoProtocol.AutoShotHintPayload> sanitized = sanitizeShotHint(shotHintPayload.get());
                    if (sanitized.isPresent()) {
                        latestShotHint = sanitized.get();
                        shotHintReceivedAtMs = now;
                    }
                }
                lastRxMs = now;
            }
            case BcnpAutoProtocol.MSG_AUTO_WAYPOINT_DELTA -> {
                final Optional<BcnpAutoProtocol.AutoWaypointDeltaPayload> waypointPayload = BcnpAutoProtocol
                        .decodeAutoWaypointDeltaPayload(decoded.payload());
                if (waypointPayload.isPresent()) {
                    final Optional<BcnpAutoProtocol.AutoWaypointDeltaPayload> sanitized = sanitizeWaypoint(waypointPayload.get());
                    if (sanitized.isEmpty()) {
                        return;
                    }
                    final BcnpAutoProtocol.AutoWaypointDeltaPayload wp = sanitized.get();
                    synchronized (waypointsByPhase) {
                        if (latestWaypointPlanId < 0 || wp.planId() > latestWaypointPlanId) {
                            waypointsByPhase.clear();
                            latestWaypointPlanId = wp.planId();
                        } else if (wp.planId() < latestWaypointPlanId) {
                            reject("WAYPOINT_STALE_PLAN", "Ignoring stale waypoint planId=" + wp.planId());
                            return;
                        } else if (waypointsByPhase.values().stream().mapToInt(List::size).sum() >= kMaxTotalWaypoints) {
                            reject("WAYPOINT_CAP", "Ignoring waypoint because cache cap was reached.");
                            return;
                        }
                        waypointsByPhase
                                .computeIfAbsent(wp.phaseSeq(), k -> new ArrayList<>())
                                .add(wp);
                    }
                }
                lastRxMs = now;
            }
            case BcnpAutoProtocol.MSG_AUTO_PHASE_COMMAND,
                 BcnpAutoProtocol.MSG_AUTO_PHASE_ACK,
                 BcnpAutoProtocol.MSG_AUTO_TELEMETRY -> {
                unsupportedPacketCount++;
                setFault("UNSUPPORTED_MESSAGE", "Received unsupported message type " + decoded.messageType() + ".");
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
        latestWaypointPlanId = -1;
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

    private Optional<BcnpAutoProtocol.AutoPlanResponsePayload> sanitizePlanResponse(
            BcnpAutoProtocol.AutoPlanResponsePayload payload) {
        final boolean[] clamped = new boolean[] { false };
        final Long planId = clampOrRejectLong("PLAN_ID", payload.planId(), 1, Integer.toUnsignedLong(Integer.MAX_VALUE), clamped);
        if (planId == null) {
            return Optional.empty();
        }
        final Integer phaseCount = clampOrRejectInt("PLAN_PHASE_COUNT", payload.phaseCount(), 1, kMaxPhaseSeq, clamped);
        if (phaseCount == null) {
            return Optional.empty();
        }
        final Integer policySource = clampOrRejectInt(
                "PLAN_POLICY_SOURCE",
                payload.policySource(),
                AutoPlannerClient.POLICY_SOURCE_LOCAL,
                AutoPlannerClient.POLICY_SOURCE_LEARNED_ACTIVE,
                clamped);
        if (policySource == null) {
            return Optional.empty();
        }
        final Integer confidence = clampOrRejectInt(
                "PLAN_CONFIDENCE",
                payload.globalConfidencePermille(),
                kMinConfidencePermille,
                kMaxConfidencePermille,
                clamped);
        if (confidence == null) {
            return Optional.empty();
        }
        onClamped(clamped[0], "PLAN_RESPONSE_CLAMP");
        return Optional.of(new BcnpAutoProtocol.AutoPlanResponsePayload(
                planId,
                payload.flags(),
                phaseCount,
                payload.planChecksum(),
                payload.objectiveId(),
                policySource,
                confidence));
    }

    private Optional<BcnpAutoProtocol.AutoShotHintPayload> sanitizeShotHint(
            BcnpAutoProtocol.AutoShotHintPayload payload) {
        final boolean[] clamped = new boolean[] { false };
        final Integer aimOffsetMrad = clampOrRejectInt("SHOT_AIM_OFFSET", payload.aimOffsetMrad(), kMinMrad, kMaxMrad, clamped);
        if (aimOffsetMrad == null) {
            return Optional.empty();
        }
        final Integer shooterRpm = clampOrRejectInt("SHOT_RPM", payload.shooterRpm(), kMinShooterRpm, kMaxShooterRpm, clamped);
        if (shooterRpm == null) {
            return Optional.empty();
        }
        final Integer hoodPositionPermille = clampOrRejectInt(
                "SHOT_HOOD",
                payload.hoodPositionPermille(),
                kMinHoodPermille,
                kMaxHoodPermille,
                clamped);
        if (hoodPositionPermille == null) {
            return Optional.empty();
        }
        final Integer fireWindowMrad = clampOrRejectInt("SHOT_FIRE_WINDOW", payload.fireWindowMrad(), 0, kMaxMrad, clamped);
        if (fireWindowMrad == null) {
            return Optional.empty();
        }
        final Integer confidencePermille = clampOrRejectInt(
                "SHOT_CONFIDENCE",
                payload.confidencePermille(),
                kMinConfidencePermille,
                kMaxConfidencePermille,
                clamped);
        if (confidencePermille == null) {
            return Optional.empty();
        }
        final Integer distanceToHubMm = clampOrRejectInt(
                "SHOT_DISTANCE",
                payload.distanceToHubMm(),
                0,
                kMaxDistanceToHubMm,
                clamped);
        if (distanceToHubMm == null) {
            return Optional.empty();
        }
        onClamped(clamped[0], "SHOT_HINT_CLAMP");
        return Optional.of(new BcnpAutoProtocol.AutoShotHintPayload(
                aimOffsetMrad,
                shooterRpm,
                hoodPositionPermille,
                fireWindowMrad,
                confidencePermille,
                distanceToHubMm,
                payload.flags()));
    }

    private Optional<BcnpAutoProtocol.AutoWaypointDeltaPayload> sanitizeWaypoint(
            BcnpAutoProtocol.AutoWaypointDeltaPayload payload) {
        final boolean[] clamped = new boolean[] { false };
        final Long planId = clampOrRejectLong("WAYPOINT_PLAN_ID", payload.planId(), 1, Integer.toUnsignedLong(Integer.MAX_VALUE), clamped);
        if (planId == null) {
            return Optional.empty();
        }
        final Integer phaseSeq = clampOrRejectInt("WAYPOINT_PHASE_SEQ", payload.phaseSeq(), 1, kMaxPhaseSeq, clamped);
        if (phaseSeq == null) {
            return Optional.empty();
        }
        final Integer waypointCount = clampOrRejectInt("WAYPOINT_COUNT", payload.waypointCount(), 1, kMaxWaypointsPerPhase, clamped);
        if (waypointCount == null) {
            return Optional.empty();
        }
        final Integer waypointIndex = clampOrRejectInt("WAYPOINT_INDEX", payload.waypointIndex(), 0, waypointCount - 1, clamped);
        if (waypointIndex == null) {
            return Optional.empty();
        }
        final Integer xMm = clampOrRejectInt("WAYPOINT_X", payload.xMm(), 0, kFieldLengthMm, clamped);
        if (xMm == null) {
            return Optional.empty();
        }
        final Integer yMm = clampOrRejectInt("WAYPOINT_Y", payload.yMm(), 0, kFieldWidthMm, clamped);
        if (yMm == null) {
            return Optional.empty();
        }
        final Integer headingMrad = clampOrRejectInt("WAYPOINT_HEADING", payload.headingMrad(), kMinMrad, kMaxMrad, clamped);
        if (headingMrad == null) {
            return Optional.empty();
        }
        final Integer maxVelocityMmS = clampOrRejectInt(
                "WAYPOINT_MAX_VELOCITY",
                payload.maxVelocityMmS(),
                kMinWaypointVelocityMmS,
                kMaxWaypointVelocityMmS,
                clamped);
        if (maxVelocityMmS == null) {
            return Optional.empty();
        }
        onClamped(clamped[0], "WAYPOINT_CLAMP");
        return Optional.of(new BcnpAutoProtocol.AutoWaypointDeltaPayload(
                planId,
                phaseSeq,
                waypointIndex,
                waypointCount,
                xMm,
                yMm,
                headingMrad,
                maxVelocityMmS));
    }

    private Integer clampOrRejectInt(String code, int value, int min, int max, boolean[] clamped) {
        if (value < min || value > max) {
            if (validationMode == BcnpValidationMode.STRICT) {
                reject(code, "Value " + value + " out of range [" + min + "," + max + "].");
                return null;
            }
            clamped[0] = true;
            return Math.max(min, Math.min(max, value));
        }
        return value;
    }

    private Long clampOrRejectLong(String code, long value, long min, long max, boolean[] clamped) {
        if (value < min || value > max) {
            if (validationMode == BcnpValidationMode.STRICT) {
                reject(code, "Value " + value + " out of range [" + min + "," + max + "].");
                return null;
            }
            clamped[0] = true;
            return Math.max(min, Math.min(max, value));
        }
        return value;
    }

    private void reject(String code, String detail) {
        rejectedPacketCount++;
        setFault(code, detail);
    }

    private void onClamped(boolean clamped, String code) {
        if (!clamped) {
            return;
        }
        clampedPacketCount++;
        setFault(code, "Inbound planner payload was clamped to safe bounds.");
    }
}
