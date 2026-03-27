package frc.robot.commands.auto;

import java.util.Optional;

// Interface for planner connectivity (coprocessor or local-only).
public interface AutoPlannerClient extends AutoCloseable {
    int POLICY_SOURCE_LOCAL = 0;
    int POLICY_SOURCE_CLASSICAL = 1;
    int POLICY_SOURCE_LEARNED_SHADOW = 2;
    int POLICY_SOURCE_LEARNED_ACTIVE = 3;

    record RemotePlan(
            int profileId,
            long planId,
            int flags,
            int phaseCount,
            long planChecksum,
            int objectiveId,
            int policySource,
            int globalConfidencePermille,
            long receivedAtMs) {
        public boolean isFresh(long nowMs, long freshnessWindowMs) {
            return nowMs - receivedAtMs <= freshnessWindowMs;
        }
    }

    AutoLinkState linkState();

    boolean isHealthy();

    default Optional<RemotePlan> latestPlan() {
        return Optional.empty();
    }

    default String lastFault() {
        return "none";
    }

    default double heartbeatAgeSeconds() {
        return -1.0;
    }

    default void periodic() {
    }

    @Override
    default void close() {
    }
}
