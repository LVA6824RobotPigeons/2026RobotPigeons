package frc.robot.commands.auto;

// Health classification for autonomous planner connectivity.
public enum AutoLinkState {
    LOCAL_ONLY,
    CONNECTING,
    HEALTHY,
    DEGRADED,
    DISCONNECTED
}
