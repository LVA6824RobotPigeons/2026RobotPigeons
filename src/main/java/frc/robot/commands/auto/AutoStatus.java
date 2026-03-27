package frc.robot.commands.auto;

import frc.robot.commands.auto.AutoSafetyGate.ValidationResult;

// Mutable status object for autonomous progress/diagnostics.

public final class AutoStatus {
    private String routineId = "none";
    private String phaseId = "idle";
    private AutoLinkState linkState = AutoLinkState.LOCAL_ONLY;
    private PlannerExecutionMode plannerMode = PlannerExecutionMode.LOCAL_ONLY;
    private boolean fallbackActive = false;
    private boolean completed = false;
    private boolean aborted = false;
    private String lastGateCode = "none";
    private String lastGateDetail = "none";

    public synchronized void startRoutine(String routineId, AutoLinkState linkState, PlannerExecutionMode plannerMode) {
        this.routineId = routineId;
        this.linkState = linkState;
        this.plannerMode = plannerMode;
        this.phaseId = "starting";
        this.fallbackActive = false;
        this.completed = false;
        this.aborted = false;
        this.lastGateCode = "none";
        this.lastGateDetail = "none";
    }

    public synchronized void startPhase(String phaseId) {
        this.phaseId = phaseId;
    }

    public synchronized void markFallbackActive(String phaseId) {
        this.phaseId = phaseId;
        this.fallbackActive = true;
    }

    public synchronized void recordGateEvent(ValidationResult result) {
        this.lastGateCode = result.code();
        this.lastGateDetail = result.detail();
    }

    public synchronized void markFallbackReason(String code, String detail) {
        this.lastGateCode = code;
        this.lastGateDetail = detail;
    }

    public synchronized void markCompleted() {
        this.phaseId = "completed";
        this.completed = true;
    }

    public synchronized void markAborted(String reason) {
        this.phaseId = "aborted";
        this.aborted = true;
        this.lastGateCode = "ABORT";
        this.lastGateDetail = reason;
    }

    public synchronized String routineId() {
        return routineId;
    }

    public synchronized String phaseId() {
        return phaseId;
    }

    public synchronized AutoLinkState linkState() {
        return linkState;
    }

    public synchronized PlannerExecutionMode plannerMode() {
        return plannerMode;
    }

    public synchronized boolean fallbackActive() {
        return fallbackActive;
    }

    public synchronized boolean completed() {
        return completed;
    }

    public synchronized boolean aborted() {
        return aborted;
    }

    public synchronized String lastGateCode() {
        return lastGateCode;
    }

    public synchronized String lastGateDetail() {
        return lastGateDetail;
    }
}
