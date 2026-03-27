package frc.robot.commands.auto;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// Executes autonomous phases w/ timeout-based fallback.
public final class AutoOrchestrator {
    private final AutoStatus status;

    public AutoOrchestrator(AutoStatus status) {
        this.status = status;
    }

    public Command runPhases(
            String routineId,
            AutoLinkState linkState,
            PlannerExecutionMode plannerMode,
            List<AutoPhase> phases) {
        return runPhases(
                routineId,
                linkState,
                plannerMode,
                phases,
                () -> false,
                "none",
                "none");
    }

    public Command runPhases(
            String routineId,
            AutoLinkState linkState,
            PlannerExecutionMode plannerMode,
            List<AutoPhase> phases,
            BooleanSupplier forcedFallbackCondition,
            String forcedFallbackCode,
            String forcedFallbackDetail) {
        final Command[] phaseCommands = phases.stream()
                // Once fallback path is activated, stop advancing phase graph for this run.
                .map(phase -> runPhase(
                        phase,
                        forcedFallbackCondition,
                        forcedFallbackCode,
                        forcedFallbackDetail).unless(status::fallbackActive))
                .toArray(Command[]::new);

        return Commands.sequence(
                Commands.runOnce(() -> status.startRoutine(routineId, linkState, plannerMode)),
                Commands.sequence(phaseCommands),
                Commands.runOnce(status::markCompleted)).finallyDo(interrupted -> {
                    if (interrupted) {
                        status.markAborted("Routine interrupted by scheduler.");
                    }
                });
    }

    private Command runPhase(
            AutoPhase phase,
            BooleanSupplier forcedFallbackCondition,
            String forcedFallbackCode,
            String forcedFallbackDetail) {
        return Commands.sequence(
                Commands.runOnce(() -> status.startPhase(phase.id())),
                phase.command()
                        .until(() -> phase.successCondition().getAsBoolean() || forcedFallbackCondition.getAsBoolean())
                        .withTimeout(phase.timeoutSeconds()),
                Commands.either(
                        Commands.none(),
                        Commands.sequence(
                                Commands.runOnce(() -> {
                                    status.markFallbackActive(phase.id());
                                    if (forcedFallbackCondition.getAsBoolean()) {
                                        status.markFallbackReason(forcedFallbackCode, forcedFallbackDetail);
                                    }
                                }),
                                phase.fallbackCommand()),
                        () -> phase.successCondition().getAsBoolean() && !forcedFallbackCondition.getAsBoolean()));
    }
}
