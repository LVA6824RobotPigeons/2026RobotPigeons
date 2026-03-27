package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutoOrchestratorTest {
    private CommandScheduler scheduler;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void runPhases_marksCompletedWhenAllPhasesSucceed() {
        final AutoStatus status = new AutoStatus();
        final AutoOrchestrator orchestrator = new AutoOrchestrator(status);

        final AtomicBoolean phaseOneDone = new AtomicBoolean(false);
        final AtomicBoolean phaseTwoDone = new AtomicBoolean(false);

        final AutoPhase phaseOne = new CommandAutoPhase(
            "phase_one",
            0.5,
            Commands.runOnce(() -> phaseOneDone.set(true)),
            phaseOneDone::get,
            Commands.none()
        );
        final AutoPhase phaseTwo = new CommandAutoPhase(
            "phase_two",
            0.5,
            Commands.runOnce(() -> phaseTwoDone.set(true)),
            phaseTwoDone::get,
            Commands.none()
        );

        final Command autoCommand = orchestrator.runPhases(
            "unit_auto",
            AutoLinkState.LOCAL_ONLY,
            PlannerExecutionMode.LOCAL_ONLY,
            List.of(phaseOne, phaseTwo)
        );

        scheduler.schedule(autoCommand);
        for (int i = 0; i < 10; i++) {
            scheduler.run();
        }

        assertFalse(autoCommand.isScheduled());
        assertTrue(status.completed());
        assertFalse(status.fallbackActive());
        assertEquals("unit_auto", status.routineId());
    }

    @Test
    void runPhases_runsFallbackWhenPhaseDoesNotSucceed() {
        final AutoStatus status = new AutoStatus();
        final AutoOrchestrator orchestrator = new AutoOrchestrator(status);

        final AtomicBoolean fallbackRan = new AtomicBoolean(false);

        final AutoPhase phase = new CommandAutoPhase(
            "phase_with_fallback",
            0.5,
            Commands.none(),
            () -> false,
            Commands.runOnce(() -> fallbackRan.set(true))
        );

        final Command autoCommand = orchestrator.runPhases(
            "fallback_auto",
            AutoLinkState.LOCAL_ONLY,
            PlannerExecutionMode.LOCAL_ONLY,
            List.of(phase)
        );

        scheduler.schedule(autoCommand);
        for (int i = 0; i < 10; i++) {
            scheduler.run();
        }

        assertFalse(autoCommand.isScheduled());
        assertTrue(fallbackRan.get());
        assertTrue(status.fallbackActive());
        assertTrue(status.completed());
    }

    @Test
    void runPhases_forcedFallbackConditionTriggersImmediateFallback() {
        final AutoStatus status = new AutoStatus();
        final AutoOrchestrator orchestrator = new AutoOrchestrator(status);
        final AtomicBoolean fallbackRan = new AtomicBoolean(false);

        final AutoPhase phase = new CommandAutoPhase(
            "phase_force_fallback",
            2.0,
            Commands.waitSeconds(1.5),
            () -> false,
            Commands.runOnce(() -> fallbackRan.set(true))
        );

        final Command autoCommand = orchestrator.runPhases(
            "active_auto",
            AutoLinkState.HEALTHY,
            PlannerExecutionMode.ACTIVE,
            List.of(phase),
            () -> true,
            "PLANNER_LINK_FALLBACK",
            "Planner health lost during ACTIVE mode."
        );

        scheduler.schedule(autoCommand);
        for (int i = 0; i < 6; i++) {
            scheduler.run();
        }

        assertFalse(autoCommand.isScheduled());
        assertTrue(fallbackRan.get());
        assertTrue(status.fallbackActive());
        assertEquals("PLANNER_LINK_FALLBACK", status.lastGateCode());
    }
}
