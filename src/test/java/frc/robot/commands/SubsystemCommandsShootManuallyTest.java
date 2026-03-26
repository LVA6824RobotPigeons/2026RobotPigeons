package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SubsystemCommandsShootManuallyTest {
    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void enableDriverStation() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void cleanupScheduler() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void shootManually_buildsPipelineFromSpinUpAndFeedSubsystemCommands() {
        final Swerve swerve = mock(Swerve.class);
        final Intake intake = mock(Intake.class);
        final Floor floor = mock(Floor.class);
        final Feeder feeder = mock(Feeder.class);
        final Shooter shooter = mock(Shooter.class);
        final Hood hood = mock(Hood.class);
        final Hanger hanger = mock(Hanger.class);

        when(shooter.dashboardSpinUpCommand()).thenReturn(Commands.none());
        when(feeder.feedCommand()).thenReturn(Commands.none());
        when(floor.feedCommand()).thenReturn(Commands.none());
        when(intake.agitateCommand()).thenReturn(Commands.none());

        final SubsystemCommands subsystemCommands = new SubsystemCommands(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger
        );

        subsystemCommands.shootManually();

        verify(shooter, times(1)).dashboardSpinUpCommand();
        verify(feeder, times(1)).feedCommand();
        verify(floor, times(1)).feedCommand();
        verify(intake, times(1)).agitateCommand();
    }

    @Test
    void shootManually_stopsShooterWhenInterrupted() {
        final Swerve swerve = mock(Swerve.class);
        final Intake intake = mock(Intake.class);
        final Floor floor = mock(Floor.class);
        final Feeder feeder = mock(Feeder.class);
        final Shooter shooter = mock(Shooter.class);
        final Hood hood = mock(Hood.class);
        final Hanger hanger = mock(Hanger.class);

        // Keep the command running so we can force an interrupt and assert interrupt handling.
        when(shooter.dashboardSpinUpCommand()).thenReturn(Commands.waitUntil(() -> false));
        when(feeder.feedCommand()).thenReturn(Commands.none());
        when(floor.feedCommand()).thenReturn(Commands.none());
        when(intake.agitateCommand()).thenReturn(Commands.none());

        final SubsystemCommands subsystemCommands = new SubsystemCommands(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger
        );

        final Command commandUnderTest = subsystemCommands.shootManually();
        CommandScheduler.getInstance().schedule(commandUnderTest);
        CommandScheduler.getInstance().run();
        assertTrue(commandUnderTest.isScheduled(), "Precondition: command must be active before cancellation");

        CommandScheduler.getInstance().cancel(commandUnderTest);
        CommandScheduler.getInstance().run();

        verify(shooter, times(1)).stop();
    }
}
