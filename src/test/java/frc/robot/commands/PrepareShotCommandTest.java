package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Landmarks;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

class PrepareShotCommandTest {
    private static final double EPS = 1e-6;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void forceBlueAllianceReferenceFrame() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
    }

    @Test
    void execute_usesCalibrationValuesAtKnownDistance() {
        final Shooter shooter = mock(Shooter.class);
        final Hood hood = mock(Hood.class);
        final PrepareShotCommand command = new PrepareShotCommand(
            shooter,
            hood,
            () -> poseAtDistanceFromHubInches(52.0)
        );

        command.execute();

        final ArgumentCaptor<Double> rpmCaptor = ArgumentCaptor.forClass(Double.class);
        final ArgumentCaptor<Double> hoodCaptor = ArgumentCaptor.forClass(Double.class);
        verify(shooter, times(1)).setRPM(rpmCaptor.capture());
        verify(hood, times(1)).setPosition(hoodCaptor.capture());
        assertTrue(Math.abs(rpmCaptor.getValue() - 2800.0) < EPS);
        assertTrue(Math.abs(hoodCaptor.getValue() - 0.19) < EPS);
    }

    @Test
    void execute_interpolatesLinearlyBetweenCalibrationPoints() {
        final Shooter shooter = mock(Shooter.class);
        final Hood hood = mock(Hood.class);
        final double midpointInches = (52.0 + 114.4) / 2.0;
        final PrepareShotCommand command = new PrepareShotCommand(
            shooter,
            hood,
            () -> poseAtDistanceFromHubInches(midpointInches)
        );

        command.execute();

        final ArgumentCaptor<Double> rpmCaptor = ArgumentCaptor.forClass(Double.class);
        final ArgumentCaptor<Double> hoodCaptor = ArgumentCaptor.forClass(Double.class);
        verify(shooter).setRPM(rpmCaptor.capture());
        verify(hood).setPosition(hoodCaptor.capture());

        assertTrue(Math.abs(rpmCaptor.getValue() - 3037.5) < EPS);
        assertTrue(Math.abs(hoodCaptor.getValue() - 0.295) < EPS);
    }

    @Test
    void isReadyToShoot_requiresBothShooterAndHoodReady() {
        final Shooter shooter = mock(Shooter.class);
        final Hood hood = mock(Hood.class);
        final PrepareShotCommand command = new PrepareShotCommand(
            shooter,
            hood,
            () -> poseAtDistanceFromHubInches(52.0)
        );

        when(shooter.isVelocityWithinTolerance()).thenReturn(true);
        when(hood.isPositionWithinTolerance()).thenReturn(false);
        assertFalse(command.isReadyToShoot());

        when(hood.isPositionWithinTolerance()).thenReturn(true);
        assertTrue(command.isReadyToShoot());
    }

    @Test
    void end_alwaysStopsShooter() {
        final Shooter shooter = mock(Shooter.class);
        final Hood hood = mock(Hood.class);
        final PrepareShotCommand command = new PrepareShotCommand(
            shooter,
            hood,
            () -> poseAtDistanceFromHubInches(52.0)
        );

        command.end(true);

        verify(shooter, times(1)).stop();
    }

    private static Pose2d poseAtDistanceFromHubInches(double distanceInches) {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final double x = hubPosition.getMeasureX().in(Inches) + distanceInches;
        final double y = hubPosition.getMeasureY().in(Inches);
        return new Pose2d(Inches.of(x), Inches.of(y), Rotation2d.kZero);
    }
}
