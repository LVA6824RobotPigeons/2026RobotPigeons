package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Landmarks;
import frc.robot.commands.auto.AutoSafetyGate.Decision;
import frc.robot.commands.auto.AutoSafetyGate.ValidationResult;
import org.junit.jupiter.api.Test;

class AutoSafetyGateTest {
    private final AutoSafetyGate gate = new AutoSafetyGate();

    @Test
    void validateHubShot_deniesWhenAllianceUnknown() {
        final ValidationResult result = gate.validateHubShot(
            new Pose2d(1.0, 1.0, Rotation2d.kZero),
            Optional.empty()
        );

        assertEquals(Decision.DENY_RETRYABLE, result.decision());
        assertEquals("ALLIANCE_UNKNOWN", result.code());
    }

    @Test
    void validateHubShot_allowsBlueWhenOnBlueHalf() {
        final ValidationResult result = gate.validateHubShot(
            new Pose2d(2.5, 1.0, Rotation2d.kZero),
            Optional.of(Alliance.Blue)
        );

        assertEquals(Decision.ALLOW, result.decision());
    }

    @Test
    void validateHubShot_deniesBlueWhenAcrossCenter() {
        final ValidationResult result = gate.validateHubShot(
            new Pose2d(9.5, 1.0, Rotation2d.kZero),
            Optional.of(Alliance.Blue)
        );

        assertEquals(Decision.DENY_RETRYABLE, result.decision());
        assertEquals("HUB_SHOT_ZONE_DENY", result.code());
    }


    @Test
    void validateAutoTraversalPose_deniesWhenOnOpponentSide() {
        final ValidationResult result = gate.validateAutoTraversalPose(
            new Pose2d(10.0, 1.0, Rotation2d.kZero),
            Optional.of(Alliance.Blue)
        );
        assertEquals(Decision.DENY_FATAL_FALLBACK, result.decision());
    }

    @Test
    void validateHubCollectionRisk_deniesWhenTooCloseToHubCenter() {
        final var hub = Landmarks.hubPosition();
        final ValidationResult result = gate.validateHubCollectionRisk(
            new Pose2d(hub.getX(), hub.getY(), Rotation2d.kZero)
        );
        assertEquals(Decision.DENY_RETRYABLE, result.decision());
    }
}
