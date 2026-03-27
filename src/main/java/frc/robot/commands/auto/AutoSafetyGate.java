package frc.robot.commands.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Landmarks;

// Centralized autonomous legality/safety decisions.
public final class AutoSafetyGate {
    private static final double kFieldLengthMeters = 16.541;
    private static final double kCenterLineX = kFieldLengthMeters / 2.0;
    // Conservative margin keeps auto shots clearly on own-field side.
    private static final double kAllianceSideMarginMeters = 0.25;

    public enum Decision {
        ALLOW,
        DENY_RETRYABLE,
        DENY_FATAL_FALLBACK
    }

    public record ValidationResult(Decision decision, String code, String detail) {
        public static ValidationResult allow(String code, String detail) {
            return new ValidationResult(Decision.ALLOW, code, detail);
        }

        public static ValidationResult denyRetryable(String code, String detail) {
            return new ValidationResult(Decision.DENY_RETRYABLE, code, detail);
        }

        public static ValidationResult denyFatal(String code, String detail) {
            return new ValidationResult(Decision.DENY_FATAL_FALLBACK, code, detail);
        }
    }

    // Robot must be clearly on own half.
    public ValidationResult validateHubShot(Pose2d robotPose, Optional<Alliance> alliance) {
        if (alliance.isEmpty()) {
            return ValidationResult.denyRetryable(
                    "ALLIANCE_UNKNOWN",
                    "Cannot safely validate alliance-zone shot legality before DS alliance is known.");
        }

        final double x = robotPose.getX();
        final boolean onOwnSide = switch (alliance.get()) {
            case Blue -> x <= kCenterLineX - kAllianceSideMarginMeters;
            case Red -> x >= kCenterLineX + kAllianceSideMarginMeters;
        };

        if (!onOwnSide) {
            return ValidationResult.denyRetryable(
                    "HUB_SHOT_ZONE_DENY",
                    "Shot request denied because robot pose is not on the alliance-side half of the field.");
        }

        return ValidationResult.allow("HUB_SHOT_ZONE_ALLOW",
                "Shot request is within conservative alliance-side bounds.");
    }

    public ValidationResult validateAutoTraversalPose(Pose2d robotPose, Optional<Alliance> alliance) {
        if (alliance.isEmpty()) {
            return ValidationResult.denyRetryable(
                    "ALLIANCE_UNKNOWN_TRAVERSAL",
                    "Cannot validate center-line traversal constraints before alliance is known.");
        }

        final double x = robotPose.getX();
        final boolean crossedOpponentSide = switch (alliance.get()) {
            case Blue -> x > kCenterLineX + kAllianceSideMarginMeters;
            case Red -> x < kCenterLineX - kAllianceSideMarginMeters;
        };
        if (crossedOpponentSide) {
            return ValidationResult.denyFatal(
                    "CENTERLINE_INCUSION_DENY",
                    "AUTO path denied because robot pose is across center line into opponent side.");
        }
        return ValidationResult.allow("CENTERLINE_ALLOW", "AUTO traversal remains on alliance side of center line.");
    }

    public ValidationResult validateHubCollectionRisk(Pose2d robotPose) {
        final double distanceToHubMeters = robotPose.getTranslation().getDistance(Landmarks.hubPosition());
        if (distanceToHubMeters < 0.80) {
            return ValidationResult.denyRetryable(
                    "HUB_COLLECTION_RISK",
                    "Intake action denied because robot is too close to HUB center for safe anti-catch posture.");
        }
        return ValidationResult.allow("HUB_COLLECTION_ALLOW", "Robot is not in a high-risk HUB collection position.");
    }
}
