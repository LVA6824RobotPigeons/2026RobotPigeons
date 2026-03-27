package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.auto.bcnp.BcnpTcpPlannerClient;

public class Limelight extends SubsystemBase {
    private static final double kMaxTranslationInnovationMeters = 1.75;
    private static final double kMaxHeadingInnovationDegrees = 65.0;
    private static final double kMaxMeasurementAgeSeconds = 0.35;
    private static final double kMaxMegaTagHeadingDisagreementDegrees = 55.0;
    private static final double kMinAvgTagArea = 0.01;
    private static final double kMinMeasurementConfidence = 0.18;

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private RejectReason lastRejectReason = RejectReason.NONE;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Provide robot yaw hint so MT2 can fuse gyro heading for better translational stability.
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            publishRejectReason(RejectReason.NO_TAGS);
            return Optional.empty();
        }

        if (poseEstimate_MegaTag2.avgTagArea < kMinAvgTagArea) {
            publishRejectReason(RejectReason.LOW_TAG_AREA);
            return Optional.empty();
        }

        // Hybrid pose: keep MT2 translation, borrow MT1 heading.
        // Heading covariance remains intentionally loose below.
        final Pose2d fusedPose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final double measurementAgeSeconds = Timer.getFPGATimestamp() - poseEstimate_MegaTag2.timestampSeconds;
        if (measurementAgeSeconds > kMaxMeasurementAgeSeconds) {
            publishRejectReason(RejectReason.STALE_TIMESTAMP);
            return Optional.empty();
        }

        final double megaTagHeadingDisagreement = Math.abs(
            poseEstimate_MegaTag1.pose.getRotation().minus(poseEstimate_MegaTag2.pose.getRotation()).getDegrees()
        );
        if (megaTagHeadingDisagreement > kMaxMegaTagHeadingDisagreementDegrees) {
            publishRejectReason(RejectReason.HEADING_DISAGREEMENT);
            return Optional.empty();
        }

        if (isOutlier(currentRobotPose, fusedPose)) {
            publishRejectReason(RejectReason.OUTLIER);
            return Optional.empty();
        }

        poseEstimate_MegaTag2.pose = fusedPose;
        final double confidence = computeMeasurementConfidence(
            poseEstimate_MegaTag2,
            currentRobotPose,
            fusedPose,
            measurementAgeSeconds,
            megaTagHeadingDisagreement
        );
        if (confidence < kMinMeasurementConfidence) {
            publishRejectReason(RejectReason.LOW_CONFIDENCE);
            return Optional.empty();
        }

        final Matrix<N3, N1> standardDeviations = computeDynamicStdDevs(
            poseEstimate_MegaTag2,
            currentRobotPose,
            fusedPose,
            measurementAgeSeconds,
            confidence
        );

        posePublisher.set(fusedPose);
        telemetryTable.getEntry("Vision Confidence").setDouble(confidence);
        publishRejectReason(RejectReason.NONE);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations, confidence));
    }

    public List<BcnpTcpPlannerClient.OpponentTrack> getOpponents() {
        // TODO(Phase C): Fetch neural detector targets when YOLO model is deployed.
        // Current stub: return an empty list so tactical planner runs without obstacles.
        return List.of();
    }

    private boolean isOutlier(Pose2d currentRobotPose, Pose2d measuredPose) {
        final Translation2d poseDelta = measuredPose.getTranslation().minus(currentRobotPose.getTranslation());
        if (poseDelta.getNorm() > kMaxTranslationInnovationMeters) {
            return true;
        }

        final double headingErrorDegrees = measuredPose.getRotation()
            .minus(currentRobotPose.getRotation())
            .getDegrees();
        return Math.abs(headingErrorDegrees) > kMaxHeadingInnovationDegrees;
    }

    private Matrix<N3, N1> computeDynamicStdDevs(
        PoseEstimate estimate,
        Pose2d currentRobotPose,
        Pose2d measuredPose,
        double measurementAgeSeconds,
        double confidence
    ) {
        final double translationInnovation = measuredPose.getTranslation().getDistance(currentRobotPose.getTranslation());
        final double headingInnovationDeg = Math.abs(
            measuredPose.getRotation().minus(currentRobotPose.getRotation()).getDegrees()
        );

        final double baseTranslationStd = estimate.tagCount >= 2 ? 0.08 : 0.16;
        final double areaPenalty = estimate.avgTagArea < 0.05 ? 0.08 : 0.0;
        final double distancePenalty = Math.max(0.0, estimate.avgTagDist - 2.5) * 0.04;
        final double innovationPenalty = Math.min(0.18, translationInnovation * 0.10);
        final double agePenalty = Math.max(0.0, measurementAgeSeconds - 0.15) * 0.20;
        final double confidencePenalty = (1.0 - confidence) * 0.22;
        final double translationStd = baseTranslationStd + areaPenalty + distancePenalty + innovationPenalty + agePenalty + confidencePenalty;

        final double baseHeadingStdDeg = estimate.tagCount >= 2 ? 9.0 : 16.0;
        final double headingStdDeg = baseHeadingStdDeg + (headingInnovationDeg * 0.35) + ((1.0 - confidence) * 12.0);
        final double headingStdRad = Math.toRadians(Math.min(35.0, headingStdDeg));

        return VecBuilder.fill(translationStd, translationStd, headingStdRad);
    }

    private double computeMeasurementConfidence(
        PoseEstimate estimate,
        Pose2d currentRobotPose,
        Pose2d measuredPose,
        double measurementAgeSeconds,
        double megaTagHeadingDisagreementDegrees
    ) {
        final double translationInnovation = measuredPose.getTranslation().getDistance(currentRobotPose.getTranslation());
        final double headingInnovation = Math.abs(
            measuredPose.getRotation().minus(currentRobotPose.getRotation()).getDegrees()
        );

        final double tagScore = Math.min(1.0, estimate.tagCount / 3.0);
        final double areaScore = Math.min(1.0, estimate.avgTagArea / 0.20);
        final double ageScore = clamp01(1.0 - (measurementAgeSeconds / kMaxMeasurementAgeSeconds));
        final double translationScore = clamp01(1.0 - (translationInnovation / kMaxTranslationInnovationMeters));
        final double headingScore = clamp01(1.0 - (headingInnovation / kMaxHeadingInnovationDegrees));
        final double fusionScore = clamp01(1.0 - (megaTagHeadingDisagreementDegrees / kMaxMegaTagHeadingDisagreementDegrees));

        return (0.20 * tagScore)
            + (0.20 * areaScore)
            + (0.15 * ageScore)
            + (0.20 * translationScore)
            + (0.15 * headingScore)
            + (0.10 * fusionScore);
    }

    private double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private void publishRejectReason(RejectReason rejectReason) {
        lastRejectReason = rejectReason;
        telemetryTable.getEntry("Vision Reject Reason").setString(rejectReason.name());
    }

    public RejectReason lastRejectReason() {
        return lastRejectReason;
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;
        public final double confidence;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations, double confidence) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
            this.confidence = confidence;
        }
    }

    public enum RejectReason {
        NONE,
        NO_TAGS,
        LOW_TAG_AREA,
        STALE_TIMESTAMP,
        HEADING_DISAGREEMENT,
        LOW_CONFIDENCE,
        OUTLIER
    }
}
