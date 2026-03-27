package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mockStatic;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

class LimelightTest {
    private static final double EPS = 1e-9;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    void getMeasurement_fusesMegaTag2TranslationWithMegaTag1Rotation() {
        final Limelight limelight = new Limelight("limelight-unit-test");
        final Pose2d currentPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));

        final PoseEstimate mt1 = new PoseEstimate();
        mt1.pose = new Pose2d(8.0, 9.0, Rotation2d.fromDegrees(70.0));
        mt1.tagCount = 1;

        final PoseEstimate mt2 = new PoseEstimate();
        mt2.pose = new Pose2d(1.8, 2.7, Rotation2d.fromDegrees(25.0));
        mt2.tagCount = 2;
        mt2.avgTagArea = 0.2;
        mt2.avgTagDist = 1.2;
        mt2.timestampSeconds = Timer.getFPGATimestamp() + 1.0;

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);

            assertTrue(measurement.isPresent());
            assertEquals(1.8, measurement.get().poseEstimate.pose.getX(), EPS);
            assertEquals(2.7, measurement.get().poseEstimate.pose.getY(), EPS);
            assertEquals(70.0, measurement.get().poseEstimate.pose.getRotation().getDegrees(), EPS);
            assertTrue(measurement.get().confidence > 0.0);
            assertTrue(measurement.get().standardDeviations.get(0, 0) > 0.0);
            assertTrue(measurement.get().standardDeviations.get(1, 0) > 0.0);
            assertTrue(measurement.get().standardDeviations.get(2, 0) > 0.0);
            assertEquals(Limelight.RejectReason.NONE, limelight.lastRejectReason());
        }
    }

    @Test
    void getMeasurement_returnsEmptyWhenTagDataIsMissing() {
        final Limelight limelight = new Limelight("limelight-unit-test");
        final Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);

        final PoseEstimate mt1 = new PoseEstimate();
        mt1.tagCount = 0;

        final PoseEstimate mt2 = new PoseEstimate();
        mt2.tagCount = 1;
        mt2.avgTagArea = 0.2;
        mt2.timestampSeconds = Timer.getFPGATimestamp();

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);

            assertFalse(measurement.isPresent());
            assertEquals(Limelight.RejectReason.NO_TAGS, limelight.lastRejectReason());
        }
    }

    @Test
    void getMeasurement_rejectsOutlierPose() {
        final Limelight limelight = new Limelight("limelight-unit-test");
        final Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);

        final PoseEstimate mt1 = new PoseEstimate();
        mt1.pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(5.0));
        mt1.tagCount = 2;

        final PoseEstimate mt2 = new PoseEstimate();
        mt2.pose = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(0.0));
        mt2.tagCount = 2;
        mt2.avgTagArea = 0.2;
        mt2.timestampSeconds = Timer.getFPGATimestamp() + 1.0;

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);

            assertFalse(measurement.isPresent());
            assertEquals(Limelight.RejectReason.OUTLIER, limelight.lastRejectReason());
        }
    }

    @Test
    void getMeasurement_rejectsStaleTimestamp() {
        final Limelight limelight = new Limelight("limelight-unit-test");
        final Pose2d currentPose = new Pose2d(1.0, 1.0, Rotation2d.kZero);

        final PoseEstimate mt1 = new PoseEstimate();
        mt1.pose = new Pose2d(1.2, 1.1, Rotation2d.fromDegrees(12.0));
        mt1.tagCount = 2;

        final PoseEstimate mt2 = new PoseEstimate();
        mt2.pose = new Pose2d(1.1, 1.2, Rotation2d.fromDegrees(10.0));
        mt2.tagCount = 2;
        mt2.avgTagArea = 0.2;
        mt2.timestampSeconds = Timer.getFPGATimestamp() - 0.8;

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);
            assertFalse(measurement.isPresent());
            assertEquals(Limelight.RejectReason.STALE_TIMESTAMP, limelight.lastRejectReason());
        }
    }

    @Test
    void getMeasurement_rejectsWhenMegaTagHeadingDisagreesTooMuch() {
        final Limelight limelight = new Limelight("limelight-unit-test");
        final Pose2d currentPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(5.0));

        final PoseEstimate mt1 = new PoseEstimate();
        mt1.pose = new Pose2d(2.1, 2.0, Rotation2d.fromDegrees(170.0));
        mt1.tagCount = 2;

        final PoseEstimate mt2 = new PoseEstimate();
        mt2.pose = new Pose2d(2.0, 2.1, Rotation2d.fromDegrees(5.0));
        mt2.tagCount = 2;
        mt2.avgTagArea = 0.3;
        mt2.timestampSeconds = Timer.getFPGATimestamp() + 1.0;

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);
            assertFalse(measurement.isPresent());
            assertEquals(Limelight.RejectReason.HEADING_DISAGREEMENT, limelight.lastRejectReason());
        }
    }
}
