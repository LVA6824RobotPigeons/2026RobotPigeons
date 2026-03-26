package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mockStatic;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        mt2.pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(5.0));
        mt2.tagCount = 2;
        mt2.timestampSeconds = 12.34;

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);

            assertTrue(measurement.isPresent());
            assertEquals(3.0, measurement.get().poseEstimate.pose.getX(), EPS);
            assertEquals(4.0, measurement.get().poseEstimate.pose.getY(), EPS);
            assertEquals(70.0, measurement.get().poseEstimate.pose.getRotation().getDegrees(), EPS);
            assertEquals(0.1, measurement.get().standardDeviations.get(0, 0), EPS);
            assertEquals(0.1, measurement.get().standardDeviations.get(1, 0), EPS);
            assertEquals(10.0, measurement.get().standardDeviations.get(2, 0), EPS);
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

        try (MockedStatic<LimelightHelpers> helpers = mockStatic(LimelightHelpers.class)) {
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-unit-test")).thenReturn(mt1);
            helpers.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-unit-test")).thenReturn(mt2);

            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentPose);

            assertFalse(measurement.isPresent());
        }
    }
}
