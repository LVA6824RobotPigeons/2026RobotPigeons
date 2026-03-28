package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DriveInputSmootherTest {
    private static final double EPS = 1e-9;

    @Test
    void deadband_zeroesSmallTranslationAndRotationInputs() {
        final DriveInputSmoother smoother = new DriveInputSmoother(
            () -> 0.10,
            () -> 0.10,
            () -> 0.149
        );

        final ManualDriveInput input = smoother.getSmoothedInput();

        assertEquals(0.0, input.forward, EPS);
        assertEquals(0.0, input.left, EPS);
        assertEquals(0.0, input.rotation, EPS);
    }

    @Test
    void fullScaleInputsRemainFullScaleAfterSmoothing() {
        final DriveInputSmoother smoother = new DriveInputSmoother(
            () -> 1.0,
            () -> 0.0,
            () -> -1.0
        );

        final ManualDriveInput input = smoother.getSmoothedInput();

        assertEquals(1.0, input.forward, EPS);
        assertEquals(0.0, input.left, EPS);
        assertEquals(-1.0, input.rotation, EPS);
    }

    @Test
    void diagonalTranslationKeepsItsDirectionAfterSmoothing() {
        final DriveInputSmoother smoother = new DriveInputSmoother(
            () -> 0.6,
            () -> 0.6,
            () -> 0.0
        );

        final ManualDriveInput input = smoother.getSmoothedInput();

        assertEquals(input.forward, input.left, EPS);
        assertTrue(input.forward > 0.0);
    }

    @Test
    void midRangeRotationIsCurvedForFinerControl() {
        final DriveInputSmoother smoother = new DriveInputSmoother(
            () -> 0.0,
            () -> 0.0,
            () -> 0.5
        );

        final ManualDriveInput input = smoother.getSmoothedInput();

        assertTrue(input.rotation > 0.0);
        assertTrue(input.rotation < 0.5);
    }
}
