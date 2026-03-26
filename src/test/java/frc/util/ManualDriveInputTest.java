package frc.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ManualDriveInputTest {
    private static final double EPS = 1e-12;

    @Test
    void defaultConstructor_initializesAllAxesToZero() {
        final ManualDriveInput input = new ManualDriveInput();

        assertEquals(0.0, input.forward, EPS);
        assertEquals(0.0, input.left, EPS);
        assertEquals(0.0, input.rotation, EPS);
    }

    @Test
    void hasTranslation_isTrueWhenEitherTranslationAxisIsNonZero() {
        assertTrue(new ManualDriveInput(0.2, 0.0, 0.0).hasTranslation());
        assertTrue(new ManualDriveInput(0.0, -0.2, 0.0).hasTranslation());
        assertFalse(new ManualDriveInput(0.0, 0.0, 0.5).hasTranslation());
    }

    @Test
    void hasRotation_isTrueWhenRotationIsNonZero() {
        assertTrue(new ManualDriveInput(0.0, 0.0, 1e-9).hasRotation());
        assertFalse(new ManualDriveInput(0.3, 0.3, 0.0).hasRotation());
    }
}
