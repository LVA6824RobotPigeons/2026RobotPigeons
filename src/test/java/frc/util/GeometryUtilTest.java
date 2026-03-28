package frc.util;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class GeometryUtilTest {
    @Test
    void isNear_handlesWrapAroundAcrossPlusMinusPi() {
        final Rotation2d expected = Rotation2d.fromDegrees(179.0);
        final Rotation2d actual = Rotation2d.fromDegrees(-179.0);

        assertTrue(GeometryUtil.isNear(expected, actual, Degrees.of(3.0)));
    }

    @Test
    void isNear_rejectsAnglesOutsideToleranceAcrossWrapAround() {
        final Rotation2d expected = Rotation2d.fromDegrees(179.0);
        final Rotation2d actual = Rotation2d.fromDegrees(-170.0);

        assertFalse(GeometryUtil.isNear(expected, actual, Degrees.of(5.0)));
    }
}
