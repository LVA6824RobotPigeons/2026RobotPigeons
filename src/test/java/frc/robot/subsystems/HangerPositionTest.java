package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HangerPositionTest {
    private static final double EPS = 1e-9;

    @Test
    void hangingPosition_convertsToExpectedMotorRotations() {
        final double rotations = Hanger.Position.HANGING.motorAngle().in(Rotations);
        assertEquals(142.0, rotations, EPS);
    }

    @Test
    void extensionPositions_areMonotonicInMotorRotations() {
        final double homed = Hanger.Position.HOMED.motorAngle().in(Rotations);
        final double extendHopper = Hanger.Position.EXTEND_HOPPER.motorAngle().in(Rotations);
        final double hanging = Hanger.Position.HANGING.motorAngle().in(Rotations);
        final double hung = Hanger.Position.HUNG.motorAngle().in(Rotations);

        assertEquals(0.0, homed, EPS);
        assertTrue(extendHopper > hung);
        assertTrue(hanging > extendHopper);
    }
}
