package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class LandmarksTest {
    private static final double EPS = 1e-9;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    void hubPosition_usesBlueCoordinatesWhenAllianceIsBlue() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();

        final Translation2d hub = Landmarks.hubPosition();

        assertEquals(182.105, hub.getMeasureX().in(Inches), EPS);
        assertEquals(158.845, hub.getMeasureY().in(Inches), EPS);
    }

    @Test
    void hubPosition_usesRedCoordinatesWhenAllianceIsRed() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
        DriverStationSim.notifyNewData();

        final Translation2d hub = Landmarks.hubPosition();

        assertEquals(469.115, hub.getMeasureX().in(Inches), EPS);
        assertEquals(158.845, hub.getMeasureY().in(Inches), EPS);
    }
}
