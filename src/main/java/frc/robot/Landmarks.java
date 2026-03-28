package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    /**
     * @return Hub location in the WPILib blue-origin field frame.
     *     When alliance is unknown (e.g., before DS handshake), this defaults to red-side values.
     */
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            // 2026 field coordinates for the blue alliance target center.
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        // Red alliance uses mirrored X coordinate.
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }
}
