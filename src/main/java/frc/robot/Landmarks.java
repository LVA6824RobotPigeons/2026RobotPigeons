package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    /**
     * @return Returns hub position (meters) based on team. (Defaults to red if team unknown)
     */
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance(); // Gets team as an Alliance object that could be null
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            // If alliance is blue
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        // If alliance is null or red
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }
}
