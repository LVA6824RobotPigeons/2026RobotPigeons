package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.Ports;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class PrepareShotCommand extends Command {
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                // Gets t percent of the way from start value to end value (T = 0-1, as 0% - 100%)
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );

    static {
        // Maps out 3 different points and how far and fast you need to shoot and then interpolates everything in between
        distanceToShotMap.put(Inches.of(52.0), new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }

    private final Shooter shooter;
    private final Hood hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Sets up data and requirments
    public PrepareShotCommand(Shooter shooter, Hood hood, Supplier<Pose2d> robotPoseSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(shooter, hood); // Requires shooter and hoot for command I think
    }

    public boolean isReadyToShoot() {
        //  Returns true if the wheels are going fast enough, and if the position is correct enough
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance(); // "gives if in tol. returns dalse if not" - Mike
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition)); // Gets distance to hub in meters
    }

    @Override
    public void initialize() {

        

    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final Shot shot = distanceToShotMap.get(distanceToHub);
        shooter.setRPM(shot.shooterRPM);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches)); // Adds to smart dashboard
    }

    @Override
    public boolean isFinished() {
        // Runs infinitely until stopped
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); // in the name of law!
    }

    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition; //basic shot data for pos and shooting
        }
    }
}
