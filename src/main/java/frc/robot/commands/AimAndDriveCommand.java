package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.Ports;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband) // Sets value for smoothing rotational change
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate) // Sets maximum rotation speed
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Sets the allowed request type to Open Loop Voltage
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)  // "Control the drive motor using a Motion Magic® Expo request." - Docs
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective) // Sets the perspective of the operator
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        Swerve swerve, // Swerve details and stuff
        DoubleSupplier yInput, // Swerve Joystick Y axis
        DoubleSupplier xInput // Swerve Joystick X axis
    ) {
        this.swerve = swerve; // Sets swerve stuff
        this.inputSmoother = new DriveInputSmoother(yInput, xInput); // Makes the input for smooth
        addRequirements(swerve); // Makes sure swerve doesn't execute alongside anything else
    }

    /*public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }*/

    /**
     * @return Returns if robot is pointing close enough to the hub to shoot.
     * (Based on {@link #kAimTolerance frc.robot.commands.AimAndDriveCommand.kAimTolerance})
     */
    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation(); // Gets absolute rotation (Based on the default reference point, which is blue alliance)
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection()); // Converts rotation to operator's perspective
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance); // Only returns true if the direction is within a certain tolerance from the target rotation
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition(); // Gets hub based on team (meters)
        final Translation2d robotPosition = swerve.getState().Pose.getTranslation(); // Gets robot position (meters)
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle(); // Gets absolute rotation (Based on the default reference point, which is blue alliance)
        return hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection()); // Converts the angle to the operator's pov
    }

    @Override
    public void initialize() {
        // Sets color sequence once when the command starts
        Ports.kCandle.setColor(
                Constants.LEDs.kYellow,
                2
        );
    }

    @Override
    public void end(boolean interrupted) {
        Ports.kCandle.removeColor(2);
    }

    @Override
    public void execute() {
        // Sets up kcandle as port
        final ManualDriveInput input = inputSmoother.getSmoothedInput(); // Smooths out input to make it more spherical
        // Sets up swerve to always face the hub, while still driving as normal
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward)) // Input maxed out
                .withVelocityY(Driving.kMaxSpeed.times(input.left)) // Input maxed out
                .withTargetDirection(getDirectionToHub()) // Gets direction to hub based on operator's view
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Makes the command never end until switched
    }
}
