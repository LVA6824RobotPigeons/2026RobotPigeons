package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Driving;
import frc.robot.Ports;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;
import frc.util.Stopwatch;

/**
 * Teleop manual drive command for the swerve drivetrain.
 *
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 */
public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING_WITH_MANUAL_ROTATION,
        DRIVING_WITH_LOCKED_HEADING
    }

    private static final Time kHeadingLockDelay = Seconds.of(0.25); // time to wait before locking heading

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Sets drive request type to Open Loop Voltage
        .withSteerRequestType(SteerRequestType.MotionMagicExpo) // Sets steer request type to Magic Magic Expo
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective); // Sets perspective to the operator's

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband) // Sets how much to smooth input
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate) // Sets maximum speed
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Sets drive request type to Open Loop Voltage
        .withSteerRequestType(SteerRequestType.MotionMagicExpo) // Sets steer request type to Motion Magic Expo
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective) // Sets perspective to the operator's
        .withHeadingPID(5, 0, 0);

    private State currentState = State.IDLING;
    private Optional<Rotation2d> lockedHeading = Optional.empty();
    private Stopwatch headingLockStopwatch = new Stopwatch();
    private ManualDriveInput previousInput = new ManualDriveInput();

    public ManualDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        addRequirements(swerve);
    }

    public void seedFieldCentric() {
        initialize(); // Initializes crap
        swerve.seedFieldCentric(); // Resets the rotation tracker, meaning if you set the position facing away from you upon initialization, it will be relative to you.
    }

    public void setLockedHeading(Rotation2d heading) {
        lockedHeading = Optional.of(heading);
        currentState = State.DRIVING_WITH_LOCKED_HEADING;
    }

    private void setLockedHeadingToCurrent() {
        final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation(); // Gets rotation based on the blue alliance
        final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection()); //  Adjusts rotation to match alliance
        setLockedHeading(headingInOperatorPerspective); // Sets "heading" to that
    }

    private void lockHeadingIfRotationStopped(ManualDriveInput input) {
        if (input.hasRotation()) { // If input is greater than 0
            headingLockStopwatch.reset(); // Resets heading stopwatch
            lockedHeading = Optional.empty(); // Locked heading reset to null
        } else {
            headingLockStopwatch.startIfNotRunning(); // Take a guess
            if (headingLockStopwatch.elapsedTime().gt(kHeadingLockDelay)) { // If elapsed time is greater than heading lock delay
                setLockedHeadingToCurrent(); // Update locked heading to our current
            }
        }
    }

    @Override
    /*
    * i moved the scs to here now.
    * we used to have it in execute, which is big nono
    * it would be recreated every single tick so wed always be number 0, while always having deallocations and reallocations
    * very bad.
    *
    * now we run it once, and keep the state tsable. yay :D
     */
    public void initialize() {
        currentState = State.IDLING;
        lockedHeading = Optional.empty();
        headingLockStopwatch.reset();
        previousInput = new ManualDriveInput();

        Ports.kCandle.setColorSequence(
            new RGBWColor[] {
                Constants.LEDs.kGreen,
                Constants.LEDs.kRed
            },
            400,
            1
        );
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput(); // Smooths input to be less jagged
        if (input.hasRotation()) {
            // If rotation is > 0
            currentState = State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (input.hasTranslation()) {
            // If x or y are greater than 0 but has no rotation
            currentState = lockedHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
            // If x or y JUST became 0 (No rotation, no translation)
            currentState = State.IDLING;
        }
        previousInput = input; // Updates input for next time it's called

        // State handling
        switch (currentState) {
            case IDLING:
                swerve.setControl(idleRequest);
                break;
            case DRIVING_WITH_MANUAL_ROTATION:
                lockHeadingIfRotationStopped(input);
                swerve.setControl(
                    fieldCentricRequest
                        .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                        .withVelocityY(Driving.kMaxSpeed.times(input.left))
                        .withRotationalRate(Driving.kMaxRotationalRate.times(input.rotation))
                );
                break;
            case DRIVING_WITH_LOCKED_HEADING:
                swerve.setControl(
                    fieldCentricFacingAngleRequest
                        .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                        .withVelocityY(Driving.kMaxSpeed.times(input.left))
                        .withTargetDirection(lockedHeading.get())
                );
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Runs until manually stopped
        return false;
    }
}
