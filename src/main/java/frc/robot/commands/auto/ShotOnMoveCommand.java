package frc.robot.commands.auto;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.commands.auto.bcnp.BcnpAutoProtocol;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// Maintains translational velocity to a target pose while the heading
// tracks a aim lead angle. When heading enters the fire window, automatically
// triggers the feed sequence.

public class ShotOnMoveCommand extends Command {
    private static final double kFeedDurationSeconds = 1.5;

    private final Swerve swerve;
    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final Floor floor;
    private final Intake intake;
    private final Supplier<Optional<BcnpAutoProtocol.AutoShotHintPayload>> shotHintSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;

    private final PIDController xController = new PIDController(2.0, 0, 0.05);
    private final PIDController yController = new PIDController(2.0, 0, 0.05);
    private final SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(5, 0, 0);
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private boolean hasFired = false;
    private double feedStartTime = -1;

    public ShotOnMoveCommand(
            Swerve swerve,
            Shooter shooter,
            Hood hood,
            Feeder feeder,
            Floor floor,
            Intake intake,
            Supplier<Optional<BcnpAutoProtocol.AutoShotHintPayload>> shotHintSupplier,
            Supplier<Pose2d> targetPoseSupplier) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.floor = floor;
        this.intake = intake;
        this.shotHintSupplier = shotHintSupplier;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(swerve, shooter, hood, feeder, floor, intake);
    }

    @Override
    public void initialize() {
        hasFired = false;
        feedStartTime = -1;
        xController.reset();
        yController.reset();
    }

    @Override
    public void execute() {
        final Pose2d currentPose = swerve.getState().Pose;
        final Pose2d targetPose = targetPoseSupplier.get();
        final Optional<BcnpAutoProtocol.AutoShotHintPayload> hintOpt = shotHintSupplier.get();

        if (hintOpt.isEmpty()) {
            // No shot solution
            driveTowardTarget(currentPose, targetPose, getDirectionToHub(currentPose));
            return;
        }

        final BcnpAutoProtocol.AutoShotHintPayload hint = hintOpt.get();

        // Pre-spin shooter and set hood to coprocessor-specified values
        shooter.setRPM(hint.shooterRpm());
        hood.setPosition(hint.hoodPositionPermille() / 1000.0);

        final Rotation2d hubDirection = getDirectionToHub(currentPose);
        final Rotation2d aimDirection = hubDirection.plus(new Rotation2d(hint.aimOffsetMrad() / 1000.0));

        driveTowardTarget(currentPose, targetPose, aimDirection);

        // Check if we're in the fire window
        final double headingError = Math.abs(
                MathUtil.angleModulus(
                        currentPose.getRotation().getRadians() - aimDirection.getRadians()));
        final double fireWindowRad = hint.fireWindowMrad() / 1000.0;

        if (headingError <= fireWindowRad && shooter.isVelocityWithinTolerance() && !hasFired) {
            // FEAST MODE ACTIVATED.
            // #HUNGRY
            // FEEDMEMORE.COM
            triggerFeed();
            hasFired = true;
            feedStartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        swerve.setControl(idleRequest);
    }

    @Override
    public boolean isFinished() {
        if (!hasFired)
            return false;
        // Wait for feed to complete
        return feedStartTime > 0 && Timer.getFPGATimestamp() - feedStartTime > kFeedDurationSeconds;
    }

    public boolean hasFired() {
        return hasFired;
    }

    private void driveTowardTarget(Pose2d currentPose, Pose2d targetPose, Rotation2d aimDirection) {
        final double vx = MathUtil.clamp(
                xController.calculate(currentPose.getX(), targetPose.getX()),
                -Driving.kMaxSpeed.magnitude(), Driving.kMaxSpeed.magnitude());
        final double vy = MathUtil.clamp(
                yController.calculate(currentPose.getY(), targetPose.getY()),
                -Driving.kMaxSpeed.magnitude(), Driving.kMaxSpeed.magnitude());

        swerve.setControl(
                aimRequest
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(aimDirection));
    }

    private Rotation2d getDirectionToHub(Pose2d robotPose) {
        final Translation2d hubPosition = Landmarks.hubPosition();
        return hubPosition.minus(robotPose.getTranslation()).getAngle();
    }

    private void triggerFeed() {
        feeder.set(Feeder.Speed.FEED);
        floor.set(Floor.Speed.FEED);
        intake.set(Intake.Speed.INTAKE);
    }
}
