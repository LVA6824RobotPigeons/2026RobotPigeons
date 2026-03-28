package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

// Field-centric pose breakoff command
public class DriveToPoseCommand extends Command {
    private static final double kXyToleranceMeters = 0.14;
    private static final double kHeadingToleranceRadians = 0.12;
    private static final double kMaxLinearVelocityMetersPerSecond = 2.2;
    private static final double kMaxAngularVelocityRadiansPerSecond = RotationsPerSecond.of(0.8).in(RotationsPerSecond)
            * 2.0 * Math.PI;

    private final Swerve swerve;
    private final Supplier<Pose2d> targetSupplier;
    private final SwerveRequest.ApplyFieldSpeeds request = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private final PIDController xController = new PIDController(1.8, 0.0, 0.0);
    private final PIDController yController = new PIDController(1.8, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(3.2, 0.0, 0.0);

    private Pose2d currentTarget = Pose2d.kZero;

    public DriveToPoseCommand(Swerve swerve, Supplier<Pose2d> targetSupplier) {
        this.swerve = swerve;
        this.targetSupplier = targetSupplier;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentTarget = targetSupplier.get();
        xController.reset();
        yController.reset();
        thetaController.reset();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        currentTarget = targetSupplier.get();
        final Pose2d currentPose = swerve.getState().Pose;

        final double vx = MathUtil.clamp(
                xController.calculate(currentPose.getX(), currentTarget.getX()),
                -kMaxLinearVelocityMetersPerSecond,
                kMaxLinearVelocityMetersPerSecond);
        final double vy = MathUtil.clamp(
                yController.calculate(currentPose.getY(), currentTarget.getY()),
                -kMaxLinearVelocityMetersPerSecond,
                kMaxLinearVelocityMetersPerSecond);
        final double omega = MathUtil.clamp(
                thetaController.calculate(currentPose.getRotation().getRadians(),
                        currentTarget.getRotation().getRadians()),
                -kMaxAngularVelocityRadiansPerSecond,
                kMaxAngularVelocityRadiansPerSecond);

        swerve.setControl(
                request.withSpeeds(
                        new ChassisSpeeds(
                                MetersPerSecond.of(vx).in(MetersPerSecond),
                                MetersPerSecond.of(vy).in(MetersPerSecond),
                                omega)));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(idleRequest);
    }

    @Override
    public boolean isFinished() {
        final Pose2d currentPose = swerve.getState().Pose;
        final double translationError = currentPose.getTranslation().getDistance(currentTarget.getTranslation());
        final double headingError = Math.abs(
                MathUtil.angleModulus(
                        currentPose.getRotation().getRadians() - currentTarget.getRotation().getRadians()));
        return translationError <= kXyToleranceMeters && headingError <= kHeadingToleranceRadians;
    }
}
