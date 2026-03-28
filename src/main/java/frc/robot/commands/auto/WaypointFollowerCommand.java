package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

// Follows a sequence of coprocessor-specified waypoints
public class WaypointFollowerCommand extends Command {
    private static final double kDefaultMaxVelocityMps = 3.5;
    private static final double kDefaultMaxAccelerationMpsSq = 3.0;
    private static final double kPositionToleranceMeters = 0.15;
    private static final double kFinalPositionToleranceMeters = 0.10;
    private static final double kHeadingToleranceRadians = 0.15;

    // Waypoint w/ target pose and velocity constraint.
    public record WaypointTarget(
            Pose2d pose,
            double maxVelocityMps) {
        public WaypointTarget(Pose2d pose) {
            this(pose, kDefaultMaxVelocityMps);
        }

        // Creates a WaypointTarget from millimeter/milliradian coprocessor coordinates.
        public static WaypointTarget fromMm(int xMm, int yMm, int headingMrad, int maxVelocityMmS) {
            return new WaypointTarget(
                    new Pose2d(
                            xMm / 1000.0,
                            yMm / 1000.0,
                            new Rotation2d(headingMrad / 1000.0)),
                    maxVelocityMmS > 0 ? maxVelocityMmS / 1000.0 : kDefaultMaxVelocityMps);
        }
    }

    private final Swerve swerve;
    private final List<WaypointTarget> waypoints;
    private final SwerveRequest.ApplyFieldSpeeds fieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final PIDController xController = new PIDController(2.5, 0.0, 0.1);
    private final PIDController yController = new PIDController(2.5, 0.0, 0.1);
    private final PIDController thetaController = new PIDController(4.0, 0.0, 0.0);

    private int currentWaypointIndex = 0;
    private double segmentStartTime = 0.0;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State profileGoal;
    private TrapezoidProfile.State profileState;

    public WaypointFollowerCommand(Swerve swerve, List<WaypointTarget> waypoints) {
        this.swerve = swerve;
        this.waypoints = Collections.unmodifiableList(new ArrayList<>(waypoints));
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentWaypointIndex = 0;
        xController.reset();
        yController.reset();
        thetaController.reset();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        if (!waypoints.isEmpty()) {
            initializeSegment(0);
        }
    }

    @Override
    public void execute() {
        if (currentWaypointIndex >= waypoints.size()) {
            swerve.setControl(idleRequest);
            return;
        }

        final WaypointTarget target = waypoints.get(currentWaypointIndex);
        final Pose2d currentPose = swerve.getState().Pose;
        final Pose2d targetPose = target.pose();

        final double dt = Timer.getFPGATimestamp() - segmentStartTime;
        profileState = profile.calculate(dt, profileState, profileGoal);

        final double dx = targetPose.getX() - currentPose.getX();
        final double dy = targetPose.getY() - currentPose.getY();
        final double distanceToTarget = Math.hypot(dx, dy);

        double ux = 0, uy = 0;
        if (distanceToTarget > 0.01) {
            ux = dx / distanceToTarget;
            uy = dy / distanceToTarget;
        }

        // Feedforward velocity along line
        final double ffVelocity = profileState.velocity;

        // PID corrections
        final double pidVx = xController.calculate(currentPose.getX(), targetPose.getX());
        final double pidVy = yController.calculate(currentPose.getY(), targetPose.getY());

        // Combine ff & pid
        final double maxV = target.maxVelocityMps();
        final double vx = MathUtil.clamp(ux * ffVelocity + pidVx, -maxV, maxV);
        final double vy = MathUtil.clamp(uy * ffVelocity + pidVy, -maxV, maxV);

        // Heading tracking
        final double omega = MathUtil.clamp(
                thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians()),
                -6.0, 6.0);

        swerve.setControl(
                fieldSpeedsRequest.withSpeeds(new ChassisSpeeds(vx, vy, omega)));

        // Check if we've reached the current waypoint
        final boolean isFinalWaypoint = currentWaypointIndex == waypoints.size() - 1;
        final double tolerance = isFinalWaypoint ? kFinalPositionToleranceMeters : kPositionToleranceMeters;

        if (distanceToTarget <= tolerance) {
            currentWaypointIndex++;
            if (currentWaypointIndex < waypoints.size()) {
                initializeSegment(currentWaypointIndex);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(idleRequest);
    }

    @Override
    public boolean isFinished() {
        if (waypoints.isEmpty()) {
            return true;
        }
        if (currentWaypointIndex < waypoints.size()) {
            return false;
        }

        // All waypoints reached, verif we're settled at the final one
        final Pose2d currentPose = swerve.getState().Pose;
        final Pose2d finalTarget = waypoints.get(waypoints.size() - 1).pose();
        final double translationError = currentPose.getTranslation().getDistance(finalTarget.getTranslation());
        final double headingError = Math.abs(
                MathUtil.angleModulus(
                        currentPose.getRotation().getRadians() - finalTarget.getRotation().getRadians()));
        return translationError <= kFinalPositionToleranceMeters && headingError <= kHeadingToleranceRadians;
    }

    // Nromalized progress as a fraction [0, 1].
    public double getProgress() {
        if (waypoints.isEmpty())
            return 1.0;
        return (double) currentWaypointIndex / waypoints.size();
    }

    public int getCurrentWaypointIndex() {
        return currentWaypointIndex;
    }

    private void initializeSegment(int waypointIndex) {
        final Pose2d currentPose = swerve.getState().Pose;
        final WaypointTarget target = waypoints.get(waypointIndex);
        final double distance = currentPose.getTranslation().getDistance(target.pose().getTranslation());

        final double maxVel = target.maxVelocityMps() > 0 ? target.maxVelocityMps() : kDefaultMaxVelocityMps;

        // If this is the last waypoint, end at zero velocity. Otherwise cruise through.
        final boolean isFinal = waypointIndex == waypoints.size() - 1;
        final double endVelocity = isFinal ? 0.0 : maxVel * 0.6;

        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVel, kDefaultMaxAccelerationMpsSq));
        profileState = new TrapezoidProfile.State(0, 0);
        profileGoal = new TrapezoidProfile.State(distance, endVelocity);
        segmentStartTime = Timer.getFPGATimestamp();

        // Reset PID for new segment
        xController.reset();
        yController.reset();
    }
}
