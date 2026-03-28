package frc.robot.commands.auto;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.commands.auto.bcnp.BcnpAutoProtocol;
import frc.robot.commands.auto.bcnp.BcnpTcpPlannerClient;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// End-to-end one-button autonomous cycle command.

// When driver holds one button, the robot autonomously executes a full
// scoring cycle: deploy intake → drive to coprocessor-selected fuel zone →
// acquire fuel → transit to hub → aim → fire. Driver releases to cancel.

// In auto mode, the coprocessor provides all waypoints. In teleop,
// the robot takes full translational and rotational control while the button
// is held.

// State machine:
// SEEKING → Drive to fuel zone, intake deployed
// COLLECTING → At fuel zone, waiting for current spike (fuel acquired)
// TRANSITING → Driving toward hub, pre-spinning shooter
// SCORING → Aligned and ready, executing feed sequence
// DONE → Feed complete, returning to neutral

public class FullCycleCommand extends Command {
    private static final double kCollectTimeoutSeconds = 4.0;
    private static final double kScoringTimeoutSeconds = 5.0;
    private static final double kFeedDurationSeconds = 1.5;
    private static final long kShotHintFreshnessMs = 500;
    private static final double kArrivalToleranceMeters = 0.30;

    public enum CycleState {
        SEEKING,
        COLLECTING,
        TRANSITING,
        SCORING,
        DONE
    }

    private final Swerve swerve;
    private final Intake intake;
    private final Shooter shooter;
    private final Hood hood;
    private final Feeder feeder;
    private final Floor floor;
    private final FuelDetector fuelDetector;
    private final BcnpTcpPlannerClient plannerClient;
    private final Supplier<Pose2d> poseSupplier;

    private final WaypointFollowerCommand.WaypointTarget[] emptyWaypoints = new WaypointFollowerCommand.WaypointTarget[0];
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private CycleState state = CycleState.SEEKING;
    private WaypointFollowerCommand currentFollower;
    private double stateStartTime = 0;
    private int phaseSeqForCollection = 1;
    private int phaseSeqForTransit = 3;

    public FullCycleCommand(
            Swerve swerve,
            Intake intake,
            Shooter shooter,
            Hood hood,
            Feeder feeder,
            Floor floor,
            FuelDetector fuelDetector,
            BcnpTcpPlannerClient plannerClient,
            Supplier<Pose2d> poseSupplier) {
        this.swerve = swerve;
        this.intake = intake;
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.floor = floor;
        this.fuelDetector = fuelDetector;
        this.plannerClient = plannerClient;
        this.poseSupplier = poseSupplier;
        addRequirements(swerve, intake, shooter, hood, feeder, floor);
    }

    @Override
    public void initialize() {
        state = CycleState.SEEKING;
        stateStartTime = Timer.getFPGATimestamp();

        // Deploy intake immediately
        intake.set(Intake.Position.INTAKE);
        intake.set(Intake.Speed.INTAKE);

        // Build waypoint follower from coprocessor phase 1 (drive to fuel zone)
        currentFollower = buildFollowerForPhase(phaseSeqForCollection);
        if (currentFollower != null) {
            currentFollower.initialize();
        }
    }

    @Override
    public void execute() {
        SmartDashboard.putString("FullCycle/State", state.name());

        final boolean justAcquired = fuelDetector.consumeAcquisitionEvent();

        switch (state) {
            case SEEKING -> executeSeeking(justAcquired);
            case COLLECTING -> executeCollecting(justAcquired);
            case TRANSITING -> executeTransiting();
            case SCORING -> executeScoring();
            case DONE -> {
            } // Do nothing, isFinished will return true
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Safe shutdown: stop all mechanisms
        shooter.stop();
        intake.set(Intake.Speed.STOP);
        intake.set(Intake.Position.STOWED);
        swerve.setControl(idleRequest);

        if (currentFollower != null) {
            currentFollower.end(true);
            currentFollower = null;
        }
    }

    @Override
    public boolean isFinished() {
        return state == CycleState.DONE;
    }

    public CycleState getCycleState() {
        return state;
    }

    // State guys

    private void executeSeeking(boolean justAcquired) {
        // Drive toward fuel zone w/ coprocessor waypoints
        if (currentFollower != null) {
            currentFollower.execute();

            // If we arrived at fuel zone or acquired fuel en route
            if (currentFollower.isFinished() || justAcquired) {
                currentFollower.end(false);
                transitionTo(justAcquired ? CycleState.TRANSITING : CycleState.COLLECTING);
                return;
            }
        } else {
            // No waypoints available,stay in seeking until coprocessor provides them
            // or fuel is opportunistically acquired :(
            if (justAcquired) {
                transitionTo(CycleState.TRANSITING);
            }
        }
    }

    private void executeCollecting(boolean justAcquired) {
        // Waiting at fuel zone for intake to grab fuel
        if (justAcquired) {
            transitionTo(CycleState.TRANSITING);
            return;
        }

        // Timeout — move on even without fuel
        if (Timer.getFPGATimestamp() - stateStartTime > kCollectTimeoutSeconds) {
            transitionTo(CycleState.TRANSITING);
        }
    }

    private void executeTransiting() {
        // Drive toward hub while pre-spinning shooter
        if (currentFollower != null) {
            currentFollower.execute();
        }

        // Pre-spin shooter during transit
        final Optional<BcnpAutoProtocol.AutoShotHintPayload> hint = plannerClient.latestShotHint(kShotHintFreshnessMs);
        if (hint.isPresent()) {
            shooter.setRPM(hint.get().shooterRpm());
            hood.setPosition(hint.get().hoodPositionPermille() / 1000.0);
        } else {
            // Fallback: pre-spin at a reasonable default
            shooter.setRPM(3500);
        }

        // Check if we've arrived at scoring position
        final boolean followerDone = currentFollower == null || currentFollower.isFinished();
        final boolean closeToHub = poseSupplier.get().getTranslation()
                .getDistance(Landmarks.hubPosition()) < 3.0; // W/in 3m of hub

        if (followerDone || closeToHub) {
            if (currentFollower != null) {
                currentFollower.end(false);
            }
            transitionTo(CycleState.SCORING);
        }
    }

    private void executeScoring() {
        // Aim at hub and fire when ready
        final Pose2d currentPose = poseSupplier.get();
        final Translation2d hubPos = Landmarks.hubPosition();
        final Rotation2d toHub = hubPos.minus(currentPose.getTranslation()).getAngle();

        // Use coprocessor shot hints if available
        final Optional<BcnpAutoProtocol.AutoShotHintPayload> hint = plannerClient.latestShotHint(kShotHintFreshnessMs);
        if (hint.isPresent()) {
            shooter.setRPM(hint.get().shooterRpm());
            hood.setPosition(hint.get().hoodPositionPermille() / 1000.0);
        }

        // Check alignment and shooter readiness
        final double headingError = Math.abs(
                currentPose.getRotation().getRadians() - toHub.getRadians());
        final boolean aligned = headingError < 0.15; // ~8.6 degrees
        final boolean shooterReady = shooter.isVelocityWithinTolerance();

        if (aligned && shooterReady) {
            // Fire! :DDDDDDDDD
            feeder.set(Feeder.Speed.FEED);
            floor.set(Floor.Speed.FEED);
            intake.set(Intake.Speed.INTAKE);

            // Send world update to coprocessor
            if (plannerClient.isHealthy()) {
                plannerClient.sendWorldUpdate(
                        Math.max(0, fuelDetector.getFuelCount() - 1), // Approximate fuel remaining
                        true,
                        0, 0,
                        currentPose,
                        0, 0);
            }

            // Wait for feed duration then done
            if (Timer.getFPGATimestamp() - stateStartTime > kFeedDurationSeconds) {
                transitionTo(CycleState.DONE);
            }
        }

        // Timeout safety
        if (Timer.getFPGATimestamp() - stateStartTime > kScoringTimeoutSeconds) {
            transitionTo(CycleState.DONE);
        }
    }

    // Helpers

    private void transitionTo(CycleState newState) {
        state = newState;
        stateStartTime = Timer.getFPGATimestamp();

        switch (newState) {
            case TRANSITING -> {
                // Stow intake, build transit waypoints
                intake.set(Intake.Position.STOWED);
                intake.set(Intake.Speed.STOP);
                currentFollower = buildFollowerForPhase(phaseSeqForTransit);
                if (currentFollower != null) {
                    currentFollower.initialize();
                }
            }
            case SCORING -> {
                currentFollower = null;
            }
            case DONE -> {
                shooter.stop();
                intake.set(Intake.Speed.STOP);
                intake.set(Intake.Position.STOWED);
            }
            default -> {
            }
        }
    }

    private WaypointFollowerCommand buildFollowerForPhase(int phaseSeq) {
        final List<BcnpAutoProtocol.AutoWaypointDeltaPayload> wpPayloads = plannerClient.waypointsForPhase(phaseSeq);

        if (wpPayloads.isEmpty()) {
            return null;
        }

        final List<WaypointFollowerCommand.WaypointTarget> targets = wpPayloads.stream()
                .map(wp -> WaypointFollowerCommand.WaypointTarget.fromMm(
                        wp.xMm(), wp.yMm(), wp.headingMrad(), wp.maxVelocityMmS()))
                .toList();

        return new WaypointFollowerCommand(swerve, targets);
    }
}
