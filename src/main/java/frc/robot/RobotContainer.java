// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.commands.auto.bcnp.BcnpTcpPlannerClient;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Treat trigger pulls beyond this value as an intentional button press.
    private static final double kTriggerThreshold = 0.5;

    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    public CommandXboxController driver = new CommandXboxController(Ports.driver);

    private Trigger driverTrigger(BooleanSupplier isActive) {
        return new Trigger(isActive);
    }

    private Trigger driverRightTrigger() {
        return driverTrigger(() -> driver.getRightTriggerAxis() > kTriggerThreshold);
    }

    private Trigger driverLeftTrigger() {
        return driverTrigger(() -> driver.getLeftTriggerAxis() > kTriggerThreshold);
    }

    private Trigger driverRightBumper() {
        return driverTrigger(() -> driver.getHID().getRightBumperButton());
    }

    private Trigger driverLeftBumper() {
        return driverTrigger(() -> driver.getHID().getLeftBumperButton());
    }

    private Trigger driverAButton() {
        return driverTrigger(() -> driver.getHID().getAButton());
    }

    private Trigger driverBButton() {
        return driverTrigger(() -> driver.getHID().getBButton());
    }

    private Trigger driverXButton() {
        return driverTrigger(() -> driver.getHID().getXButton());
    }

    private Trigger driverYButton() {
        return driverTrigger(() -> driver.getHID().getYButton());
    }

    private Trigger driverBackButton() {
        return driverTrigger(() -> driver.getHID().getBackButton());
    }

    private Trigger driverPovUp() {
        return driverTrigger(() -> driver.getHID().getPOV() == 0);
    }

    private Trigger driverPovDown() {
        return driverTrigger(() -> driver.getHID().getPOV() == 180);
    }

    private Trigger driverPovRight() {
        return driverTrigger(() -> driver.getHID().getPOV() == 90);
    }

    private Trigger driverPovLeft() {
        return driverTrigger(() -> driver.getHID().getPOV() == 270);
    }

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));

    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    );
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        // Keep vision integration running continuously in every mode, including disabled, so the pose estimator converges before autonomous starts.
        limelight.setDefaultCommand(updateVisionCommand());

        // Home mechanisms at mode entry so later position commands have a known reference.
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        // Driver "fire" controls: trigger uses auto-aim path, bumper uses manual shooter RPM.
        driverRightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driverRightBumper().whileTrue(subsystemCommands.shootManually());
        // Driver "intake" controls.
        driverLeftTrigger().whileTrue(intake.intakeCommand());
        driverLeftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        // D-pad vertical controls hanger stages.
        driverPovUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driverPovDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        // D-pad horizontal currently cycles through predefined hood stages.
        driverPovRight().onTrue(Commands.runOnce(() -> hood.cycleStage()));
        driverPovLeft().onTrue(Commands.runOnce(() -> hood.cycleStage()));        
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);
        /* 
        driver.pov(180).onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        driver.pov(-90).onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        driver.pov(0).onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        driver.pov(90).onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero))); */
        // Re-zero the driver's forward direction without rebooting robot code.
        driver.back().onTrue(Commands.runOnce(manualDriveCommand::seedFieldCentric)); 
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                // Feed limelight pose into the drivetrain estimator with per-measurement covariance.
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });

            // Periodically feed opponent tracks back to the active tactical planner
            if (autoRoutines.getPlannerClient() instanceof BcnpTcpPlannerClient bcnp) {
                bcnp.sendOpponentUpdates(limelight.getOpponents());
            }
        })
        .ignoringDisable(true);
    }
}
