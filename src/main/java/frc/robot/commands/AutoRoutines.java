// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        /* add smart dash keys and init routine */
        autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {

        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot"); // Makes new routine
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence( // One after another
                startToOutpost.resetOdometry(), // Resets "movement tracker"
                startToOutpost.cmd() // Move to outpost (The area where balls are loaded by people
            )
        );

        routine.observe(hanger::isHomed).onTrue( // Only executed if hanger is homed
            Commands.sequence( // One after another
                Commands.waitSeconds(0.5)//, // Waits a bit (Very small amount)
//                intake.runOnce(() -> intake.set(Intake.Position.INTAKE))  // Move to intake position
            )
        );

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd()); // After 1 second, it drives to depot

        outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand()); // Starts intake 1 second before it arrives at depot
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd()); // Wait 0.1 second and then move to shooting position

        depotToShootingPose.active().whileTrue(limelight.idle()); // Goes to shooting move when limelight is idle
        depotToShootingPose.atTime(0.5).onTrue( // 0.5 seconds after command starts, it aims and then shoot
            Commands.parallel( // Both at once
                shooter.spinUpCommand(2600), // Starts spinning wheels to shoot
                hood.positionCommand(0.32) // Moves to position until within tolerance
            )
        );
        depotToShootingPose.done().onTrue( // Executes once in shooting position
            Commands.sequence( // One after another
                subsystemCommands.aimAndShoot()
                    .withTimeout(5), // Aims and shoots, but force killed after 5 seconds
                shootingPoseToTower.cmd() // Starts moving to tower
            )
        );

        shootingPoseToTower.active().whileTrue(limelight.idle()); // Move to tower when limelight is idle
        shootingPoseToTower.active().onTrue(hanger.positionCommand(Hanger.Position.HANGING)); // Activates tower command when in hanging position
        shootingPoseToTower.done().onTrue(hanger.positionCommand(Hanger.Position.HUNG)); // Finishes tower command when position is in hung

        return routine; // Finishes routine
    }
}
