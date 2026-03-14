package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier yInput;
    private final DoubleSupplier xInput;

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.yInput = forwardInput;
        this.xInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {

        Ports.kCandle.setColor(
                Constants.LEDs.kCyan,
                4
        );

        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, yInput, xInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getState().Pose);
        // Aims and then drives, then prepares shot after 0.25 seconds, then waits until aimed and ready to shoot, then "feeds" for some reason
        // (All at the same time)
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed())
        );
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed()) // Feeds
            .handleInterrupt(shooter::stop); // Stops shooter
    }

    private Command feed() {
        // One after another
        return Commands.sequence(
            Commands.waitSeconds(0.25), // Waits
            Commands.parallel( // All at once
                feeder.feedCommand(), // Starts feeding
                Commands.waitSeconds(0.125) // Waits a lil
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand())) // Spits out
            )
        );
    }
}
