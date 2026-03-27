package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// Concrete auto backed by WPILib commands/suppliers.
public final class CommandAutoPhase implements AutoPhase {
    private final String id;
    private final double timeoutSeconds;
    private final Command command;
    private final BooleanSupplier successCondition;
    private final Command fallbackCommand;

    public CommandAutoPhase(
            String id,
            double timeoutSeconds,
            Command command,
            BooleanSupplier successCondition,
            Command fallbackCommand) {
        this.id = id;
        this.timeoutSeconds = timeoutSeconds;
        this.command = command;
        this.successCondition = successCondition;
        this.fallbackCommand = fallbackCommand;
    }

    public static CommandAutoPhase of(
            String id,
            double timeoutSeconds,
            Command command,
            BooleanSupplier successCondition) {
        return new CommandAutoPhase(id, timeoutSeconds, command, successCondition, Commands.none());
    }

    @Override
    public String id() {
        return id;
    }

    @Override
    public double timeoutSeconds() {
        return timeoutSeconds;
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public BooleanSupplier successCondition() {
        return successCondition;
    }

    @Override
    public Command fallbackCommand() {
        return fallbackCommand;
    }
}
