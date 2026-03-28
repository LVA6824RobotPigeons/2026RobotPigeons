package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// One orchestrated autonomous phase w/ explicit success and fallback behavior.
public interface AutoPhase {
    String id();

    double timeoutSeconds();

    Command command();

    BooleanSupplier successCondition();

    default Command fallbackCommand() {
        return Commands.none();
    }
}
