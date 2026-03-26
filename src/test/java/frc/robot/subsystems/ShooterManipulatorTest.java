package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShooterManipulatorTest {
    private Shooter shooter;
    private List<TalonFX> motors;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setup() {
        shooter = new Shooter();
        motors = shooter.motorsForTesting();

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        for (TalonFX motor : motors) {
            motor.getSimState().setSupplyVoltage(12.0);
        }
    }

    @AfterEach
    void cleanup() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void setPercentOutput_putsMotorsInVoltageControlMode() {
        shooter.setPercentOutput(0.5);
        waitForSignals();

        for (TalonFX motor : motors) {
            final String controlMode = motor.getControlMode(false).refresh().getValue().toString();
            assertTrue(controlMode.contains("Voltage"));
        }
    }

    @Test
    void setRPM_putsMotorsInVelocityControlMode() {
        shooter.setRPM(3000);
        waitForSignals();

        for (TalonFX motor : motors) {
            final String controlMode = motor.getControlMode(false).refresh().getValue().toString();
            assertTrue(controlMode.contains("Velocity"));
        }
    }

    @Test
    void isVelocityWithinTolerance_falseWhenNotInVelocityMode() {
        shooter.setPercentOutput(0.4);
        setAllMeasuredVelocityRpm(3000);

        assertFalse(shooter.isVelocityWithinTolerance());
    }

    @Test
    void isVelocityWithinTolerance_falseWheneverAnyMotorIsOutsideTolerance() {
        shooter.setRPM(3000);
        setAllMeasuredVelocityRpm(3000);
        setMeasuredVelocityRpm(1, 2600);

        assertFalse(shooter.isVelocityWithinTolerance());
    }

    @Test
    void isVelocityWithinTolerance_trueWhenAllMotorsAreNearTarget() {
        shooter.setRPM(3000);
        setMeasuredVelocityRpm(0, 2950);
        setMeasuredVelocityRpm(1, 3000);
        setMeasuredVelocityRpm(2, 3075);

        assertTrue(shooter.isVelocityWithinTolerance());
    }

    @Test
    void spinUpCommand_finishesOnlyAfterShooterReachesTolerance() {
        final CommandScheduler scheduler = CommandScheduler.getInstance();
        final Command command = shooter.spinUpCommand(3000);

        scheduler.schedule(command);
        scheduler.run();
        assertTrue(command.isScheduled());

        setMeasuredVelocityRpm(0, 3000);
        setMeasuredVelocityRpm(1, 3000);
        setMeasuredVelocityRpm(2, 2600);

        scheduler.run();
        assertTrue(command.isScheduled());

        setMeasuredVelocityRpm(2, 3000);

        scheduler.run();
        assertFalse(command.isScheduled());
    }

    private void setAllMeasuredVelocityRpm(double rpm) {
        final double rps = RPM.of(rpm).in(RotationsPerSecond);
        for (TalonFX motor : motors) {
            motor.getSimState().setRotorVelocity(rps);
        }
        waitForSignals();
    }

    private void setMeasuredVelocityRpm(int motorIndex, double rpm) {
        final double rps = RPM.of(rpm).in(RotationsPerSecond);
        motors.get(motorIndex).getSimState().setRotorVelocity(rps);
        waitForSignals();
    }

    private static void waitForSignals() {
        // One robot loop for sim values/control requests to propagate.
        Timer.delay(0.02);
    }
}
