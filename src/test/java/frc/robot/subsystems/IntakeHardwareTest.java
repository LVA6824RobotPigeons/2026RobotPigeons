package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeHardwareTest {
    private Intake intake;
    private TalonFX pivotMotor;
    private TalonFX rollerMotor;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setup() throws Exception {
        intake = new Intake();
        pivotMotor = readMotorField(intake, "pivotMotor");
        rollerMotor = readMotorField(intake, "rollerMotor");

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        pivotMotor.getSimState().setSupplyVoltage(12.0);
        rollerMotor.getSimState().setSupplyVoltage(12.0);
    }

    @AfterEach
    void cleanup() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void setPosition_usesMotionMagicOnPivot() {
        intake.set(Intake.Position.INTAKE);
        waitOneLoop();
        assertTrue(controlModeString(pivotMotor).contains("MotionMagic"));
    }

    @Test
    void setSpeed_usesVoltageOnRoller() {
        intake.set(Intake.Speed.INTAKE);
        waitOneLoop();
        assertTrue(controlModeString(rollerMotor).contains("Voltage"));
    }

    @Test
    void intakeCommand_startsAndStopsRollerAsExpected() {
        final Command command = intake.intakeCommand();
        CommandScheduler.getInstance().schedule(command);
        CommandScheduler.getInstance().run();
        waitOneLoop();

        assertTrue(controlModeString(pivotMotor).contains("MotionMagic"));
        assertTrue(controlModeString(rollerMotor).contains("Voltage"));

        CommandScheduler.getInstance().cancel(command);
        CommandScheduler.getInstance().run();
        waitOneLoop();

        assertTrue(controlModeString(rollerMotor).contains("Voltage"));
    }

    private static TalonFX readMotorField(Object instance, String fieldName) throws Exception {
        final Field field = instance.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        return (TalonFX) field.get(instance);
    }

    private static String controlModeString(TalonFX motor) {
        return motor.getControlMode(false).refresh().getValue().toString();
    }

    private static void waitOneLoop() {
        Timer.delay(0.02);
    }
}
