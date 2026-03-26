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

class FloorHardwareTest {
    private Floor floor;
    private TalonFX motor;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setup() throws Exception {
        floor = new Floor();
        motor = readMotorField(floor, "motor");
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        motor.getSimState().setSupplyVoltage(12.0);
    }

    @AfterEach
    void cleanup() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void setFeed_usesVoltageControlMode() {
        floor.set(Floor.Speed.FEED);
        waitOneLoop();
        assertTrue(controlModeString(motor).contains("Voltage"));
    }

    @Test
    void feedCommand_startsAndStopsAsExpected() {
        final Command command = floor.feedCommand();
        CommandScheduler.getInstance().schedule(command);
        CommandScheduler.getInstance().run();
        waitOneLoop();
        assertTrue(controlModeString(motor).contains("Voltage"));

        CommandScheduler.getInstance().cancel(command);
        CommandScheduler.getInstance().run();
        waitOneLoop();
        assertTrue(controlModeString(motor).contains("Voltage"));
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
