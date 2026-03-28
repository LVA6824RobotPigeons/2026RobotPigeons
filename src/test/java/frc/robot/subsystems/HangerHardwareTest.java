package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HangerHardwareTest {
    private Hanger hanger;
    private TalonFX motor;

    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setup() throws Exception {
        hanger = new Hanger();
        motor = readMotorField(hanger, "motor");
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        motor.getSimState().setSupplyVoltage(12.0);
    }

    @AfterEach
    void cleanup() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void setPosition_usesMotionMagicControlMode() {
        hanger.set(Hanger.Position.HANGING);
        waitOneLoop();
        assertTrue(controlModeString(motor).contains("MotionMagic"));
    }

    @Test
    void setPercentOutput_usesVoltageControlMode() {
        hanger.setPercentOutput(0.25);
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
