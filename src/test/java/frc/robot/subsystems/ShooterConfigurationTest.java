package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class ShooterConfigurationTest {
    private static final double EPS = 1e-9;

    @Test
    void createConfiguration_setsRequestedInversion() {
        final TalonFXConfiguration leftCfg = Shooter.createConfiguration(InvertedValue.CounterClockwise_Positive);
        final TalonFXConfiguration rightCfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertEquals(InvertedValue.CounterClockwise_Positive, leftCfg.MotorOutput.Inverted);
        assertEquals(InvertedValue.Clockwise_Positive, rightCfg.MotorOutput.Inverted);
    }

    @Test
    void createConfiguration_setsNeutralModeToCoast() {
        final TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);
        assertEquals(NeutralModeValue.Coast, cfg.MotorOutput.NeutralMode);
    }

    @Test
    void createConfiguration_disablesReverseVoltage() {
        final TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);
        assertEquals(0.0, cfg.Voltage.PeakReverseVoltage, EPS);
    }

    @Test
    void createConfiguration_setsCurrentLimits() {
        final TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertTrue(cfg.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(120.0, cfg.CurrentLimits.StatorCurrentLimit, EPS);
        assertTrue(cfg.CurrentLimits.SupplyCurrentLimitEnable);
        assertEquals(70.0, cfg.CurrentLimits.SupplyCurrentLimit, EPS);
    }

    @Test
    void createConfiguration_setsSlot0VelocityGains() {
        final TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertEquals(0.5, cfg.Slot0.kP, EPS);
        assertEquals(2.0, cfg.Slot0.kI, EPS);
        assertEquals(0.0, cfg.Slot0.kD, EPS);

        final double expectedKV = 12.0 / Constants.KrakenX60.kFreeSpeed.in(RotationsPerSecond);
        assertEquals(expectedKV, cfg.Slot0.kV, EPS);
    }
}
