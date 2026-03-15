package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.*;

class ShooterConfigurationTest {
    private static final double EPS = 1e-9;

    @Test
    void createConfiguration_setsRequestedInversion() {
        TalonFXConfiguration leftCfg = Shooter.createConfiguration(InvertedValue.CounterClockwise_Positive);

        TalonFXConfiguration rightCfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertEquals(
                InvertedValue.CounterClockwise_Positive,
                leftCfg.MotorOutput.Inverted
        );

        assertEquals(
                InvertedValue.Clockwise_Positive,
                rightCfg.MotorOutput.Inverted
        );
    }

    @Test
    void createConfiguration_setsNeutralModeToCoast() {
        TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertEquals(
                NeutralModeValue.Coast,
                cfg.MotorOutput.NeutralMode
        );
    }

    @Test
    void createConfiguration_disablesReverseVoltage() {
        TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertEquals(
                0.0,
                cfg.Voltage.PeakReverseVoltage,
                EPS
        );
    }

    @Test
    void createConfigruation_setsCurrentLimits() {
        TalonFXConfiguration cfg = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertTrue(
                cfg.CurrentLimits.SupplyCurrentLimitEnable
        );

        assertEquals(
                120,
                cfg.CurrentLimits.StatorCurrentLimit,
                EPS
        );

        assertTrue(
                cfg.CurrentLimits.SupplyCurrentLimitEnable
        );

        assertEquals(
                70,
                cfg.CurrentLimits.SupplyCurrentLimit,
                EPS
        );
    }

    @Test
    void createConfiguration_setsSlot0VelocityGains() {
        TalonFXConfiguration CfgMcCfgFace = Shooter.createConfiguration(InvertedValue.Clockwise_Positive);

        assertEquals(
                0.5,
                CfgMcCfgFace.Slot0.kP, EPS
        );

        assertEquals(
                2.0,
                CfgMcCfgFace.Slot0.kI,
                EPS
        );

        assertEquals(
                0.0,
                CfgMcCfgFace.Slot0.kD,
                EPS
        );

        double expectedKV = 12.0 / Constants.KrakenX60.kFreeSpeed.in(RotationsPerSecond);

        assertEquals(
                expectedKV,CfgMcCfgFace.Slot0.kV,
                EPS
        );
    }
}