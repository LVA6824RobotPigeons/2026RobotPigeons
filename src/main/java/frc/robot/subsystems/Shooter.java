package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    // Allowed steady-state error before shooter is considered "ready".
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);


    private final TalonFX leftMotor, middleMotor, rightMotor;
    public final List<TalonFX> motors;
    private AngularVelocity targetVelocity = RPM.of(0);

    // Dashboard-adjustable fallback RPM for manual shooting mode.
    private double dashboardTargetRPM = 4000;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kCANivoreCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kCANivoreCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kCANivoreCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

        SmartDashboard.putData(this);
    }

    static TalonFXConfiguration createConfiguration(InvertedValue invertDirection) {
        return new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    // Prevent commanding negative shooter voltage (mechanically unnecessary here).
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        motor.getConfigurator().apply(createConfiguration(invertDirection));
    }    

    public void setRPM(double rpm) {
        targetVelocity = RPM.of(rpm);
        for (final TalonFX motor : motors) {
            motor.setControl(
                new VelocityVoltage(targetVelocity)
                    .withSlot(0)
            );
        }
    }

    public void setPercentOutput(double percentOutput) {
        final var output = Volts.of(percentOutput * 12.0);
        for (final TalonFX motor : motors) {
            motor.setControl(
                new VoltageOut(output)
            );
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        // One-shot setpoint command that completes only after all wheels stabilize.
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public double getTargetRpm() {
        return targetVelocity.in(RPM);
    }

    public double getAverageRpm() {
        return motors.stream()
            .mapToDouble(motor -> Math.abs(motor.getVelocity().refresh().getValue().in(RPM)))
            .average()
            .orElse(0.0);
    }

    public double getAverageSupplyCurrentAmps() {
        return motors.stream()
            .mapToDouble(motor -> motor.getSupplyCurrent().refresh().getValue().in(Amps))
            .average()
            .orElse(0.0);
    }

    public boolean isVelocityWithinTolerance() {
        // Valid readiness requires both: velocity mode is active and all motors are near target.
        return motors.stream().allMatch(motor -> {
            final ControlModeValue controlMode = motor.getControlMode(false).refresh().getValue();
            final boolean isInVelocityMode = switch (controlMode) {
                case VelocityVoltage, VelocityVoltageFOC -> true;
                default -> false;
            };
            final double currentRpsMagnitude = Math.abs(motor.getVelocity().refresh().getValue().in(RotationsPerSecond));
            final double targetRpsMagnitude = Math.abs(targetVelocity.in(RotationsPerSecond));
            final double toleranceRps = kVelocityTolerance.in(RotationsPerSecond);
            return isInVelocityMode && MathUtil.isNear(currentRpsMagnitude, targetRpsMagnitude, toleranceRps);
        });
    }

    List<TalonFX> motorsForTesting() {
        return motors;
    }

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> targetVelocity.in(RPM), null);
    }
}
