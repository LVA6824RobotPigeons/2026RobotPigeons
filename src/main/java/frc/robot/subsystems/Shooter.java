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
//import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);


    private final TalonFX leftMotor, middleMotor, rightMotor;
    public final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;
    private boolean enabled = false;           // shooter starts OFF
    private static final double MIN_RPM = 1000; 
    private static final double MAX_RPM = 6000;

    public Shooter() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kCANivoreCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kCANivoreCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kCANivoreCANBus);
        motors = List.of(leftMotor, middleMotor, rightMotor);

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

        SmartDashboard.putData(this);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
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
        
        motor.getConfigurator().apply(config);
    }    

    public double getCurrentRPM() {
        return motors.get(0).getVelocity().getValue().in(RPM); // assumes all motors roughly same
    }

    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                velocityRequest
                    .withVelocity(RPM.of(rpm))
            );
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }

public void setRPMFromTrigger(double triggerValue) {
        if (!enabled) return; // ignore trigger if shooter is OFF
    
        // deadband for tiny values
        if (triggerValue < 0.05) triggerValue = 0;
    
        double desiredRPM = MIN_RPM + (MAX_RPM - MIN_RPM) * triggerValue;
        double currentRPM = getCurrentRPM();
    
        // Only increase RPM if below desired; otherwise coast naturally
        if (desiredRPM > currentRPM) {
            setRPM(desiredRPM);
        }
    
        System.out.printf("Trigger: %.2f | Desired RPM: %.0f | Current RPM: %.0f%n",
                          triggerValue, desiredRPM, currentRPM);
    }

    public void toggleEnabled() {
        enabled = !enabled;
    
        if (!enabled) {
            stop(); // stops motors completely
            System.out.println("Shooter OFF");
        } else {
            setRPM(MIN_RPM); // idle at 1000 RPM
            System.out.println("Shooter ENABLED (IDLE 1000 RPM)");
        }
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpCommand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM)); 
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

/*     public AngularVelocity getVelocity(int index) {
        return motors.get(index).getVelocity().getValue();
    } */

    private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
        builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }

/*     List<TalonFX> motorsForTesting() {
        return motors;
    } */

    @Override
    public void initSendable(SendableBuilder builder) {
        initSendable(builder, leftMotor, "Left");
        initSendable(builder, middleMotor, "Middle");
        initSendable(builder, rightMotor, "Right");
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Dashboard RPM", () -> dashboardTargetRPM, value -> dashboardTargetRPM = value);
        builder.addDoubleProperty("Target RPM", () -> velocityRequest.getVelocityMeasure().in(RPM), null);
    }
}
