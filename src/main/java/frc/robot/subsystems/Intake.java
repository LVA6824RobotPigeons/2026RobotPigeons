package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.robot.subsystems.led8.LED8Implimentation;

public class Intake extends SubsystemBase { //makes it public
    public enum Speed { //speedy
        STOP(0), //you stop it
        INTAKE(0.8); //relative to how you control it

        private final double percentOutput; //control the persengtage out to the stop variable

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        } //set how fast it go, out of its maximum speed

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        } //sets  persentage for voltage
    }

    public enum Position { //points of areas
        HOMED(110),
        STOWED(100),
        INTAKE(-4),
        AGITATE(20);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }  //sets degrees and postitions

        public Angle angle() {
            return Degrees.of(degrees);
        } //angles!!
    }

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5); //angles with speed
    private static final double kHomingPercentOutput = 0.1;
    private static final double kHomingCurrentThresholdAmps = 6.0;
    private static final double kHomingTimeoutSeconds = 3.0;

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0); //adding motion to all of it

    private boolean isHomed = false;

    public Intake() {
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kCANivoreCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);
        configurePivotMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
    }//connecting motor with intake

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )//intake persentages
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )//conning intake to feedback
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(//maximum speed per secound
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        pivotMotor.getConfigurator().apply(config);
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )//setting up brakes
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );//confirgring the limits
        rollerMotor.getConfigurator().apply(config);
    }

    private boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    private void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        ); //something
    }

    public void set(Position position) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
    }

    public void set(Speed speed) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }  //setting speed and postion as a set control

    public Command intakeCommand() {
        return startEnd(
            () -> {
                LED8Implimentation.intakeOn();
                set(Position.INTAKE);
                set(Speed.INTAKE);
            },
            () -> {
                LED8Implimentation.intakeOff();
                Ports.kCandle.removeColor(30);
                set(Speed.STOP);
            }
        );
    }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> set(Position.AGITATE)),
                    Commands.waitUntil(this::isPositionWithinTolerance),
                    runOnce(() -> set(Position.INTAKE)),
                    Commands.waitUntil(this::isPositionWithinTolerance)
                ) //making a sequence opf putting postion relative to speed
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Position.INTAKE);
                set(Speed.STOP);
            });
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(kHomingPercentOutput)),
            Commands.waitUntil(this::isHomingCurrentReached)
                .withTimeout(kHomingTimeoutSeconds),
            Commands.either(
                runOnce(() -> {
                    pivotMotor.setPosition(Position.HOMED.angle());
                    isHomed = true;
                    set(Position.STOWED);
                }),
                runOnce(() -> setPivotPercentOutput(0)),
                this::isHomingCurrentReached
            ) //postion is wait and apply in amplitude
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private boolean isHomingCurrentReached() {
        return pivotMotor.getSupplyCurrent().getValue().in(Amps) > kHomingCurrentThresholdAmps;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}//finshed code with making everything connected to intake
//Comments made By Gabe
