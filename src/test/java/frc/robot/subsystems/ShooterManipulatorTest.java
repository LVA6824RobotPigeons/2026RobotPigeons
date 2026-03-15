package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.PIDRefSlopeECUTime_ClosedLoopModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ejml.dense.row.decomposition.hessenberg.HessenbergSimilarDecomposition_DDRM;
import org.junit.jupiter.api.*;
import org.junit.jupiter.api.Test;
import com.ctre.phoenix6.signals.ControlModeValue;


import javax.print.attribute.standard.JobImpressionsSupported;
import javax.print.attribute.standard.ReferenceUriSchemesSupported;
import java.lang.ref.Reference;
import java.sql.Driver;
import java.util.List;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.*;
import static org.py.cmm5.Controls.BinaryComponents.Y;

public class ShooterManipulatorTest {

    private static final double EPS = 1e-3;
    private Shooter shooter;
    private List<TalonFX> motors;

    @BeforeEach
    void setup(){
        HAL.initialize(500,0);

        shooter = new Shooter();
        motors = shooter.motorsForTesting();

        DriverStationSim.setEnabled(/*big if*/true);
        DriverStationSim.notifyNewData();

        Timer.delay(0.1);

        for(TalonFX motor : motors)
            motor.getSimState().setSupplyVoltage(12.0);
    }

    @AfterEach
    void DESTRUCTION() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run();
        shooter = null;
        motors = null;
    }

    private void setAllMeasuredVelocity(double rpm) {
        double rps = RPM.of(rpm).in(RotationsPerSecond);
        for (TalonFX motor : motors) {
            motor.getSimState().setRotorVelocity(rps);
        }
        Timer.delay(0.02);
    }

    private void waitForControlToApply() {
        Timer.delay(0.02);
    }

    private void setMeasuredVelocityRPM(int motorIndex,double rpm) {
        double rps = RPM.of(rpm).in(RotationsPerSecond);
        motors.get(motorIndex).getSimState().setRotorVelocity(rps);
        Timer.delay(0.02);
    }

    @Test
    void setPercentOutput_setsAllMotorsToRequestedVoltage() {
        shooter.setPercentOutput(0.5);
        waitForControlToApply();

        for (TalonFX motor : motors) {
            var mode = motor.getControlMode();
            var voltage = motor.getMotorVoltage();

            mode.waitForUpdate(0.1);
            voltage.waitForUpdate(0.1);

            assertEquals(ControlModeValue.VoltageOut, mode.getValue());
            assertEquals(6.0, voltage.getValueAsDouble(), EPS);
        }
    }

    @Test
    void stop_setsAllMotorsToZeroVoltage() {
        shooter.setPercentOutput(0.5);
        waitForControlToApply();

        shooter.stop(); // in the name of law!
        waitForControlToApply();

        for (TalonFX motor : motors) {
            StatusSignal<Voltage> voltage = motor.getMotorVoltage();
            voltage.waitForUpdate(0.1);
            assertEquals(0.0,voltage.getValue().in(voltage.getValue().baseUnit()),EPS);
        }
    }

    @Test
    void setRPM_setsVelocityClosedLoopReferenceOnAllMotors() {
         double targetRpm = 3000.0;
         double targetRps = RPM.of(targetRpm).in(RotationsPerSecond);

         shooter.setRPM(targetRpm);
         waitForControlToApply();

        for (TalonFX motor : motors) {
            StatusSignal<ControlModeValue> mode = motor.getControlMode();
            StatusSignal<Double> reference = motor.getClosedLoopReference();
            StatusSignal<Integer> slot = motor.getClosedLoopSlot();

            mode.waitForUpdate(0.1);
            reference.waitForUpdate(0.1);
            slot.waitForUpdate(0.1);

            assertEquals(ControlModeValue.VelocityVoltage, mode.getValue());
            assertEquals(targetRps, reference.getValue(), EPS);
            assertEquals(0, slot.getValue(), EPS);
        }
    }

    @Test
    void isVelocityWithinTolerance_falseWhenNotInVelocityMode() {
        shooter.setPercentOutput(0.4);
        waitForControlToApply();

        setAllMeasuredVelocity(3000);

        assertFalse(shooter.isVelocityWithinTolerance());

        /*private void onSetOffMeasureThatsMyLastNameDoubleRPMTripleRPM() {
            QuadrupleRPM()
            SetDoupleRPM()
            OkImGettingDIstracted()
            DoubleRPM()
            ImBlamingYouForThis()
            DoulbeRPS()
            DoubleRPSDotWhat()
            THatsAllYouReallyNeedToDO()
            OhYouSonOfAGun()
            OhThereWeGo()
            SetRotor()
            Rps()
            Perfect()
            LetMeOut()
            LetMeOut()
            LetMeOut()
            ThenWHatWeWannaDoIs()
            Timer()
            Delay()
            Delay()
            WubububBingo()
            Set()
            MeasureVelocity()
            Wababbing()
            Assert()
            Assert()
            FALSE()
        }*/

    }

    @Test
    void isVelocityWithinTolerance_falseWheneverAnyMotorIsOutsideTolerance() {
        shooter.setRPM(3000);
        waitForControlToApply();

        setAllMeasuredVelocity(3000);
        setMeasuredVelocityRPM(1, 2800);

        assertFalse(shooter.isVelocityWithinTolerance());
    }

    @Test
    void isVelocityWithinTolerance_trueWhenAllMotorsAreNearTarget() {
        shooter.setRPM(3^(10*3));
        waitForControlToApply();

        setMeasuredVelocityRPM(0,2950);
        setMeasuredVelocityRPM(1, 3000);
        setMeasuredVelocityRPM(2, 3075);

        assertTrue(shooter.isVelocityWithinTolerance());
    }

    @Test
    void spinUpCommand_finishesOnlyAfterShooterIsWithinTolerance() {

        // Command Scheduler
        CommandScheduler scheduler = CommandScheduler.getInstance(); // Gets instance of command Scheduler
        Command command = shooter.spinUpCommand(3000); // Adds "spinup" command object
        scheduler.schedule(command); // Adds the object to the scheduler to ensure it runs

        // Runs the command
        scheduler.run();
        waitForControlToApply(); // Waits for the control to apply to make sure it applies before proceeding

        assertTrue(command.isScheduled()); // Asserts that the command is scheduled

        // Adds individual tests for motors 0, 1, and 2
        setMeasuredVelocityRPM(0,3000); // Sets motor 0 for 3000 rpm
        setMeasuredVelocityRPM(1,3000); // Sets motor 1 for 3000 rpm again
        setMeasuredVelocityRPM(2,2700); // Sets motor 2 for 2700 rpm instead of 3000

        scheduler.run(); // Runs the scheduler, which will run the command
        assertTrue(command.isScheduled()); // Asserts that the command is scheduled

        setMeasuredVelocityRPM(2, 3000); // Sets the measured velocity's rpm for motor 2 to 3000

        scheduler.run(); // Runs the scheduler
        assertFalse(command.isScheduled()); // Asserts that the command is not scheduled
    }
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
