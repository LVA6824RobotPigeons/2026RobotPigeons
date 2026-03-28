// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED8Implimentation;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    public final RobotContainer m_robotContainer;
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */

    public Robot() {
        //Main.robot = this;
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        //Main.robot = this;
        LED8Implimentation.robotStart();
        // Initialize startup LED state before subsystems begin scheduling commands.
  
        // RobotContainer owns subsystem construction, default commands, and trigger bindings.
        m_robotContainer = new RobotContainer();

        // Expose scheduler for debugging command ownership/live state.
        SmartDashboard.putData(CommandScheduler.getInstance());
        // Slightly lower brownout threshold to reduce brownout trips during brief current spikes.
        RobotController.setBrownoutVoltage(Volts.of(6.1));


    }
    
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        m_robotContainer.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        m_robotContainer.resetFuelDetector();
        LED8Implimentation.teleopMode();
    }
    @Override
    public void teleopExit() {
        LED8Implimentation.teleopOff();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.resetFuelDetector();
        LED8Implimentation.autoMode();
    }
    @Override
    public void autonomousExit() {
        LED8Implimentation.autoOff();
    }

}
