// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {

  public static Robot robot;

  // Weither not to use ControlsManager's Controller selection feature
  // Turn off if you are only using one controller, it becomes too
  // Annoying and obstructive to be useful
  public static final boolean use_cm_controller_selection = true;

  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    robot = new Robot();
    RobotBase.startRobot(robot::getThis);
  }
}
