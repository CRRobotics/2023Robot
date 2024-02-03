// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Starts the *command based* robot structure
 */
public final class Main {
  private Main() {} // Only exists to prevent the instantiation of the <code>Main</code> class

  public static void main(String... args) // String... almost the same as String[], this is called a varargs parameter
  {
    RobotBase.startRobot(Robot::new); // Robot::new returns 
  }
}
