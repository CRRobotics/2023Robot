// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Subsubsubclass of the <code>RobotBase</code> class from edu.wpi.first.wpilibj.RobotBase
 * This class is the iterative => timed version, meaning it operates using a command based system
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * Method runs once at the time of the creation of the <code>Robot</code> class
   * Never have an infinite or while loop
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * Runs once every 50 milliseconds. This is roughly 4 times faster than the blink of a human eye
   * Never have an infinite or while loop
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * Same as <code>robotInnit</code>, but only for the disabled state
   */
  @Override
  public void disabledInit() {}

  /**
   * Same as <code>robotPeriodic</code>, but only for the disabled state
   */
  @Override
  public void disabledPeriodic() {}

  /**
   * Same as <code>robotExit</code>, but only for the disabled state
   */
  @Override
  public void disabledExit() {}

  /**
   * Same as <code>robotInnit</code>, but only for the autonomous state
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Same as <code>robotPeriodic</code>, but only for the autonomous state
   */
  @Override
  public void autonomousPeriodic() {}

  /**
   * Same as <code>robotExit</code>, but only for the autonomous state
   */
  @Override
  public void autonomousExit() {}

  /**
   * Same as <code>robotInnit</code>, but only for the teleoperated state
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * Same as <code>robotPeriodic</code>, but only for the teleoperated state
   */
  @Override
  public void teleopPeriodic() {}

  /**
   * Same as <code>robotExit</code>, but only for the teleoperated state
   */
  @Override
  public void teleopExit() {}

  /**
   * Same as <code>robotInnit</code>, but only for the testing state
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Same as <code>robotPeriodic</code>, but only for the testing state
   */
  @Override
  public void testPeriodic() {}

  /**
   * Same as <code>robotExit</code>, but only for the testing state
   */
  @Override
  public void testExit() {}
}
