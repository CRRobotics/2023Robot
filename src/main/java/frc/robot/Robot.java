// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.misc.PieceType;

/**
 * Subsubsubclass of the <code>RobotBase</code> class from edu.wpi.first.wpilibj.RobotBase
 * This class is the iterative => timed version, meaning it operates using a command based system
 */
public class Robot extends TimedRobot {
  // OBJECTS
  // Active auto command based on the Smart Dashboard Sendable Chooser
  private Command autoCommand;
  // Robot container, contains keybindings to commands, subsystems, default commands, etc.
  private RobotContainer robotContainer;
  // Piece type selector for changing actions
  public static PieceType pieceType;



  // ROBOT
  /**
   * Method runs once at the time of the creation of the <code>Robot</code> class
   * Never have an infinite or while loop
   */
  @Override
  public void robotInit() {
    System.out.println("Robot initiated");
    robotContainer = new RobotContainer();
    pieceType = PieceType.Cone;
  }

  /**
   * Runs once every 50 milliseconds in every state.
   * This is roughly 4 times faster than the blink of a human eye.
   * Never have an infinite or while loop
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // Runs qued commands
    SmartDashboard.putString("PIECE MODE", pieceType == PieceType.Cone ? "CONE" : "CUBE");
  }



  // DISABLED STATE
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}



  // AUTONOMOUS STATE
  @Override
  public void autonomousInit() {
    System.out.println("Auto engaged");
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      System.out.println("Current auto path: " + autoCommand.getName());
      autoCommand.schedule();
    } else {
      System.out.println("Auto path not selected");
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}



  // TELEOP STATE
  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    robotContainer.getGrabber().removeDefaultCommand();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    robotContainer.getElevator().getElbowMotor().setNeutralMode(NeutralMode.Coast);
    robotContainer.getElevator().getWristMotor().setNeutralMode(NeutralMode.Coast);
  }



  // TEST STATE
  @Override
  public void testInit() {CommandScheduler.getInstance().cancelAll();}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}



  // HELPER METHODS
  public static PieceType getPieceType() {return pieceType;}

  public static void setPieceType(PieceType setPieceType) {pieceType = setPieceType;}

  public static void togglePieceType() {
    if (pieceType == PieceType.Cone) setPieceType(PieceType.Cube);
    else if (pieceType == PieceType.Cube) setPieceType(PieceType.Cone);
  }
}
