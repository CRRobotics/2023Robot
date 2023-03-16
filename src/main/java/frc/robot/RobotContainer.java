// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto.OnePieceBalance;
import frc.robot.commands.Auto.OnePieceOnePickupBalance;
import frc.robot.commands.Auto.PlaceTopDriveBackwards;
import frc.robot.commands.Auto.TwoPiece;
import frc.robot.commands.Auto.TwoPieceBalance;
import frc.robot.commands.Auto.ZeroPiece;
import frc.robot.commands.Auto.ZeroPieceBalance;
import frc.robot.commands.Elevator.AcquireDoubleSub;
import frc.robot.commands.Elevator.FoldIn;
import frc.robot.commands.Elevator.PlaceBottom;
import frc.robot.commands.Elevator.PlaceMid;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.commands.Elevator.ResetArmEncoders;
import frc.robot.commands.Elevator.SetArmPosition;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.DriveToPiece;
import frc.robot.commands.drivetrain.DriveToScoring;
import frc.robot.commands.drivetrain.TestModule;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class RobotContainer {
  // The robot's subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator = new Elevator();
  private final Grabber grabber = new Grabber();

  // The driver's controller
  XboxController driver = new XboxController(Constants.Controller.driveControllerPort);
  XboxController controller = new XboxController(1);

  public static SendableChooser<String> autoMode = new SendableChooser<>();

  static {
    autoMode.setDefaultOption("1PieceBalance", "1PieceBalance");
    autoMode.addOption("OnePiece", "OnePiece");
    autoMode.addOption("OnePieceBalance", "OnePieceBalance");
    autoMode.addOption("OnePieceOnePickupBalance", "OnePieceOnePickupBalance");
    autoMode.addOption("TwoPiece", "TwoPiece");
    autoMode.addOption("TwoPieceBalance", "TwoPieceBalance");
    autoMode.addOption("ZeroPiece", "ZeroPiece");
    autoMode.addOption("ZeroPieceBalance", "ZeroPieceBalance");
    SmartDashboard.putData("Auto Mode", autoMode);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    driveTrain.setDefaultCommand(
        new JoystickDrive(driveTrain));

    SmartDashboard.putNumber("elevator/elevator setpoint", 0);
    SmartDashboard.putNumber("elevator/elevator P", 0.003);
    SmartDashboard.putNumber("elevator/elevator I", 0);
    SmartDashboard.putNumber("elevator/elevator D", 0);
    SmartDashboard.putNumber("elevator/kg", 0.03);

    SmartDashboard.putNumber("elbow/elbow setpoint", 0);
    SmartDashboard.putNumber("elbow/elbow P", 0.04);
    SmartDashboard.putNumber("elbow/elbow I", 0);
    SmartDashboard.putNumber("elbow/elbow D", 0);

    SmartDashboard.putNumber("elbow/elbow position", 0);
    SmartDashboard.putNumber("elbow/elbow voltage", 0);
    SmartDashboard.putNumber("elbow/kg", 0);

    SmartDashboard.putNumber("wrist/wrist setpoint", 0);
    SmartDashboard.putNumber("wrist/wrist P", 0.08);
    SmartDashboard.putNumber("wrist/wrist I", 0);
    SmartDashboard.putNumber("wrist/wrist D", 0);

    SmartDashboard.putNumber("wrist/wrist position", 0);
    SmartDashboard.putNumber("wrist/wrist voltage", 0);
    SmartDashboard.putNumber("wrist/kg", 0.01);

    SmartDashboard.putNumber("grabber/speed", 1);
    

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new PlaceTop(elevator, grabber));
    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new PlaceMid(elevator, grabber));
    new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new PlaceBottom(elevator, grabber));
    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new AcquireDoubleSub(elevator));
    new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(new FoldIn(elevator));

    new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Grab(grabber));
    new JoystickButton(controller, XboxController.Button.kRightBumper.value)
      .whileTrue(new Ungrab(grabber));

    new JoystickButton(driver, XboxController.Button.kA.value)
      .whileTrue(new DriveToScoring(driveTrain));
    new JoystickButton(driver, XboxController.Button.kB.value)
      .whileTrue(new DriveToPiece(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    Command auto;
    switch (autoMode.getSelected()) {
      default:
        auto = new OnePieceBalance(driveTrain, elevator, grabber);
        break;
      case "OnePiece":
        auto = new OnePieceBalance(driveTrain, elevator, grabber);
        break;
      case "OnePieceOnePickupBalance":
        auto = new OnePieceOnePickupBalance(driveTrain, elevator, grabber);
        break;
      case "TwoPiece":
        auto = new TwoPiece(driveTrain, elevator, grabber);
        break;
      case "TwoPieceBalance":
        auto = new TwoPieceBalance(driveTrain, elevator, grabber);
        break;
      case "ZeroPiece":
        auto = new ZeroPiece(driveTrain, elevator, grabber);
        break;
      case "ZeroPieceBalance":
        auto = new ZeroPieceBalance(driveTrain);
        break;
    }
    return auto;
  }
}