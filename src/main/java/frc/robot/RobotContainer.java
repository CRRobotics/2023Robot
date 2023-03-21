// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto.OnePiece;
import frc.robot.commands.Auto.OnePiece2;
import frc.robot.commands.Auto.OnePieceBalance;
import frc.robot.commands.Auto.OnePieceOnePickupBalance;
import frc.robot.commands.Auto.TwoPiece;
import frc.robot.commands.Auto.TwoPieceBalance;
import frc.robot.commands.Auto.ZeroPiece;
import frc.robot.commands.Auto.ZeroPiece2;
import frc.robot.commands.Auto.ZeroPieceBalance;
import frc.robot.commands.Elevator.AcquireDoubleSub;
import frc.robot.commands.Elevator.AutoTop;
import frc.robot.commands.Elevator.FoldIn;
import frc.robot.commands.Elevator.GroundPickupCube;
import frc.robot.commands.Elevator.PlaceBottom;
import frc.robot.commands.Elevator.PlaceMid;
import frc.robot.commands.Elevator.PlaceTop;
import frc.robot.commands.Elevator.SetArmPosition;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.commands.drivetrain.BalanceRoutine;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.DriveStates;
import frc.robot.commands.drivetrain.DriveToPiece;
import frc.robot.commands.drivetrain.DriveToScoring;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Ungrab;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class RobotContainer {
  
  // SUBSYSTEMS
  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator = new Elevator();
  private final Grabber grabber = new Grabber();
  

  
  // INPUT
  XboxController driver = new XboxController(Constants.Controller.driveControllerPort);
  XboxController controller = new XboxController(1);
  
  
  
  // OTHER
  public static DriveStates driveStates = DriveStates.normal;
  
  public static SendableChooser<String> autoMode = new SendableChooser<>();
  // configure sendable chooser
  static {
    autoMode.setDefaultOption("1PieceBalance", "1PieceBalance");
    autoMode.addOption("OnePiece", "OnePiece");
    autoMode.addOption("OnePieceBalance", "OnePieceBalance");
    autoMode.addOption("OnePieceOnePickupBalance", "OnePieceOnePickupBalance");
    autoMode.addOption("TwoPiece", "TwoPiece");
    autoMode.addOption("TwoPieceBalance", "TwoPieceBalance");
    autoMode.addOption("ZeroPiece", "ZeroPiece");
    autoMode.addOption("ZeroPieceBalance", "ZeroPieceBalance");
    autoMode.addOption("PlaceTop", "PlaceTop");
    autoMode.addOption("PlaceMid", "PlaceMid");
    autoMode.addOption("PlaceLow", "PlaceLow");
    autoMode.addOption("ZeroPiece2", "ZeroPiece2");
    autoMode.addOption("OnePiece2", "OnePiece2");
    SmartDashboard.putData("Auto Mode", autoMode);
  }



  // CONSTRUCTOR
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // KEYBINDINGS
    configureButtonBindings();

    // SMART DASHBOARD
    configureSmartDashboard();
    
    // DEFAULT COMMANDS
    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain));
    grabber.setDefaultCommand(new Grab(grabber));
  }
  


  // METHODS
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto;
    switch (autoMode.getSelected()) {
      // SCORES ONE PIECE
      default:
        auto = new OnePiece(driveTrain, elevator, grabber);
        break;
      case "OnePiece":
        auto = new OnePiece(driveTrain, elevator, grabber);
        break;
      case "OnePieceOnePickupBalance":
        auto = new OnePieceOnePickupBalance(driveTrain, elevator, grabber);
        break;

      // SCORES TWO PIECES
      case "TwoPiece":
        auto = new TwoPiece(driveTrain, elevator, grabber);
        break;
      case "TwoPieceBalance":
        auto = new TwoPieceBalance(driveTrain, elevator, grabber);
        break;

      // SCORES ZERO PIECES
      case "ZeroPiece":
        auto = new ZeroPiece(driveTrain);
        break;
      case "ZeroPieceBalance":
        auto = new ZeroPieceBalance(driveTrain);
        break;

      // PLACE
      case "PlaceTop":
        auto = new AutoTop(elevator, grabber);
        break;
      // case "PlaceMid":
      //   auto = new AutoMid(elevator, grabber);
      //   break;
      case "PlaceLow":
        auto = new PlaceBottom(elevator, grabber);
        break;
      case "ZeroPiece2":
        auto = new ZeroPiece2(driveTrain);
        break;
      case "OnePiece2":
        auto = new OnePiece2(driveTrain, grabber, elevator);
    }
    return auto;
  }
  


  // HELPER METHODS
  /**
   * Define keybindings for commands
   */
  private void configureButtonBindings() {
    // DRIVER
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new DriveToScoring(driveTrain));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new DriveToPiece(driveTrain));
    new JoystickButton(driver, XboxController.Button.kX.value).whileTrue(new BalanceRoutine(driveTrain));
    new JoystickButton(driver, 5).whileTrue(new DriveSlow());
    new JoystickButton(driver, 6).whileTrue(new DriveFast());

    // CONTROLLER - elevator
    new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new PlaceTop(elevator));
    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new PlaceMid(elevator));
    new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new GroundPickupCube(elevator, grabber));
    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new AcquireDoubleSub(elevator));
    new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(new FoldIn(elevator));
    new JoystickButton(controller, XboxController.Button.kBack.value).onTrue(new GroundPickupCube(elevator, grabber));
    
    // CONTROLLER - grabber
    new JoystickButton(controller, XboxController.Button.kRightStick.value).onTrue(new InstantCommand(() -> {Robot.togglePieceType();}));
    new JoystickButton(controller, XboxController.Button.kLeftBumper.value).whileTrue(new Grab(grabber));
    new JoystickButton(controller, XboxController.Button.kRightBumper.value).whileTrue(new Ungrab(grabber));
  }

  /**
   * Push log info to Smart Dashboard
   */
  private void configureSmartDashboard() {
    // GRABBER
    SmartDashboard.putNumber("grabber/speed", 0.6);
    SmartDashboard.putNumber("grabber/current limit", 10);
    
    // DRIVETRAIN
    SmartDashboard.putNumber("drivetrain/xP", 0);
    SmartDashboard.putNumber("drivetrain/xI", 0);
    SmartDashboard.putNumber("drivetrain/xD", 0);

    SmartDashboard.putNumber("drivetrain/thetaP", 0);
    SmartDashboard.putNumber("drivetrain/thetaI", 0);
    SmartDashboard.putNumber("drivetrain/thetaD", 0);

    SmartDashboard.putNumber("drivetrain/balanceP", 0);
    SmartDashboard.putNumber("drivetrain/balanceI", 0);
    SmartDashboard.putNumber("drivetrain/balanceD", 0);

    // ELEVATOR
    SmartDashboard.putNumber("elevator/elevator setpoint", 0);
    SmartDashboard.putNumber("elevator/elbow setpoint", 0);
    SmartDashboard.putNumber("elevator/arm setpoint", 0);
  }
}