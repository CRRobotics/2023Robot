// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.Auto.TuneRotation;
import frc.robot.commands.Auto.TuneTranslation;
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
  // The robot's subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator = new Elevator();
  private final Grabber grabber = new Grabber();

  // The driver's controller
  public static DriveStates driveStates = DriveStates.normal;

  XboxController driver = new XboxController(Constants.Controller.driveControllerPort);
  XboxController controller = new XboxController(1);

  

  public static SendableChooser<String> autoMode = new SendableChooser<>();
  public Elevator getElevator(){
    return elevator;
  }

  public Grabber getGrabber() {
    return grabber;
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

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
    autoMode.addOption("Tune Translation", "Tune Translation");
    autoMode.addOption("Tune Rotation", "Tune Rotation");
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
    grabber.setDefaultCommand(new Grab(grabber));

    SmartDashboard.putNumber("grabber/speed", 0.6);
    SmartDashboard.putNumber("grabber/current limit", 10);
    
    SmartDashboard.putNumber("drivetrain/xP", 0);
    SmartDashboard.putNumber("drivetrain/xI", 0);
    SmartDashboard.putNumber("drivetrain/xD", 0);
    SmartDashboard.putNumber("drivetrain/thetaP", 0);
    SmartDashboard.putNumber("drivetrain/thetaI", 0);
    SmartDashboard.putNumber("drivetrain/thetaD", 0);

    SmartDashboard.putNumber("drivetrain/balanceP", 0);
    SmartDashboard.putNumber("drivetrain/balanceI", 0);
    SmartDashboard.putNumber("drivetrain/balanceD", 0);

    SmartDashboard.putNumber("elevator/elevator setpoint", 0);
    SmartDashboard.putNumber("elevator/elbow setpoint", 0);
    SmartDashboard.putNumber("elevator/arm setpoint", 0);
  
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

    new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new PlaceTop(elevator));
    new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new PlaceMid(elevator));
    new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new GroundPickupCube(elevator, grabber));
    new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new AcquireDoubleSub(elevator));
    new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(new FoldIn(elevator));
    new JoystickButton(controller, XboxController.Button.kBack.value).onTrue(new GroundPickupCube(elevator, grabber));

    new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Grab(grabber));
    new JoystickButton(controller, XboxController.Button.kRightBumper.value)
      .whileTrue(new Ungrab(grabber));
    new JoystickButton(controller, XboxController.Button.kRightStick.value)
      .onTrue(new InstantCommand(() -> {Robot.togglePieceType();}));

new JoystickButton(driver, XboxController.Button.kA.value)
  .whileTrue(new DriveToScoring(driveTrain));
new JoystickButton(driver, XboxController.Button.kB.value)
  .whileTrue(new DriveToPiece(driveTrain));
new JoystickButton(driver, XboxController.Button.kX.value)
  .whileTrue(new BalanceRoutine(driveTrain));
new JoystickButton(driver, 6).whileTrue(new DriveSlow());
new JoystickButton(driver, 5).whileTrue(new DriveFast());
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
        auto = new OnePiece(driveTrain, elevator, grabber);
        break;
      case "OnePiece":
        auto = new OnePiece(driveTrain, elevator, grabber);
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
        auto = new ZeroPiece(driveTrain);
        break;
      case "ZeroPieceBalance":
        auto = new ZeroPieceBalance(driveTrain);
        break;
      case "PlaceTop":
        auto = new AutoTop(elevator, grabber);
        break;
      // case "PlaceMid":
        // auto = new AutoMid(elevator, grabber);
        // break;
      case "PlaceLow":
        auto = new PlaceBottom(elevator, grabber);
        break;
      case "ZeroPiece2":
        auto = new ZeroPiece2(driveTrain);
        break;
      case "OnePiece2":
        auto = new OnePiece2(driveTrain, grabber, elevator);
        break;
      case "Tune Translation":
        auto = new TuneTranslation(driveTrain);
        break;
      case "Tune Rotation":
        auto = new TuneRotation(driveTrain);
        break;
    }
    return auto;
  }

  public static Pose2d getInitialPose(String pathName, boolean mirrored) {
    Pose2d path = PathPlanner.loadPath(pathName, Constants.Auto.constraints).getInitialPose();
    if (!mirrored) return path;
    else return new Pose2d(new Translation2d(16.5, 0), new Rotation2d(Math.PI)).minus(path);
  }
}