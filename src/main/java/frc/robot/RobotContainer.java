// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.TestModule;
import frc.robot.misc.Constants;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  // The robot's subsystems
  private final DriveTrain driveTrain = new DriveTrain();

  // The driver's controller
  XboxController driver = new XboxController(Constants.Controller.driveControllerPort);

  public DriveTrain getDriveTrain() {
      return driveTrain;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    driveTrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new JoystickDrive(driveTrain));
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
    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileTrue(new TestModule(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(
//         Constants.Auto.maxSpeed,
//         Constants.Auto.maxAcceleration)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(Constants.Drive.driveKinematics);

//     // An example trajectory to follow. All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         config);

//     var thetaController = new ProfiledPIDController(
//         Constants.Auto.thetaP, 0, 0, Constants.Auto.thetaPIDConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//     //     exampleTrajectory,
//     //     driveTrain::getPose, // Functional interface to feed supplier
//     //     Constants.Drive.driveKinematics,

//     //     // Position controllers
//     //     new PIDController(Constants.Auto.xP, 0, 0),
//     //     new PIDController(Constants.Auto.yP, 0, 0),
//     //     thetaController,
//     //     driveTrain::setModuleStates,
//     //     driveTrain);

//     // Reset odometry to the starting pose of the trajectory.
//     // driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

//     // // Run path following command, then stop at the end.
//     // return swerveControllerCommand.andThen(() -> driveTrain.drive(0, 0, 0, false));
//   }
}