// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.AHRS;
public class DriveTrain extends SubsystemBase {
    // Create MAXSwerveModules
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.Drive.frontLeftWheelID,
            Constants.Drive.frontLeftTurnID,
            Constants.Drive.frontLeftAngularOffset);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.Drive.frontRightWheelID,
            Constants.Drive.frontRightTurnID,
            Constants.Drive.frontRightAngularOffset);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.Drive.backLeftWheelID,
            Constants.Drive.backLeftTurnID,
            Constants.Drive.backLeftAngularOffset);

    private final SwerveModule backRight = new SwerveModule(
            Constants.Drive.backRightWheelID,
            Constants.Drive.backRightTurnID,
            Constants.Drive.backRightAngularOffset);
        private final ArrayList<SwerveModule> modules = new ArrayList<>(Arrays.asList(frontLeft, frontRight, backLeft, backRight));

    // The gyro sensor
    private final AHRS gyro = new AHRS();

    // Odometry class for tracking robot pose
//     SwerveDriveOdometry odometry = new SwerveDriveOdometry(
//             Constants.Drive.driveKinematics,
//             Rotation2d.fromDegrees(gyro.getAngle()),
//         //     (SwerveModulePosition[]) modules.stream().map(SwerveModule::getPosition).collect(Collectors.toList()).toArray() // iterates through modules's position
//         new SwerveModulePosition[]{frontLeft.getPosition()}
//         //     new SwerveModulePosition[] {
//         //             frontLeft.getPosition(),
//         //             frontRight.getPosition(),
//         //             backLeft.getPosition(),
//         //             backRight.getPosition()
//         //     }
//             );

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
    }

//     @Override
//     public void periodic() {
//         // Update the odometry in the periodic block
//         odometry.update(
//                 Rotation2d.fromDegrees(gyro.getAngle()),
//                 // (SwerveModulePosition[]) modules.stream().map(SwerveModule::getPosition).collect(Collectors.toList()).toArray()); // iterates through modules's position
                
//         new SwerveModulePosition[]{frontLeft.getPosition()});
//     }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
//     public Pose2d getPose() {
//         return odometry.getPoseMeters();
//     }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
//     public void resetOdometry(Pose2d pose) {
//         odometry.resetPosition(
//                 Rotation2d.fromDegrees(gyro.getAngle()),
//                 // (SwerveModulePosition[]) modules.stream().map(SwerveModule::getPosition).collect(Collectors.toList()).toArray(),
                
//         new SwerveModulePosition[]{frontLeft.getPosition()},
//                 pose);
//     }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Adjust input based on max speed
        xSpeed *= Constants.Drive.maxSpeed;
        ySpeed *= Constants.Drive.maxSpeed;
        rot *= Constants.Drive.maxAngularSpeed;

        var swerveModuleStates = Constants.Drive.driveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getAngle()))
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.maxSpeed);
        for (int i = 0; i < modules.size(); i++) modules.get(i).setDesiredState(swerveModuleStates[i]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        if (modules.size() == 4) {
            frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Constants.Drive.maxSpeed);
                frontLeft.setDesiredState(desiredStates[0]);
                frontRight.setDesiredState(desiredStates[1]);
                backLeft.setDesiredState(desiredStates[2]);
                backRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        for (SwerveModule module : modules) module.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.Drive.gyroReversed ? -1.0 : 1.0);
    }

    public double getAngle() {
        return gyro.getAngle();
    }
}