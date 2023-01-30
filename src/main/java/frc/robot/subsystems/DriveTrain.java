// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.misc.NetworkTableWrapper;

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

    // The gyro sensor
    private final AHRS gyro = new AHRS();

    // Odometry class for tracking robot pose
    // SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    //         Constants.Drive.driveKinematics,
    //         Rotation2d.fromDegrees(gyro.getAngle()),
    //         new SwerveModulePosition[] {
    //                 frontLeft.getPosition(),
    //                 frontRight.getPosition(),
    //                 backLeft.getPosition(),
    //                 backRight.getPosition()
    //         }
    //         );
    
    // Kalman filter for tracking robot pose
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Drive.driveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d() // needs to be set based on auto path
        );

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
    }

    @Override
    public void periodic() {
        // update with encoder and gyroscope data
        poseEstimator.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });
        // update with visions data from these cameras ids:
        for (int i : new int[]{0, 2}) 
            poseEstimator.addVisionMeasurement(
                new Pose2d(
                    NetworkTableWrapper.getData(i, "rx"),
                    NetworkTableWrapper.getData(i, "ry"),
                    Rotation2d.fromDegrees(NetworkTableWrapper.getData(i, "theta"))
                    ),
                Timer.getFPGATimestamp(), // needs to be tested and calibrated
                VecBuilder.fill(0.1, 0.1, 0.1) // needs to be calibrated
                );
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(gyro.getAngle()),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                },
                pose);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
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
        return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    /**
     * Returns the turn rate of the robot from the gyroscope.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.Drive.gyroReversed ? -1.0 : 1.0);
    }
}