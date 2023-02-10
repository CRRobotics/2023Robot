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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.misc.NetworkTableWrapper;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.AHRS;
public class DriveTrain extends SubsystemBase {
    // Create MAXSwerveModules
    private final SwerveModule frontLeft = new SwerveModule( // chimera 11& 12
            Constants.Drive.chimeraWheelID,
            Constants.Drive.chimeraTurnID,
            Constants.Drive.frontLeftAngularOffset);

    private final SwerveModule frontRight = new SwerveModule( // manticore 9&10
            Constants.Drive.manticoreWheelID,
            Constants.Drive.manticoreTurnID,
            Constants.Drive.frontRightAngularOffset);

    private final SwerveModule backLeft = new SwerveModule( //phoenix 13&14
            Constants.Drive.phoenixWheelID,
            Constants.Drive.phoenixTurnID,
            Constants.Drive.backLeftAngularOffset);

    private final SwerveModule backRight = new SwerveModule( //Leviathan 5&6
            Constants.Drive.leviathanWheelID,
            Constants.Drive.leviathanTurnID,
            Constants.Drive.backRightAngularOffset);
// Cerberus 7&8
    // The gyro sensor
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Constants.Drive.driveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        }
    );
    
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
        new Pose2d(1.6, 4.4, Rotation2d.fromRadians(2.8)), // needs to be set based on auto path
        VecBuilder.fill(1, 1, 1),
        VecBuilder.fill(0.1, 0.1, 0.1)
    );

    private Field2d field = new Field2d();
    private Field2d odoField = new Field2d();

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        resetOdometry(new Pose2d(1.6, 4.4, Rotation2d.fromRadians(2.8)));
    }

    @Override
    public void periodic() {
        SwerveModulePosition[] swervePosition = {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
        
        poseEstimator.update(
            Rotation2d.fromDegrees(-gyro.getAngle()),
            swervePosition
        );

        odometry.update(
            Rotation2d.fromDegrees(-gyro.getAngle()),
            swervePosition
        );
        
        // update with visions data from these cameras ids:
        for (int i : new int[]{0, 2, 4}) if (NetworkTableWrapper.getData(i, "ntags") != 0) {
            poseEstimator.addVisionMeasurement(
                new Pose2d(
                    NetworkTableWrapper.getData(i, "rx"),
                    NetworkTableWrapper.getData(i, "ry"),
                    Rotation2d.fromRadians(NetworkTableWrapper.getData(i, "theta"))
                ),
                Timer.getFPGATimestamp(), // needs to be tested and calibrated
                VecBuilder.fill(0.05, 0.05, 0.05) // needs to be calibrated
            );
        }

        // field
        // odoField.setRobotPose(odometry.getPoseMeters());
        // SmartDashboard.putData(odoField);
        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("odoY", odometry.getPoseMeters().getY());

        SmartDashboard.putNumber("posex", poseEstimator.getEstimatedPosition().getX());
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
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drive.maxSpeed);
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