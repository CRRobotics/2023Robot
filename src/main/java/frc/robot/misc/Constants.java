package frc.robot.misc;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public interface Constants {
    interface DriveConstants {
        double wheelDiameter = 0.0762; //meters
        double wheelTeeth = 14;
        // 45 teeth bevel gear, 22 teeth 1st stage spur gear, 15 teeth on bevel pinion gear
        double wheelGearRatio = (double)(45 * 22) / (wheelTeeth * 15);
        double wheelEncoderPositionConversion = wheelDiameter * Math.PI; // meters
        double wheelEncoderVelocityConversion = wheelEncoderPositionConversion / wheelGearRatio / 60; // meters per second
        double wheelP = 0.04;
        double wheelI = 0;
        double wheelD = 0;
        double wheelFF = 0;
        double wheelOutputMin = -1;
        double wheelOutputMax = 1;
        IdleMode wheelIdleMode = IdleMode.kBrake;
        int wheelCurrentLimit = 50; // amps

        boolean turnInverted = true;
        double turnEncoderPositionConversion = Math.PI * 2; // radians
        double turnEncoderVelocityConversion = Math.PI * 2 / 60; // radians per seconds
        double turnP = 1;
        double turnI = 0;
        double turnD = 0;
        double turnFF = 0;
        double turnOutputMin = -1;
        double turnOutputMax = 1;
        double turnEncoderPositionPIDMinInput = 0;
        IdleMode turnIdleMode = IdleMode.kBrake;
        int turnCurrentLimit = 50; // amps
        //New constants that are new I guess (if they are 0 it is unset)
        //Front Left Swerve
        int kFrontLeftDrivingCanId = 0;
        int kFrontLeftTurningCanId = 0;
        double kFrontLeftChassisAngularOffset = 0;
        //Front Right Swerve
        int kFrontRightDrivingCanId = 0;
        int kFrontRightTurningCanId = 0;
        double kFrontRightChassisAngularOffset = 0;
        //Rear Left Swerve
        int kRearLeftDrivingCanId = 0;
        int kRearLeftTurningCanId = 0;
        double kBackLeftChassisAngularOffset = 0;
        //Rear Right Swerve
        int kRearRightDrivingCanId = 0;
        int kRearRightTurningCanId = 0;
        double kBackRightChassisAngularOffset = 0;
        //SwerveDriveKinematics Object
        public static final double kTrackWidth = Units.inchesToMeters(26.5);// Distance between centers of right and left wheels on robot//TODO Set kTrackWidth to actual track width
        double kWheelBase = Units.inchesToMeters(26.5);// Distance between front and back wheels on robot //TODO Set kWheelBase to actual wheel base
        SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));        //Swerve Max Speed (copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java)
        double kMaxSpeedMetersPerSecond = 3;
        double kMaxAngularSpeed = 2 * Math.PI; // radians per second;
        boolean kGyroReversed = false; //Determines whether the gyro is reversed (I think)
    }
}
