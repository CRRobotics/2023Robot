package frc.robot.misc;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public interface Constants {
    interface SwerveModule {
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
    }

    interface Drive {
        // all unset
        int frontLeftWheelID = 0;
        int frontLeftTurnID = 0;
        double frontLeftAngularOffset = 0;

        int frontRightWheelID = 0;
        int frontRightTurnID = 0;
        double frontRightAngularOffset = 0;

        int backLeftWheelID = 0;
        int backLeftTurnID = 0;
        double backLeftAngularOffset = 0;

        int backRightWheelID = 0;
        int backRightTurnID = 0;
        double backRightAngularOffset = 0;

        double trackWidth = Units.inchesToMeters(26.5);// Distance between centers of right and left wheels on robot//TODO Set kTrackWidth to actual track width
        double wheelBase = Units.inchesToMeters(26.5);// Distance between front and back wheels on robot //TODO Set kWheelBase to actual wheel base
        SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2),
                new Translation2d(wheelBase / 2, -trackWidth / 2),
                new Translation2d(-wheelBase / 2, trackWidth / 2),
                new Translation2d(-wheelBase / 2, -trackWidth / 2));        //Swerve Max Speed (copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java)
        double maxSpeed = 3; // meters per second
        double maxAngularSpeed = 2 * Math.PI; // radians per second;
        boolean gyroReversed = false; //Determines whether the gyro is reversed (I think)

    }

    interface Controller {
        int driveControllerPort = 0;
    }

    interface Auto {
        double maxSpeed = 3; // meters per second
        double maxAcceleration = 3; // meters per second squared
        double maxAngularSpeed = Math.PI; // radians per second
        double maxAngularAcceleration = Math.PI; // radians per second squared
        double thetaP = 1; // pids for auto
        double xP = 1;
        double yP = 1;
        TrapezoidProfile.Constraints thetaPIDConstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeed, maxAngularAcceleration);
    }
}
