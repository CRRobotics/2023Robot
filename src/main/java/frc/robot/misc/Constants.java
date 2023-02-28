package frc.robot.misc;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Constants {
    interface SwerveModule {
        double wheelDiameter = 0.0762; //meters
        double wheelTeeth = 14;
        // 45 teeth bevel gear, 22 teeth 1st stage spur gear, 15 teeth on bevel pinion gear
        double wheelGearRatio = (double)(45 * 22) / (wheelTeeth * 15.0);
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
        double turnEncoderPositionPIDMaxInput = turnEncoderPositionConversion;
        IdleMode turnIdleMode = IdleMode.kBrake;
        int turnCurrentLimit = 50; // amps
    }

    interface Drive {
        // all unset
        int frontLeftWheelID = 13;
        int frontLeftTurnID = 14;
        double frontLeftAngularOffset = -Math.PI / 2;

        int frontRightWheelID = 9;
        int frontRightTurnID = 10;
        double frontRightAngularOffset = 0;

        int backLeftWheelID = 5;
        int backLeftTurnID = 6;
        double backLeftAngularOffset = Math.PI;

        int backRightWheelID = 11;
        int backRightTurnID = 12;
        double backRightAngularOffset = Math.PI / 2;

        double trackWidth = Units.inchesToMeters(26.5);// Distance between centers of right and left wheels on robot//TODO Set kTrackWidth to actual track width
        double wheelBase = Units.inchesToMeters(26.5);// Distance between front and back wheels on robot //TODO Set kWheelBase to actual wheel base
        SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2),
                new Translation2d(wheelBase / 2, -trackWidth / 2),
                new Translation2d(-wheelBase / 2, trackWidth / 2),
                new Translation2d(-wheelBase / 2, -trackWidth / 2));        //Swerve Max Speed (copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java)
        double maxSpeed = 2; // meters per second
        double maxAngularSpeed = Math.PI; // radians per second;
        boolean gyroReversed = true; //Determines whether the gyro is reversed (I think)

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

    interface Elevator {
<<<<<<< Updated upstream
        int armMotor1ID = 16;
        double armMotor1P = 0.005;
        double armMotor1I = 0;
        double armMotor1D = 0;

        int armMotor2ID = 17;
        double armMotor2P = 0.005;
        double armMotor2I = 0;
        double armMotor2D = 0;

        int elevatorMotorID = 15;
        double elevatorP = 0.001;
        double elevatorI = 0;
        double elevatorD = 0;
=======
        double elevatorMotorID = 0;
        double elevatorP = SmartDashboard.getNumber("P", 0);
        double elevatorI = SmartDashboard.getNumber("I", 0);
        double elevatorD = SmartDashboard.getNumber("D", 0);
>>>>>>> Stashed changes
        double elevatorEncoderRate = 1;
    }
}
