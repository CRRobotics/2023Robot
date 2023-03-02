package frc.robot.misc;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Constants {
    interface Indexer {
        double indexerP = 0.001; //TODO these values need to be tuned
        double indexerI = 0;
        double indexerD = 0;
        int indexerCamTable = 0;
    }
    interface SwerveModule {
        double wheelDiameter = 0.0762; //meters
        double wheelTeeth = 14;
        // 45 teeth bevel gear, 22 teeth 1st stage spur gear, 15 teeth on bevel pinion gear
        double wheelGearRatio = (45.0 * 22) / (wheelTeeth * 15.0);
        double wheelEncoderPositionConversion = wheelDiameter * Math.PI / wheelGearRatio; // meters
        double wheelEncoderVelocityConversion = wheelEncoderPositionConversion / 60.0; // meters per second
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
        int chimeraWheelID = 11;
        int chimeraTurnID = 12;
        double frontLeftAngularOffset = -Math.PI / 2;

        int manticoreWheelID = 9;
        int manticoreTurnID = 10;
        double frontRightAngularOffset = 0;

        int phoenixWheelID = 13;
        int phoenixTurnID = 14;
        double backLeftAngularOffset = Math.PI;

        int leviathanWheelID = 5;
        int leviathanTurnID = 6;
        double backRightAngularOffset = Math.PI / 2;

        double trackWidth = Units.inchesToMeters(26.5);// Distance between centers of right and left wheels on robot//TODO Set kTrackWidth to actual track width
        double wheelBase = Units.inchesToMeters(26.5);// Distance between front and back wheels on robot //TODO Set kWheelBase to actual wheel base
        SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2),
                new Translation2d(wheelBase / 2, -trackWidth / 2),
                new Translation2d(-wheelBase / 2, trackWidth / 2),
                new Translation2d(-wheelBase / 2, -trackWidth / 2));        //Swerve Max Speed (copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java)
        double maxSpeed = 1; // meters per second
        double maxAngularSpeed = Math.PI; // radians per second;
        boolean gyroReversed = true; //Determines whether the gyro is reversed (I think)

    }

    interface Grabber {
        int motorID = 18; //Placeholder
    }

    interface Controller {
        int driveControllerPort = 1;
    }

    interface Auto {
        double maxSpeed = 0.25; // meters per second
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
        int elbowMotorID = 16;
        double elbowMotorP = 0.005;
        double elbowMotorI = 0;
        double elbowMotorD = 0;
        double elbowMotorCurrentLimit = 15;
        double elbowMotorVoltageLimit = 6;

        int wristMotorID = 17;
        double wristMotorP = 0.005;
        double wristMotorI = 0;
        double wristMotorD = 0;
        double wristMotorCurrentLimit = 15;
        double wristMotorVoltageLimit = 6;

        int elevatorMotorID = 62;
        double elevatorP = 0.001;
        double elevatorI = 0;
        double elevatorD = 0;
        double elevatorEncoderRate = 1;
        int elevatorCurrentLimit = 10;
    interface Acquisition {
        int acquisitionMotor1 = 1;
        int acquisitionMotor2 = 2;
        double acquisitionDown = Math.PI;
    }
}
