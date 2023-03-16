package frc.robot.misc;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public interface Constants {
    interface Indexer {
        double indexerP = 0.001; //TODO these values need to be tuned
        double indexerI = 0;
        double indexerD = 0;
        String indexerCamTable = "0";
    }
    interface SwerveModule {
        double wheelDiameter = 0.0762; //meters
        double wheelCircumference = Math.PI * wheelDiameter; // meters
        double wheelTeeth = 14;
        // 45 teeth bevel gear, 22 teeth 1st stage spur gear, 15 teeth on bevel pinion gear
        double wheelGearRatio = (45.0 * 22) / (wheelTeeth * 15.0);
        double wheelEncoderPositionConversion = wheelDiameter * Math.PI / wheelGearRatio; // meters
        double wheelEncoderVelocityConversion = wheelEncoderPositionConversion / 60.0; // meters per second
        double wheelMotorFreeSpeed = 5676.0 / 60; // rotations per second
        double wheelFreeSpeed = (wheelMotorFreeSpeed * wheelCircumference) / wheelGearRatio; // rotations per second
        double wheelP = 0.04;
        double wheelI = 0;
        double wheelD = 0;
        double wheelFF = 0.22194;
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
        int turnCurrentLimit = 20; // amps
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
                new Translation2d(-wheelBase / 2, -trackWidth / 2));//Swerve Max Speed (copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java)
        double maxSpeed = 4.8; // meters per second
        double maxAngularSpeed = 2 * Math.PI; // radians per second;
        boolean gyroReversed = true; //Determines whether the gyro is reversed (I think)

        double driveDeadBand = 0.025;

        double magnitudeSlewRate = 1.8; //rads per second
        double kDirectionSlewRate = 1.2; // percent per second (1 = 100%)
        double rotationSlewRate = 2.0; // percent per second (1 = 100%)
    }

    interface Grabber {
        int motorID = 18; //Placeholder
        double grabSpeed = 0.1;
    }

    interface Controller {
        int driveControllerPort = 0;
    }

    interface Auto {
        double maxSpeed = 3; // meters per second
        double maxAcceleration = 4; // meters per second squared
        PathConstraints constraints = new PathConstraints(maxSpeed, maxAcceleration);
        double maxAngularSpeed = Math.PI; // radians per second
        double maxAngularAcceleration = Math.PI; // radians per second squared
        double thetaP = 1; // pids for auto
        double xP = 1;
        double yP = 1;
        TrapezoidProfile.Constraints thetaPIDConstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeed, maxAngularAcceleration);
        double balanceP = 0;
        double balanceI = 0;
        double balanceD = 0;
        double balanceTolerance = 5;
    }

    interface Elevator {
        double allowableAngleError = 0.035;
        double allowableElevatorError = 0.02;
        int elevatorMotorID = 62;
        double elevatorP = 0.003;
        double elevatorI = 0;
        double elevatorD = 0;
        double elevatorKG = 0.03;
        double elevatorTicksPerMeter = 13434.639;
        int elevatorCurrentLimit = 40;
        double elevatorCalibrationSpeed = 0.01;
        double elevatorMaxVelocity = 1;
        double elevatorMaxAcceleration = 1;

        int elbowMotorID = 16;
        double elbowMotorP = 0.04;
        double elbowMotorI = 0;
        double elbowMotorD = 0;
        double elbowKG = 0.01;
        double elbowMotorCurrentLimit = 15;
        double elbowMotorVoltageLimit = 6;
        double elbowTicksPerDegree = 1422.639;
        double elbowTicksPerRadian = elbowTicksPerDegree * (180 / Math.PI);
        double elbowOffset = -158.7; // degrees

        double elbowMaxVelocity = 5;
        double elbowMaxAcceleration = 15;

        int wristMotorID = 17;
        double wristMotorP = 0.08;
        double wristMotorI = 0;
        double wristMotorD = 0;
        double wristKG = 0.015;
        double wristMotorCurrentLimit = 15;
        double wristMotorVoltageLimit = 6;
        double wristTicksPerDegree = 189.639;
        double wristTicksPerRadian = wristTicksPerDegree * (180 / Math.PI);
        double wristMaxVelocity = 5;
        double wristMaxAcceleration = 15;
        double wristOffset = 20.3; // degrees

        double highElevator = 0;
        double highElbow = 0;
        double highWrist = 0;

        double midElevator = 0;
        double midElbow = 0;
        double midWrist = 0;
        
        double lowElevator = 0;
        double lowElbow = 0;
        double lowWrist = 0;

        double indexerElevator = 0;
        double indexerElbow = 0;
        double indexerWrist = 0;

        double substationElevator = 0;
        double substationElbow = 0;
        double substationWrist = 0;

        double groundElevator = 0;
        double groundElbow = 0;
        double groundWrist = 0;
    }

    interface Acquisition {
        int highMotorID = 1;
        int lowMotorID = 2;
        double intakeSpeed = 0.25;
        double rejectSpeed = 0.25;
        double downPosition = 0;
    }
}
