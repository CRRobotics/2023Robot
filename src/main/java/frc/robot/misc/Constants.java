package frc.robot.misc;

import com.revrobotics.CANSparkMax.IdleMode;

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
    }
}
