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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.misc.NetworkTableWrapper;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
public class Grabber extends SubsystemBase implements Constants{
    TalonSRX motor = new TalonSRX(Grabber.motorID);

    /**
     * Sets the neutral mode to brake. Configures the factory defaults in constructor.
     */
    public Grabber()
    {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configContinuousCurrentLimit(3);
    }

    /**
     * Sets the position of the motor to the given position.
     * @param pos The new position of the grabber in encoder ticks.
     */
    public void setPos(double pos)
    {
        motor.set(ControlMode.Position, pos);
    }
    /**
     * sets the speed of the motor
     * @param speed Represents the speed of a percent of the motor capcity.
     */
    public void setSpeed(double speed)
    {
        motor.set(ControlMode.PercentOutput, speed);
    }
}