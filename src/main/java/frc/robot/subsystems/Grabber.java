// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
public class Grabber extends SubsystemBase implements Constants{
    TalonFX motor = new TalonFX(Grabber.motorID);
    TalonFXConfiguration config;
    CurrentLimitsConfigs currentLimits;

    /**
     * Sets the neutral mode to brake. Configures the factory defaults in constructor.
     */
    public Grabber()
    {
        config = new TalonFXConfiguration();
        currentLimits = new CurrentLimitsConfigs();
        currentLimits.withSupplyCurrentLimit(3);
        config.withCurrentLimits(currentLimits);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * sets the speed of the motor
     * @param speed Represents the speed of a percent of the motor capcity.
     */
    public void setSpeed(double speed)
    {
        motor.set(speed);
    }

    public void setCurrentLimit(int currentLimit) {
        currentLimits.withSupplyCurrentLimit(currentLimit);
        config.withCurrentLimits(currentLimits);
        motor.getConfigurator().apply(config);
    }
}