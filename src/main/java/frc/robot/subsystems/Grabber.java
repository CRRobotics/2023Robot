// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
public class Grabber extends SubsystemBase implements Constants{
    TalonFX motor = new TalonFX(Grabber.motorID);

    /**
     * Sets the neutral mode to brake. Configures the factory defaults in constructor.
     */
    public Grabber()
    {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, 3,
            3, 0));
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

    public void setCurrentLimit(int currentLimit) {
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, currentLimit,
           currentLimit, 0));
    }
}