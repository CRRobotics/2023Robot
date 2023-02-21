package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Indexer extends SubsystemBase {
    private final VictorSPX indexerMotor;
    private final PIDController indexerPID;
    private double currentAngle;
    private XboxController controller;

    public Indexer(int indexerMotorID){
        indexerMotor = new VictorSPX(indexerMotorID);
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        indexerMotor.setInverted(true);
        controller = new XboxController(0);
        indexerPID = new PIDController(Constants.Indexer.indexerP, Constants.Indexer.indexerI, Constants.Indexer.indexerD);
    }

    public void turnToAngle(double targetAngle){
        indexerMotor.set(ControlMode.Velocity, indexerPID.calculate(currentAngle, targetAngle));
    }

    public void setIndexerVelocity(double velocity){
        indexerMotor.set(ControlMode.PercentOutput, velocity);
    }

    public double getAngle(){
        return currentAngle;
    }
}