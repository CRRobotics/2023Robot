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
    //The current angle of the
    private double currentAngle;
    private XboxController controller;

    /**
     * Creates an indexer object given a motor id as an int.
     * @param indexerMotorID The motorId at which the indexer motor (VictorSPX) is connected.
     */
    public Indexer(int indexerMotorID){
        indexerMotor = new VictorSPX(indexerMotorID);
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        indexerMotor.setInverted(true);
        controller = new XboxController(0);
        indexerPID = new PIDController(Constants.Indexer.indexerP, Constants.Indexer.indexerI, Constants.Indexer.indexerD);
    }

    /**
     * Turns the spindexer to a specified angle in radians.
     * @param targetAngle The target angle in radians.
     */
    public void turnToAngle(double targetAngle){
        indexerMotor.set(ControlMode.Velocity, indexerPID.calculate(currentAngle, targetAngle));
    }

    /**
     * Sets the output velocity of the motor to the given percent.
     * @param velocity The given percent.
     */
    public void setIndexerVelocity(double velocity){
        indexerMotor.set(ControlMode.PercentOutput, velocity);
    }

    /**
     * Gets the angle of the spindexer in radians.
     * @return the angle of the spindexer in radians.
     */
    public double getAngle(){
        return currentAngle;
    }
}
