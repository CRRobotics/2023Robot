package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;

public class Indexer extends SubsystemBase{
    private final CANSparkMax indexerMotor;
    private final PIDController indexerPID;
    private final RelativeEncoder indexerEncoder;
    private double currentAngle;

    public Indexer(int indexerMotorID){
        indexerMotor = new CANSparkMax(indexerMotorID, MotorType.kBrushless);
        indexerEncoder = indexerMotor.getEncoder();
        indexerPID = new PIDController(Constants.Indexer.indexerP, Constants.Indexer.indexerI, Constants.Indexer.indexerD);
    }

    public void turnToAngle(double targetAngle){
        indexerMotor.set(indexerPID.calculate(indexerEncoder.getPosition() / 2 * Math.PI, targetAngle));
        currentAngle = indexerEncoder.getPosition() / 2 * Math.PI;
    }

    public void setIndexerVelocity(double velocity){
        indexerMotor.set(velocity);
    }

    public double getAngle(){
        return currentAngle;
    }

    public RelativeEncoder getEncoder(){
        return indexerEncoder;
    }
}
