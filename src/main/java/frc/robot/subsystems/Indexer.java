package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.misc.Constants;

public class Indexer{
    private final CANSparkMax indexerMotor;
    private final PIDController indexerPID;
    private final RelativeEncoder indexerEncoder;

    public Indexer(int indexerMotorID){
        indexerMotor = new CANSparkMax(indexerMotorID, MotorType.kBrushless);
        indexerEncoder = indexerMotor.getEncoder();
        indexerPID = new PIDController(Constants.Indexer.indexerP, Constants.Indexer.indexerI, Constants.Indexer.indexerD);
    }

    public void turnToAngle(double targetAngle){
        indexerMotor.set(indexerPID.calculate(indexerEncoder.getPosition() / 360.0, targetAngle));
    }

    public void setIndexerVelocity(double velocity){
        indexerMotor.set(velocity);
    }
}
