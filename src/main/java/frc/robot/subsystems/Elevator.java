package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Elevator extends SubsystemBase{
    private VictorSP armMotor;
    private final PIDController elevatorPID;
    private final PIDController armPID;
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    public Elevator(int motorID, int motorID2){
        elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        armMotor = new VictorSPX(motorID2);
        elevatorPID = new PIDController(Constants.Elevator.elevatorP, Constants.Elevator.elevatorI, Constants.Elevator.elevatorD);
        armPID = new PIDController(0.001, 0, 0); //TODO make these constants
        elevatorEncoder = elevatorMotor.getEncoder();
    }

    public void setElevatorVelocity(double velocity)
    {
        elevatorMotor.set(velocity);
    }

    public void setMotorPosition(double targetPosition)
    {
        elevatorMotor.set(elevatorPID.calculate(elevatorEncoder.getPosition() * Constants.Elevator.elevatorEncoderRate, targetPosition));
    }
}