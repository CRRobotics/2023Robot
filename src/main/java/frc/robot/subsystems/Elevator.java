package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Elevator extends SubsystemBase{
    private TalonFX armMotor1;
    private TalonFX armMotor2;
    private final PIDController elevatorPID;
    private final PIDController armPID1;
    private final PIDController armPID2;
    private CANSparkMax elevatorMotor;
    private boolean coneOrCube;
    private SparkMaxLimitSwitch bottomSwitch;
    //True for cone, false for cube
    private RelativeEncoder elevatorEncoder;
    private ControlMode control;
    public Elevator(int motorID, int motorID2, int motorID3){
        control = ControlMode.Position;
        coneOrCube = true;
        elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        armMotor1 = new TalonFX(motorID2);
        armMotor2 = new TalonFX(motorID3);
        elevatorPID = new PIDController(Constants.Elevator.elevatorP, Constants.Elevator.elevatorI, Constants.Elevator.elevatorD);
        armPID1 = new PIDController(0.001, 0, 0); //TODO make these constants
        armPID2 = new PIDController(0.001, 0, 0); //TODO make these constants
        elevatorEncoder = elevatorMotor.getEncoder();
    }

    public void setElevatorVelocity(double velocity)
    {
        elevatorMotor.set(velocity);
    }

    public void setElevatorPosition(double targetPosition)
    {
        elevatorMotor.set(elevatorPID.calculate(elevatorEncoder.getPosition() * Constants.Elevator.elevatorEncoderRate, targetPosition));
    }

    public void setConeOrCube()
    {
        if (coneOrCube) coneOrCube = false;
        else coneOrCube = true;
    }

    public boolean getConeOrCube()
    {
        return coneOrCube; 
    }

    public void setArmMotors(double pos1, double pos2){
        armMotor1.set(control, armPID1.calculate(pos1)) ;
        armMotor2.set(control,armPID2.calculate(pos2));
    }

}