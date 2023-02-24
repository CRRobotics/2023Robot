package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private SparkMaxLimitSwitch topSwitch;
    //True for cone, false for cube
    private RelativeEncoder elevatorEncoder;
    private ControlMode control;
    private XboxController controller;
    public Elevator(int motorID, int motorID2, int motorID3){
        
        control = ControlMode.PercentOutput;
        controller = new XboxController(2);
        coneOrCube = true;
        elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        armMotor1 = new TalonFX(motorID2);
        armMotor2 = new TalonFX(motorID3);
        armMotor1.setNeutralMode(NeutralMode.Brake);
        armMotor2.setNeutralMode(NeutralMode.Brake);
        elevatorPID = new PIDController(Constants.Elevator.elevatorP, Constants.Elevator.elevatorI, Constants.Elevator.elevatorD);
        armPID1 = new PIDController(0.005, 0, 0); //TODO make these constants
        armPID2 = new PIDController(0.005, 0, 0); //TODO make these constants
        //elevatorEncoder = elevatorMotor.getEncoder();
        //bottomSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        //bottomSwitch.enableLimitSwitch(true);
        //topSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        //topSwitch.enableLimitSwitch(true);
        armMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        armMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    public void setElevatorVelocity(double velocity)
    {
        elevatorMotor.set(velocity);
        System.out.println("VELOCITY"+ velocity);
    }

    public void setElevatorPosition(double targetPosition)
    {
       // elevatorMotor.set(elevatorPID.calculate(elevatorEncoder.getPosition() * Constants.Elevator.elevatorEncoderRate, targetPosition));
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
        System.out.println(armMotor1.getSelectedSensorPosition());
        //System.out.println(armPID1.calculate(armMotor1.getSelectedSensorPosition(), pos1));
        armMotor1.set(ControlMode.PercentOutput, controller.getLeftY() * 0.2);
        armMotor2.set(ControlMode.PercentOutput, controller.getRightY() * 0.1);
        //armMotor1.set(ControlMode.PercentOutput, armPID1.calculate(armMotor1.getSelectedSensorPosition(), pos1));
        //armMotor2.set(ControlMode.PercentOutput, armPID2.calculate(armMotor2.getSelectedSensorPosition(), pos2));
    }
}