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

/**
 * Contains the arm and elevator assemblies
 */
public class Elevator extends SubsystemBase implements Constants{
    private TalonFX armMotor1; //Works with armMotor2
    private TalonFX armMotor2; //Works with armMotor1
    private final PIDController armPID1;
    private final PIDController armPID2;
    private CANSparkMax elevatorMotor; // Linear axis
    private final PIDController elevatorPID;
    private RelativeEncoder elevatorEncoder;

    private SparkMaxLimitSwitch topSwitch; //Limit switch on linear axis
    private SparkMaxLimitSwitch bottomSwitch; //Limit switch on linear axis

    private boolean coneOrCube; //True for cone, false for cube
    private ControlMode control;
    private XboxController controller;

    /**
     * Constructs an <code>Elevator</code> using motor ID's in <code>Contants.java</code>
     */
    public Elevator(){
        
        armMotor1 = new TalonFX(Elevator.armMotor1ID);
        armMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        armMotor1.setNeutralMode(NeutralMode.Brake);
        armPID1 = new PIDController(Elevator.armMotor1P, Elevator.armMotor1I, Elevator.armMotor1D); //TODO make these constants

        armMotor2 = new TalonFX(Elevator.armMotor2ID);
        armMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        armMotor2.setNeutralMode(NeutralMode.Brake);
        armPID2 = new PIDController(Elevator.armMotor1P, Elevator.armMotor1I, Elevator.armMotor1D); //TODO make these constants

        elevatorMotor = new CANSparkMax(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        //elevatorEncoder = elevatorMotor.getEncoder();
        elevatorPID = new PIDController(Constants.Elevator.elevatorP, Constants.Elevator.elevatorI, Constants.Elevator.elevatorD);

        //bottomSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        //bottomSwitch.enableLimitSwitch(true);
        //topSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        //topSwitch.enableLimitSwitch(true);

        coneOrCube = true;
        control = ControlMode.PercentOutput;
        controller = new XboxController(2);
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